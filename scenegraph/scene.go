package scenegraph

import (
	"context"
	"errors"
	"sync"
)

// RootID 是根节点的固定标识。
const RootID NodeID = ""

// 错误定义。
var (
	ErrNodeNotFound = errors.New("scenegraph: node not found")
	ErrNodeExists   = errors.New("scenegraph: node already exists")
	ErrCycle        = errors.New("scenegraph: attach would create a cycle")
	ErrRootMutation = errors.New("scenegraph: cannot remove or re-attach root")
)

// Scene 是场景图，支持多读单写并发：
// 所有查询在内部持有读锁，所有修改通过 Tx.Commit 在写锁内原子完成。
type Scene struct {
	mu    sync.RWMutex
	root  *Node
	nodes map[NodeID]*Node
	rev   uint64
}

// New 创建只含根节点的空场景。
func New() *Scene {
	s := &Scene{nodes: make(map[NodeID]*Node)}
	root := &Node{
		id:          RootID,
		local:       Identity(),
		world:       Identity(),
		localBounds: EmptyAABB(),
		worldBounds: EmptyAABB(),
	}
	root.scene = s
	s.root = root
	s.nodes[RootID] = root
	return s
}

// Root 返回根节点。
func (s *Scene) Root() *Node { return s.root }

// Get 按 ID 查找节点；不存在返回 nil。
func (s *Scene) Get(id NodeID) *Node {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.nodes[id]
}

// Revision 返回场景修订号，每次成功提交使其递增。
func (s *Scene) Revision() uint64 {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.rev
}

// recomputeSubtree 重新计算 n 及其全部后代的世界变换与世界包围盒。
// 调用方必须持有写锁。只有受影响的子树会被触碰。
func recomputeSubtree(n *Node) {
	if n.parent != nil {
		n.world = n.parent.world.Mul(n.local)
	} else {
		n.world = n.local
	}
	n.worldBounds = n.localBounds.Transformed(n.world)
	for _, c := range n.children {
		recomputeSubtree(c)
	}
}

// isDescendant 报告 n 是否为 ancestor 的后代（含自身）。
func isDescendant(n, ancestor *Node) bool {
	for p := n; p != nil; p = p.parent {
		if p == ancestor {
			return true
		}
	}
	return false
}

// removeFromParent 把 n 从父节点的子列表中摘除，返回原下标。
func removeFromParent(n *Node) int {
	if n.parent == nil {
		return -1
	}
	siblings := n.parent.children
	for i, c := range siblings {
		if c == n {
			n.parent.children = append(siblings[:i:i], siblings[i+1:]...)
			return i
		}
	}
	return -1
}

// unregisterSubtree 把子树内全部节点从索引中移除。
func unregisterSubtree(s *Scene, n *Node) {
	delete(s.nodes, n.id)
	for _, c := range n.children {
		unregisterSubtree(s, c)
	}
}

// Tx 是一批待提交的修改。Tx 不是并发安全的。
type Tx struct {
	scene *Scene
	ops   []txOp
}

type txOp struct {
	kind   opKind
	id     NodeID
	parent NodeID
	local  Mat4
	bounds AABB
}

type opKind int

const (
	opAdd opKind = iota
	opRemove
	opAttach
	opSetLocal
	opSetBounds
)

// Begin 开启一个事务。
func (s *Scene) Begin() *Tx { return &Tx{scene: s} }

// Add 记录“新增节点”操作。parent 为空串（RootID）表示挂到根。
func (t *Tx) Add(id NodeID, parent NodeID, local Mat4, bounds AABB) {
	t.ops = append(t.ops, txOp{kind: opAdd, id: id, parent: parent, local: local, bounds: bounds})
}

// Remove 记录“移除节点及其子树”操作。
func (t *Tx) Remove(id NodeID) {
	t.ops = append(t.ops, txOp{kind: opRemove, id: id})
}

// Attach 记录“把节点重挂载到新父节点”操作，保持其在兄弟中的相对语义为“追加到末尾”。
func (t *Tx) Attach(id NodeID, newParent NodeID) {
	t.ops = append(t.ops, txOp{kind: opAttach, id: id, parent: newParent})
}

// SetLocal 记录“设置局部变换”操作。
func (t *Tx) SetLocal(id NodeID, local Mat4) {
	t.ops = append(t.ops, txOp{kind: opSetLocal, id: id, local: local})
}

// SetBounds 记录“设置局部包围盒”操作。
func (t *Tx) SetBounds(id NodeID, bounds AABB) {
	t.ops = append(t.ops, txOp{kind: opSetBounds, id: id, bounds: bounds})
}

// undo 记录一条可回滚动作。
type undo struct {
	op     txOp
	node   *Node
	parent *Node
	index  int
	local  Mat4
	bounds AABB
}

// Commit 校验并原子地应用事务中的所有操作。
// 若 ctx 已取消或任一操作校验失败，场景完全保持原状（已应用的操作回滚）。
// 成功时场景修订号加一。
func (t *Tx) Commit(ctx context.Context) error {
	s := t.scene
	if err := ctx.Err(); err != nil {
		return err
	}
	if len(t.ops) == 0 {
		return nil // 空事务：不触碰场景，也不递增修订号
	}
	s.mu.Lock()
	defer s.mu.Unlock()

	var undos []undo
	rollback := func() {
		for i := len(undos) - 1; i >= 0; i-- {
			u := undos[i]
			switch u.op.kind {
			case opAdd:
				idx := removeFromParent(u.node)
				_ = idx
				unregisterSubtree(s, u.node)
			case opRemove:
				u.node.parent = u.parent
				ch := u.parent.children
				if u.index >= len(ch) {
					u.parent.children = append(ch, u.node)
				} else {
					ch = append(ch, nil)
					copy(ch[u.index+1:], ch[u.index:])
					ch[u.index] = u.node
					u.parent.children = ch
				}
				registerSubtree(s, u.node)
				recomputeSubtree(u.node)
			case opAttach:
				removeFromParent(u.node)
				u.node.parent = u.parent
				ch := u.parent.children
				if u.index >= len(ch) {
					u.parent.children = append(ch, u.node)
				} else {
					ch = append(ch, nil)
					copy(ch[u.index+1:], ch[u.index:])
					ch[u.index] = u.node
					u.parent.children = ch
				}
				recomputeSubtree(u.node)
			case opSetLocal:
				u.node.local = u.local
				recomputeSubtree(u.node)
			case opSetBounds:
				u.node.localBounds = u.bounds
				u.node.worldBounds = u.node.localBounds.Transformed(u.node.world)
			}
		}
	}

	for _, op := range t.ops {
		if err := ctx.Err(); err != nil {
			rollback()
			return err
		}
		u, err := s.applyOp(op)
		if err != nil {
			rollback()
			return err
		}
		undos = append(undos, u)
	}
	s.rev++
	return nil
}

// registerSubtree 把子树内全部节点加入索引。
func registerSubtree(s *Scene, n *Node) {
	s.nodes[n.id] = n
	for _, c := range n.children {
		registerSubtree(s, c)
	}
}

// applyOp 应用单个操作并返回其回滚记录。调用方必须持有写锁。
func (s *Scene) applyOp(op txOp) (undo, error) {
	switch op.kind {
	case opAdd:
		if op.id == RootID {
			return undo{}, ErrNodeExists
		}
		if _, ok := s.nodes[op.id]; ok {
			return undo{}, ErrNodeExists
		}
		parent, ok := s.nodes[op.parent]
		if !ok {
			return undo{}, ErrNodeNotFound
		}
		n := &Node{
			id:          op.id,
			local:       op.local,
			localBounds: op.bounds,
			parent:      parent,
			scene:       s,
		}
		parent.children = append(parent.children, n)
		s.nodes[op.id] = n
		recomputeSubtree(n)
		return undo{op: op, node: n}, nil

	case opRemove:
		n, ok := s.nodes[op.id]
		if !ok {
			return undo{}, ErrNodeNotFound
		}
		if n == s.root {
			return undo{}, ErrRootMutation
		}
		parent := n.parent
		idx := removeFromParent(n)
		n.parent = nil
		unregisterSubtree(s, n)
		return undo{op: op, node: n, parent: parent, index: idx}, nil

	case opAttach:
		n, ok := s.nodes[op.id]
		if !ok {
			return undo{}, ErrNodeNotFound
		}
		if n == s.root {
			return undo{}, ErrRootMutation
		}
		newParent, ok := s.nodes[op.parent]
		if !ok {
			return undo{}, ErrNodeNotFound
		}
		if isDescendant(newParent, n) {
			return undo{}, ErrCycle
		}
		oldParent := n.parent
		idx := removeFromParent(n)
		n.parent = newParent
		newParent.children = append(newParent.children, n)
		recomputeSubtree(n)
		return undo{op: op, node: n, parent: oldParent, index: idx}, nil

	case opSetLocal:
		n, ok := s.nodes[op.id]
		if !ok {
			return undo{}, ErrNodeNotFound
		}
		old := n.local
		n.local = op.local
		recomputeSubtree(n)
		return undo{op: op, node: n, local: old}, nil

	case opSetBounds:
		n, ok := s.nodes[op.id]
		if !ok {
			return undo{}, ErrNodeNotFound
		}
		old := n.localBounds
		n.localBounds = op.bounds
		n.worldBounds = n.localBounds.Transformed(n.world)
		return undo{op: op, node: n, bounds: old}, nil
	}
	return undo{}, nil
}
