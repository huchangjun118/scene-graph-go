package scenegraph

import (
	"errors"
	"fmt"
	"sync"
)

// 场景操作错误。
var (
	ErrNodeExists   = errors.New("scenegraph: node already exists")
	ErrNodeNotFound = errors.New("scenegraph: node not found")
	ErrCycle        = errors.New("scenegraph: attach would create a cycle")
	ErrAttachRoot   = errors.New("scenegraph: cannot attach a node to itself")
)

// Scene 为场景图内核，支持多读单写并发。
// 所有查询在读锁下执行，看到的是最近一次完整提交的状态，
// 不会读到撕裂的世界变换；所有变更在写锁下原子完成。
type Scene struct {
	mu    sync.RWMutex
	nodes map[NodeID]*node
	roots []*node // 根节点列表，按成为根的顺序，保证遍历确定性
}

// NewScene 创建空场景。
func NewScene() *Scene {
	return &Scene{nodes: make(map[NodeID]*node)}
}

// Add 新增节点（初始为根节点）。ID 已存在时返回 ErrNodeExists。
func (s *Scene) Add(id NodeID, local Transform, bounds AABB) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.addLocked(id, local, bounds)
}

func (s *Scene) addLocked(id NodeID, local Transform, bounds AABB) error {
	if _, ok := s.nodes[id]; ok {
		return fmt.Errorf("%w: %q", ErrNodeExists, id)
	}
	n := &node{id: id, local: local, bounds: bounds}
	s.nodes[id] = n
	s.roots = append(s.roots, n)
	n.recomputeWorld(Mat4Identity())
	return nil
}

// Remove 移除节点及其整个子树。被移除子树中的节点不再存在于场景中。
func (s *Scene) Remove(id NodeID) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	n, ok := s.nodes[id]
	if !ok {
		return fmt.Errorf("%w: %q", ErrNodeNotFound, id)
	}
	s.detachLocked(n)
	for _, d := range n.collectSubtree() {
		delete(s.nodes, d.id)
	}
	return nil
}

// collectSubtree 按先序遍历返回子树全部节点（含自身）。
func (n *node) collectSubtree() []*node {
	out := []*node{n}
	for _, c := range n.children {
		out = append(out, c.collectSubtree()...)
	}
	return out
}

// detachLocked 将节点从父节点（或根列表）摘下，不调整其世界变换，
// 调用方负责随后重算。
func (s *Scene) detachLocked(n *node) {
	if n.parent != nil {
		p := n.parent
		for i, c := range p.children {
			if c == n {
				p.children = append(p.children[:i], p.children[i+1:]...)
				break
			}
		}
		n.parent = nil
		return
	}
	for i, r := range s.roots {
		if r == n {
			s.roots = append(s.roots[:i], s.roots[i+1:]...)
			break
		}
	}
}

// Attach 将 child 挂载到 parent 下。child 的整个子树随之移动，
// 世界变换按新父节点重算。若挂载会形成环（parent 是 child 自身或其后代），
// 返回 ErrCycle 且场景保持原状。
func (s *Scene) Attach(child, parent NodeID) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.attachLocked(child, parent)
}

func (s *Scene) attachLocked(child, parent NodeID) error {
	if child == parent {
		return fmt.Errorf("%w: %q", ErrAttachRoot, child)
	}
	c, ok := s.nodes[child]
	if !ok {
		return fmt.Errorf("%w: %q", ErrNodeNotFound, child)
	}
	p, ok := s.nodes[parent]
	if !ok {
		return fmt.Errorf("%w: %q", ErrNodeNotFound, parent)
	}
	// 环检测：沿 parent 的祖先链向上找，若遇到 child 则成环。
	// 检测在任何结构修改之前完成，失败时场景不变。
	for a := p; a != nil; a = a.parent {
		if a == c {
			return fmt.Errorf("%w: attach %q under %q", ErrCycle, child, parent)
		}
	}
	if c.parent == p {
		return nil // 已挂载在目标父节点下，幂等
	}
	s.detachLocked(c)
	c.parent = p
	p.children = append(p.children, c)
	c.recomputeWorld(p.world)
	return nil
}

// Detach 将节点从父节点摘下，成为根节点；其子树跟随移动。
// 摘下后节点局部变换不变，世界变换按根（单位父变换）重算。
func (s *Scene) Detach(id NodeID) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	n, ok := s.nodes[id]
	if !ok {
		return fmt.Errorf("%w: %q", ErrNodeNotFound, id)
	}
	if n.parent == nil {
		return nil // 已是根，幂等
	}
	s.detachLocked(n)
	s.roots = append(s.roots, n)
	n.recomputeWorld(Mat4Identity())
	return nil
}

// SetLocal 设置节点局部变换，仅重算该节点子树的世界变换（增量更新），
// 其余节点的世界变换与查询结果不受影响。
func (s *Scene) SetLocal(id NodeID, t Transform) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	n, ok := s.nodes[id]
	if !ok {
		return fmt.Errorf("%w: %q", ErrNodeNotFound, id)
	}
	n.local = t
	pw := Mat4Identity()
	if n.parent != nil {
		pw = n.parent.world
	}
	n.recomputeWorld(pw)
	return nil
}

// SetBounds 设置节点局部包围盒，重算该节点世界包围盒（不影响变换）。
func (s *Scene) SetBounds(id NodeID, b AABB) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	n, ok := s.nodes[id]
	if !ok {
		return fmt.Errorf("%w: %q", ErrNodeNotFound, id)
	}
	n.bounds = b
	n.worldBox = b.Transform(n.world)
	return nil
}

// Get 返回节点快照；不存在时 ok=false。
func (s *Scene) Get(id NodeID) (Node, bool) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	n, ok := s.nodes[id]
	if !ok {
		return Node{}, false
	}
	return n.snapshot(), true
}

// Roots 按确定顺序返回根节点 ID 列表。
func (s *Scene) Roots() []NodeID {
	s.mu.RLock()
	defer s.mu.RUnlock()
	out := make([]NodeID, len(s.roots))
	for i, r := range s.roots {
		out[i] = r.id
	}
	return out
}

// Update 以事务方式执行一批变更：fn 在场景的私有副本上执行，
// 仅当 fn 返回 nil 时变更才原子提交；fn 返回错误（取消）时场景保持原状，
// 不会停留在中间状态。提交与回滚对并发读者均不可见中间态。
func (s *Scene) Update(fn func(tx *Scene) error) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	tx := s.cloneUnlocked()
	if err := fn(tx); err != nil {
		return err // 副本被丢弃，原场景不变
	}
	s.nodes = tx.nodes
	s.roots = tx.roots
	return nil
}

// cloneUnlocked 深拷贝场景图（调用方需持有写锁）。
func (s *Scene) cloneUnlocked() *Scene {
	c := &Scene{nodes: make(map[NodeID]*node, len(s.nodes))}
	// 第一遍：创建所有节点副本。
	for id, n := range s.nodes {
		c.nodes[id] = &node{
			id:       id,
			local:    n.local,
			bounds:   n.bounds,
			world:    n.world,
			worldBox: n.worldBox,
		}
	}
	// 第二遍：按原顺序重建父子关系与根列表。
	for id, n := range s.nodes {
		cn := c.nodes[id]
		if n.parent != nil {
			cn.parent = c.nodes[n.parent.id]
		}
		cn.children = make([]*node, len(n.children))
		for i, ch := range n.children {
			cn.children[i] = c.nodes[ch.id]
		}
	}
	c.roots = make([]*node, len(s.roots))
	for i, r := range s.roots {
		c.roots[i] = c.nodes[r.id]
	}
	return c
}

// walkLocked 按确定顺序（根列表顺序 + 先序遍历）访问全部节点。
func (s *Scene) walkLocked(fn func(n *node)) {
	var visit func(n *node)
	visit = func(n *node) {
		fn(n)
		for _, c := range n.children {
			visit(c)
		}
	}
	for _, r := range s.roots {
		visit(r)
	}
}
