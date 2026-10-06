package scenegraph

// NodeID 是节点的唯一标识，由调用方提供以保证可复现与幂等导入。
type NodeID string

// Node 是场景图中的一个节点。
//
// 所有访问器内部获取场景读锁，因此与并发提交同时调用是安全的，
// 读到的永远是某次已提交状态的字段值，不会出现撕裂。
type Node struct {
	id          NodeID
	local       Mat4
	world       Mat4
	localBounds AABB
	worldBounds AABB
	parent      *Node
	children    []*Node
	scene       *Scene
}

// ID 返回节点标识。ID 在节点生命周期内不变，无需加锁。
func (n *Node) ID() NodeID { return n.id }

// Local 返回局部变换。
func (n *Node) Local() Mat4 {
	n.scene.mu.RLock()
	defer n.scene.mu.RUnlock()
	return n.local
}

// World 返回世界变换（反映最近一次已提交的更新）。
func (n *Node) World() Mat4 {
	n.scene.mu.RLock()
	defer n.scene.mu.RUnlock()
	return n.world
}

// LocalBounds 返回节点自身（局部空间）包围盒。
func (n *Node) LocalBounds() AABB {
	n.scene.mu.RLock()
	defer n.scene.mu.RUnlock()
	return n.localBounds
}

// WorldBounds 返回世界空间包围盒。
func (n *Node) WorldBounds() AABB {
	n.scene.mu.RLock()
	defer n.scene.mu.RUnlock()
	return n.worldBounds
}

// Parent 返回父节点；根节点或已摘除的节点返回 nil。
func (n *Node) Parent() *Node {
	n.scene.mu.RLock()
	defer n.scene.mu.RUnlock()
	return n.parent
}

// Children 按挂载顺序返回子节点快照。
func (n *Node) Children() []*Node {
	n.scene.mu.RLock()
	defer n.scene.mu.RUnlock()
	out := make([]*Node, len(n.children))
	copy(out, n.children)
	return out
}
