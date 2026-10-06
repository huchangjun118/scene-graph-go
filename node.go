package scenegraph

// NodeID 为节点标识，由调用方提供，需在场景内唯一。
type NodeID string

// Transform 为节点的局部变换（平移、旋转、缩放）。
type Transform struct {
	Position Vec3
	Rotation Quat
	Scale    Vec3
}

// IdentityTransform 返回单位局部变换（缩放为 1）。
func IdentityTransform() Transform {
	return Transform{Rotation: QuatIdentity(), Scale: Vec3{1, 1, 1}}
}

// Matrix 返回局部变换对应的仿射矩阵。
func (t Transform) Matrix() Mat4 {
	return Mat4Compose(t.Position, t.Rotation, t.Scale)
}

// Node 为场景节点快照（只读视图）。场景内部状态不直接暴露，
// 所有查询返回的都是提交时刻的拷贝，可安全在锁外使用。
type Node struct {
	ID       NodeID
	Parent   NodeID // 根节点的 Parent 为空
	Children []NodeID
	Local    Transform
	World    Mat4 // 最近一次提交时计算的世界变换
	Bounds   AABB // 局部空间包围盒
	WorldBox AABB // 世界空间包围盒（World 作用于 Bounds）
}

// node 为场景内部节点。
type node struct {
	id       NodeID
	parent   *node
	children []*node // 按挂载顺序，保证遍历确定性
	local    Transform
	bounds   AABB
	world    Mat4
	worldBox AABB
}

// snapshot 生成节点的只读拷贝。
func (n *node) snapshot() Node {
	children := make([]NodeID, len(n.children))
	for i, c := range n.children {
		children[i] = c.id
	}
	var pid NodeID
	if n.parent != nil {
		pid = n.parent.id
	}
	return Node{
		ID:       n.id,
		Parent:   pid,
		Children: children,
		Local:    n.local,
		World:    n.world,
		Bounds:   n.bounds,
		WorldBox: n.worldBox,
	}
}

// recomputeWorld 由父世界矩阵重算自身世界变换，并递归传播到全部后代。
// 只访问以 n 为根的子树，保证增量更新不影响其余节点。
func (n *node) recomputeWorld(parentWorld Mat4) {
	n.world = parentWorld.Mul(n.local.Matrix())
	n.worldBox = n.bounds.Transform(n.world)
	for _, c := range n.children {
		c.recomputeWorld(n.world)
	}
}
