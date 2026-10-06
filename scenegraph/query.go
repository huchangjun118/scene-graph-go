package scenegraph

// collect 在读锁内按确定性的深度优先顺序（子节点按挂载顺序）
// 收集满足 keep 的节点。keep 为 nil 时收集全部节点。
func (s *Scene) collect(keep func(n *Node) bool) []*Node {
	s.mu.RLock()
	defer s.mu.RUnlock()
	var out []*Node
	var walk func(n *Node)
	walk = func(n *Node) {
		if keep == nil || keep(n) {
			out = append(out, n)
		}
		for _, c := range n.children {
			walk(c)
		}
	}
	walk(s.root)
	return out
}

// VisitAll 按确定性的深度优先顺序（子节点按挂载顺序）访问所有节点。
// fn 返回 false 时停止遍历。
//
// 遍历快照在读锁内一次性建立，fn 在读锁释放后调用，
// 因此 fn 内可以自由调用节点的加锁访问器而不会死锁。
func (s *Scene) VisitAll(fn func(n *Node) bool) {
	for _, n := range s.collect(nil) {
		if !fn(n) {
			return
		}
	}
}

// QueryRegion 返回世界包围盒与给定区域相交的全部节点，顺序与 VisitAll 一致。
// 根节点（无包围盒语义，若未设置则为空盒）同样参与判定。
func (s *Scene) QueryRegion(region AABB) []*Node {
	return s.collect(func(n *Node) bool {
		return n.worldBounds.Intersects(region)
	})
}

// QueryFrustum 返回世界包围盒与视锥相交的全部节点，顺序与 VisitAll 一致。
func (s *Scene) QueryFrustum(f Frustum) []*Node {
	return s.collect(func(n *Node) bool {
		return f.IntersectsAABB(n.worldBounds)
	})
}

// Hit 是一次射线命中结果。
type Hit struct {
	Node *Node
	T    float64 // 沿射线方向的命中距离（起点在盒内时为 0）
}

// RayCast 返回射线的最近命中节点；无命中返回 ok=false。
// 命中判定基于节点的世界包围盒，语义见 AABB.IntersectRay。
// 距离相同（并列）时按 VisitAll 顺序取先者，保证确定性。
func (s *Scene) RayCast(r Ray) (Hit, bool) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	var best Hit
	found := false
	var walk func(n *Node)
	walk = func(n *Node) {
		if t, ok := n.worldBounds.IntersectRay(r); ok && (!found || t < best.T) {
			best = Hit{Node: n, T: t}
			found = true
		}
		for _, c := range n.children {
			walk(c)
		}
	}
	walk(s.root)
	return best, found
}
