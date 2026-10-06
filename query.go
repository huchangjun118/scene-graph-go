package scenegraph

// QueryRegion 返回世界包围盒与给定区域相交（含接触）的全部节点 ID。
// 返回顺序为场景确定遍历顺序（根列表顺序 + 先序），
// 在固定输入序列下逐条可复现。
func (s *Scene) QueryRegion(region AABB) []NodeID {
	s.mu.RLock()
	defer s.mu.RUnlock()
	var out []NodeID
	s.walkLocked(func(n *node) {
		if n.worldBox.Intersects(region) {
			out = append(out, n.id)
		}
	})
	return out
}

// QueryFrustum 返回世界包围盒与视锥相交或在视锥内的全部节点 ID。
// 判定为保守判定（包围盒可能被裁剪平面切角时仍视为命中），
// 返回顺序与 QueryRegion 相同，确定可复现。
func (s *Scene) QueryFrustum(f Frustum) []NodeID {
	s.mu.RLock()
	defer s.mu.RUnlock()
	var out []NodeID
	s.walkLocked(func(n *node) {
		if f.IntersectsAABB(n.worldBox) {
			out = append(out, n.id)
		}
	})
	return out
}

// Raycast 返回射线命中的最近节点及其命中参数 t。
//
// 边界行为（均为确定行为）：
//   - 命中距离取世界包围盒的进入参数 tNear；射线起点在盒内时 tNear=0。
//   - 射线与包围盒相切（恰好擦过边/面）视为命中。
//   - 节点尺度退化（某个轴缩放为 0，包围盒退化为面/线/点）时，
//     仍按退化后的世界包围盒做 slab 相交，行为与常规盒一致。
//   - 多个节点命中距离相同时，按场景确定遍历顺序取先者，保证可复现。
//   - 方向向量无需归一化；返回的 t 与 Dir 的尺度一致。
//
// 未命中任何节点时 ok=false。
func (s *Scene) Raycast(r Ray) (id NodeID, t float64, ok bool) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	best := inf()
	var bestID NodeID
	hit := false
	s.walkLocked(func(n *node) {
		if tn, _, ok := HitAABB(r, n.worldBox); ok && tn < best {
			best, bestID, hit = tn, n.id, true
		}
	})
	return bestID, best, hit
}
