package scenegraph

import (
	"math"
	"testing"
)

// 构造一个带包围盒的三节点场景：
//
//	near:  局部盒中心 (0,0,0) 半尺寸 1，位于原点
//	far:   平移到 (10,0,0)，局部盒同上
//	child: far 的子节点，局部平移 (0,5,0)，局部盒同上 -> 世界中心 (10,5,0)
func buildQueryScene(t *testing.T) *Scene {
	t.Helper()
	s := NewScene()
	box := AABBFromCenterExtents(Vec3{}, Vec3{1, 1, 1})
	must := func(err error) {
		t.Helper()
		if err != nil {
			t.Fatal(err)
		}
	}
	must(s.Add("near", IdentityTransform(), box))
	must(s.Add("far", Transform{Position: Vec3{10, 0, 0}, Rotation: QuatIdentity(), Scale: Vec3{1, 1, 1}}, box))
	must(s.Add("child", Transform{Position: Vec3{0, 5, 0}, Rotation: QuatIdentity(), Scale: Vec3{1, 1, 1}}, box))
	must(s.Attach("child", "far"))
	return s
}

func sameIDs(a, b []NodeID) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

func TestQueryRegion(t *testing.T) {
	s := buildQueryScene(t)

	// 只覆盖原点附近的区域。
	got := s.QueryRegion(AABBFromCenterExtents(Vec3{}, Vec3{0.5, 0.5, 0.5}))
	if !sameIDs(got, []NodeID{"near"}) {
		t.Fatalf("region near origin = %v", got)
	}
	// 覆盖 far 与 child 的区域。
	got = s.QueryRegion(AABBFromCenterExtents(Vec3{10, 2.5, 0}, Vec3{1.5, 3.5, 1.5}))
	if !sameIDs(got, []NodeID{"far", "child"}) {
		t.Fatalf("region around far/child = %v", got)
	}
	// 与包围盒边缘接触（闭区间）也算相交。
	got = s.QueryRegion(AABB{Min: Vec3{1, 1, 1}, Max: Vec3{2, 2, 2}})
	if !sameIDs(got, []NodeID{"near"}) {
		t.Fatalf("touching region = %v", got)
	}
	// 与手工逐节点推导对照：区域查询结果必须等于逐节点 Intersects 的过滤结果。
	region := AABBFromCenterExtents(Vec3{5, 2, 0}, Vec3{6, 4, 2})
	var want []NodeID
	for _, id := range []NodeID{"near", "far", "child"} {
		n, _ := s.Get(id)
		if n.WorldBox.Intersects(region) {
			want = append(want, id)
		}
	}
	if got := s.QueryRegion(region); !sameIDs(got, want) {
		t.Fatalf("region query = %v, manual = %v", got, want)
	}

	// 区域查询必须反映最近一次提交的变换。
	if err := s.SetLocal("far", Transform{Position: Vec3{-10, 0, 0}, Rotation: QuatIdentity(), Scale: Vec3{1, 1, 1}}); err != nil {
		t.Fatal(err)
	}
	got = s.QueryRegion(AABBFromCenterExtents(Vec3{-10, 0, 0}, Vec3{0.5, 0.5, 0.5}))
	if !sameIDs(got, []NodeID{"far"}) {
		t.Fatalf("after committed move, region = %v", got)
	}
}

func TestQueryFrustum(t *testing.T) {
	s := buildQueryScene(t)

	// 构造一个简单的正交视锥：x∈[-2,2], y∈[-2,2], z∈[-1,10]，
	// 直接以 6 个平面表示（内侧为盒形区域）。
	fr := Frustum{
		{N: Vec3{1, 0, 0}, D: 2},   // x >= -2
		{N: Vec3{-1, 0, 0}, D: 2},  // x <= 2
		{N: Vec3{0, 1, 0}, D: 2},   // y >= -2
		{N: Vec3{0, -1, 0}, D: 2},  // y <= 2
		{N: Vec3{0, 0, 1}, D: 1},   // z >= -1
		{N: Vec3{0, 0, -1}, D: 10}, // z <= 10
	}
	got := s.QueryFrustum(fr)
	if !sameIDs(got, []NodeID{"near"}) {
		t.Fatalf("frustum = %v, want [near]", got)
	}

	// 与手工逐节点推导对照。
	var want []NodeID
	for _, id := range []NodeID{"near", "far", "child"} {
		n, _ := s.Get(id)
		if fr.IntersectsAABB(n.WorldBox) {
			want = append(want, id)
		}
	}
	if !sameIDs(got, want) {
		t.Fatalf("frustum query = %v, manual = %v", got, want)
	}

	// 由视图-投影矩阵提取的视锥：单位矩阵对应 NDC 立方体 [-1,1]^3。
	frNDC := FrustumFromMatrix(Mat4Identity())
	got = s.QueryFrustum(frNDC)
	if !sameIDs(got, []NodeID{"near"}) {
		t.Fatalf("NDC frustum = %v, want [near]", got)
	}
}

func TestRaycast(t *testing.T) {
	s := buildQueryScene(t)

	// 从 -x 方向向 +x 发射：先命中 near（t=4），最近命中为 near。
	id, tt, ok := s.Raycast(Ray{Origin: Vec3{-5, 0, 0}, Dir: Vec3{1, 0, 0}})
	if !ok || id != "near" || !almostEq(tt, 4) {
		t.Fatalf("raycast = %v t=%v ok=%v, want near t=4", id, tt, ok)
	}

	// 从 near 与 far 之间向 +x：near 在射线后方，最近命中为 far（t=4）。
	id, _, ok = s.Raycast(Ray{Origin: Vec3{5, 0, 0}, Dir: Vec3{1, 0, 0}})
	if !ok || id != "far" {
		t.Fatalf("raycast = %v ok=%v, want far", id, ok)
	}

	// 未命中。
	if _, _, ok = s.Raycast(Ray{Origin: Vec3{-5, 10, 0}, Dir: Vec3{1, 0, 0}}); ok {
		t.Fatalf("expected miss")
	}

	// 边界：射线起点在包围盒内，t=0 且命中该盒。
	id, tt, ok = s.Raycast(Ray{Origin: Vec3{0, 0, 0}, Dir: Vec3{0, 0, 1}})
	if !ok || id != "near" || !almostEq(tt, 0) {
		t.Fatalf("origin-inside raycast = %v t=%v ok=%v, want near t=0", id, tt, ok)
	}

	// 边界：射线与包围盒相切（沿 y=1 的切线），视为命中。
	id, _, ok = s.Raycast(Ray{Origin: Vec3{-5, 1, 0}, Dir: Vec3{1, 0, 0}})
	if !ok || id != "near" {
		t.Fatalf("tangent raycast = %v ok=%v, want near hit", id, ok)
	}

	// 边界：方向未归一化，t 与方向尺度一致（Dir 放大 2 倍，t 减半）。
	_, tt, ok = s.Raycast(Ray{Origin: Vec3{-5, 0, 0}, Dir: Vec3{2, 0, 0}})
	if !ok || !almostEq(tt, 2) {
		t.Fatalf("scaled dir t = %v, want 2", tt)
	}
}

func TestRaycastDegenerateScale(t *testing.T) {
	s := NewScene()
	box := AABBFromCenterExtents(Vec3{}, Vec3{1, 1, 1})
	// y 轴缩放为 0：世界包围盒退化为 xz 平面上的薄片。
	if err := s.Add("flat", Transform{Position: Vec3{}, Rotation: QuatIdentity(), Scale: Vec3{1, 0, 1}}, box); err != nil {
		t.Fatal(err)
	}
	n, _ := s.Get("flat")
	if !almostEq(n.WorldBox.Min.Y, 0) || !almostEq(n.WorldBox.Max.Y, 0) {
		t.Fatalf("degenerate world box = %+v", n.WorldBox)
	}
	// 沿 y 轴穿过退化面：确定命中，t=5。
	id, tt, ok := s.Raycast(Ray{Origin: Vec3{0, 5, 0}, Dir: Vec3{0, -1, 0}})
	if !ok || id != "flat" || !almostEq(tt, 5) {
		t.Fatalf("degenerate raycast = %v t=%v ok=%v, want flat t=5", id, tt, ok)
	}
	// 平行于退化面且不在面内：确定不命中。
	if _, _, ok := s.Raycast(Ray{Origin: Vec3{0, 1, 0}, Dir: Vec3{1, 0, 0}}); ok {
		t.Fatalf("parallel ray outside degenerate slab should miss")
	}
}

func TestRaycastTieBreakDeterministic(t *testing.T) {
	// 两个节点世界包围盒完全重合，命中距离相同：
	// 必须按遍历顺序取先者，结果确定。
	s := NewScene()
	box := AABBFromCenterExtents(Vec3{}, Vec3{1, 1, 1})
	_ = s.Add("first", IdentityTransform(), box)
	_ = s.Add("second", IdentityTransform(), box)
	id, _, ok := s.Raycast(Ray{Origin: Vec3{-5, 0, 0}, Dir: Vec3{1, 0, 0}})
	if !ok || id != "first" {
		t.Fatalf("tie-break = %v, want first", id)
	}
}

func TestFrustumFromPerspectiveMatrix(t *testing.T) {
	// 构造对称透视投影矩阵（列主序，clip = P * v），
	// 验证提取出的视锥能正确包含/排除点。
	fov, aspect, near, far := math.Pi/2, 1.0, 1.0, 100.0
	f := 1 / math.Tan(fov/2)
	p := Mat4{
		f / aspect, 0, 0, 0,
		0, f, 0, 0,
		0, 0, (far + near) / (near - far), -1,
		0, 0, 2 * far * near / (near - far), 0,
	}
	fr := FrustumFromMatrix(p)
	// 相机位于原点朝 -z 看：z=-5 处的单位盒应在视锥内。
	inside := AABBFromCenterExtents(Vec3{0, 0, -5}, Vec3{0.5, 0.5, 0.5})
	if !fr.IntersectsAABB(inside) {
		t.Fatalf("box at z=-5 should be inside frustum")
	}
	// 相机后方的盒应在视锥外。
	behind := AABBFromCenterExtents(Vec3{0, 0, 5}, Vec3{0.5, 0.5, 0.5})
	if fr.IntersectsAABB(behind) {
		t.Fatalf("box behind camera should be outside frustum")
	}
	// 远超远平面的盒应在视锥外。
	tooFar := AABBFromCenterExtents(Vec3{0, 0, -200}, Vec3{0.5, 0.5, 0.5})
	if fr.IntersectsAABB(tooFar) {
		t.Fatalf("box beyond far plane should be outside frustum")
	}
}
