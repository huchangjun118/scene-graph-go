package scenegraph

import (
	"context"
	"testing"
)

func addBox(t *testing.T, s *Scene, id, parent NodeID, local Mat4, half float64) {
	t.Helper()
	tx := s.Begin()
	tx.Add(id, parent, local, AABB{Vec3{-half, -half, -half}, Vec3{half, half, half}})
	if err := tx.Commit(context.Background()); err != nil {
		t.Fatalf("add %s: %v", id, err)
	}
}

func ids(nodes []*Node) []NodeID {
	out := make([]NodeID, len(nodes))
	for i, n := range nodes {
		out[i] = n.ID()
	}
	return out
}

func eqIDs(t *testing.T, got []*Node, want ...NodeID) {
	t.Helper()
	g := ids(got)
	if len(g) != len(want) {
		t.Fatalf("got %v, want %v", g, want)
	}
	for i := range want {
		if g[i] != want[i] {
			t.Fatalf("got %v, want %v", g, want)
		}
	}
}

func TestQueryRegion(t *testing.T) {
	s := New()
	addBox(t, s, "n1", RootID, Translation(0, 0, 0), 1)
	addBox(t, s, "n2", RootID, Translation(10, 0, 0), 1)
	addBox(t, s, "n3", "n2", Translation(0, 0, 0), 1) // 世界位置同 n2

	got := s.QueryRegion(AABB{Vec3{-2, -2, -2}, Vec3{2, 2, 2}})
	eqIDs(t, got, "n1")

	got = s.QueryRegion(AABB{Vec3{8, -2, -2}, Vec3{12, 2, 2}})
	eqIDs(t, got, "n2", "n3")

	// 边界接触算相交。
	got = s.QueryRegion(AABB{Vec3{1, -1, -1}, Vec3{9, 1, 1}})
	eqIDs(t, got, "n1", "n2", "n3")
}

func TestQueryRegionReflectsLatestCommit(t *testing.T) {
	s := New()
	addBox(t, s, "n1", RootID, Translation(0, 0, 0), 1)
	eqIDs(t, s.QueryRegion(AABB{Vec3{8, -1, -1}, Vec3{12, 1, 1}}))

	tx := s.Begin()
	tx.SetLocal("n1", Translation(10, 0, 0))
	if err := tx.Commit(context.Background()); err != nil {
		t.Fatal(err)
	}
	eqIDs(t, s.QueryRegion(AABB{Vec3{8, -1, -1}, Vec3{12, 1, 1}}), "n1")
}

func TestQueryFrustum(t *testing.T) {
	s := New()
	addBox(t, s, "in", RootID, Translation(0, 0, 0), 0.5)
	addBox(t, s, "out", RootID, Translation(5, 0, 0), 0.5)
	addBox(t, s, "edge", RootID, Translation(1.5, 0, 0), 0.5) // 盒 [1,2]，与 [-1,1] 相切于 x=1

	// 单位矩阵作为视图-投影矩阵 → 裁剪空间 [-1,1]^3。
	f := FrustumFromMatrix(Identity())
	got := s.QueryFrustum(f)
	eqIDs(t, got, "in", "edge")
}

func TestRayCastNearest(t *testing.T) {
	s := New()
	addBox(t, s, "near", RootID, Translation(0, 0, 5), 1)
	addBox(t, s, "far", RootID, Translation(0, 0, 10), 1)

	hit, ok := s.RayCast(Ray{Origin: Vec3{0, 0, -5}, Dir: Vec3{0, 0, 1}})
	if !ok {
		t.Fatal("no hit")
	}
	if hit.Node.ID() != "near" {
		t.Fatalf("hit %q, want near", hit.Node.ID())
	}
	if hit.T != 9 { // 起点 z=-5 到 near 盒近面 z=4
		t.Fatalf("t = %v, want 9", hit.T)
	}
}

func TestRayCastMiss(t *testing.T) {
	s := New()
	addBox(t, s, "n", RootID, Translation(0, 0, 5), 1)
	if _, ok := s.RayCast(Ray{Origin: Vec3{10, 0, -5}, Dir: Vec3{0, 0, 1}}); ok {
		t.Fatal("unexpected hit")
	}
	// 背向射线不得命中。
	if _, ok := s.RayCast(Ray{Origin: Vec3{0, 0, -5}, Dir: Vec3{0, 0, -1}}); ok {
		t.Fatal("unexpected hit behind origin")
	}
}

func TestRayCastOriginInside(t *testing.T) {
	s := New()
	addBox(t, s, "n", RootID, Translation(0, 0, 0), 1)
	hit, ok := s.RayCast(Ray{Origin: Vec3{0, 0, 0}, Dir: Vec3{0, 0, 1}})
	if !ok {
		t.Fatal("no hit for origin inside box")
	}
	if hit.T != 0 {
		t.Fatalf("t = %v, want 0 for origin inside", hit.T)
	}
}

func TestRayCastTangent(t *testing.T) {
	s := New()
	addBox(t, s, "n", RootID, Translation(0, 0, 5), 1)
	// 射线贴着盒子的 y=1 面擦过（相切）。
	hit, ok := s.RayCast(Ray{Origin: Vec3{0, 1, -5}, Dir: Vec3{0, 0, 1}})
	if !ok {
		t.Fatal("tangent ray must hit")
	}
	if hit.T != 9 {
		t.Fatalf("t = %v, want 9", hit.T)
	}
}

func TestRayCastDegenerateScale(t *testing.T) {
	s := New()
	// x 方向尺度退化为 0：包围盒被压成 y-z 平面上的薄片。
	addBox(t, s, "flat", RootID, Scaling(0, 1, 1).Mul(Translation(0, 0, 5)), 1)

	// 垂直射向薄片的射线：命中。
	if _, ok := s.RayCast(Ray{Origin: Vec3{0, 0, -5}, Dir: Vec3{0, 0, 1}}); !ok {
		t.Fatal("ray through degenerate slab must hit")
	}
	// 平行于薄片但不在其平面内的射线：不命中。
	if _, ok := s.RayCast(Ray{Origin: Vec3{1, 0, -5}, Dir: Vec3{0, 0, 1}}); ok {
		t.Fatal("ray offset from degenerate slab must miss")
	}
}

func TestRayCastTieBreakDeterministic(t *testing.T) {
	s := New()
	addBox(t, s, "first", RootID, Translation(0, 0, 5), 1)
	addBox(t, s, "second", RootID, Translation(0, 0, 5), 1) // 与 first 完全重合

	hit, ok := s.RayCast(Ray{Origin: Vec3{0, 0, -5}, Dir: Vec3{0, 0, 1}})
	if !ok {
		t.Fatal("no hit")
	}
	if hit.Node.ID() != "first" {
		t.Fatalf("tie must resolve to traversal-first node, got %q", hit.Node.ID())
	}
}

func TestQueryResultsReproducible(t *testing.T) {
	build := func() *Scene {
		s := New()
		addBox(t, s, "a", RootID, Translation(0, 0, 0), 1)
		addBox(t, s, "b", "a", Translation(1, 0, 0), 1)
		addBox(t, s, "c", RootID, Translation(0, 1, 0), 1)
		return s
	}
	region := AABB{Vec3{-2, -2, -2}, Vec3{3, 3, 3}}
	frustum := FrustumFromMatrix(Identity())
	ray := Ray{Origin: Vec3{0, 0, -10}, Dir: Vec3{0, 0, 1}}

	s1, s2 := build(), build()
	r1, r2 := ids(s1.QueryRegion(region)), ids(s2.QueryRegion(region))
	if len(r1) != len(r2) {
		t.Fatalf("region query not reproducible: %v vs %v", r1, r2)
	}
	for i := range r1 {
		if r1[i] != r2[i] {
			t.Fatalf("region query not reproducible: %v vs %v", r1, r2)
		}
	}
	f1, f2 := ids(s1.QueryFrustum(frustum)), ids(s2.QueryFrustum(frustum))
	if len(f1) != len(f2) {
		t.Fatalf("frustum query not reproducible: %v vs %v", f1, f2)
	}
	for i := range f1 {
		if f1[i] != f2[i] {
			t.Fatalf("frustum query not reproducible: %v vs %v", f1, f2)
		}
	}
	h1, _ := s1.RayCast(ray)
	h2, _ := s2.RayCast(ray)
	if h1.Node.ID() != h2.Node.ID() || h1.T != h2.T {
		t.Fatalf("raycast not reproducible: %v vs %v", h1, h2)
	}
}
