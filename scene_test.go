package scenegraph

import (
	"errors"
	"math"
	"testing"
)

func almostEq(a, b float64) bool { return math.Abs(a-b) < 1e-9 }

func vecAlmostEq(a, b Vec3) bool {
	return almostEq(a.X, b.X) && almostEq(a.Y, b.Y) && almostEq(a.Z, b.Z)
}

// 手工推导世界坐标：从节点沿父链逐层复合 TRS。
func manualWorldPoint(s *Scene, id NodeID, p Vec3) Vec3 {
	n, ok := s.Get(id)
	if !ok {
		return p
	}
	for {
		m := n.Local.Matrix()
		p = m.TransformPoint(p)
		if n.Parent == "" {
			return p
		}
		n, _ = s.Get(n.Parent)
	}
}

func TestTransformPropagation(t *testing.T) {
	s := NewScene()
	must := func(err error) {
		t.Helper()
		if err != nil {
			t.Fatal(err)
		}
	}
	must(s.Add("root", Transform{Position: Vec3{1, 0, 0}, Rotation: QuatIdentity(), Scale: Vec3{1, 1, 1}}, AABB{}))
	must(s.Add("mid", Transform{Position: Vec3{0, 2, 0}, Rotation: QuatFromAxisAngle(Vec3{0, 0, 1}, math.Pi/2), Scale: Vec3{2, 2, 2}}, AABB{}))
	must(s.Add("leaf", Transform{Position: Vec3{1, 0, 0}, Rotation: QuatIdentity(), Scale: Vec3{1, 1, 1}}, AABB{}))
	must(s.Attach("mid", "root"))
	must(s.Attach("leaf", "mid"))

	// 手工推导：leaf 局部点 (1,0,0) 先经 leaf 自身局部变换平移到 (2,0,0)，
	// 再经 mid: R(90°z)*S(2)*(2,0,0)+(0,2,0) = (0,4,0)+(0,2,0) = (0,6,0)，
	// 最后经 root 平移 (1,0,0) 得 (1,6,0)。
	leaf, _ := s.Get("leaf")
	got := leaf.World.TransformPoint(Vec3{1, 0, 0})
	want := Vec3{1, 6, 0}
	if !vecAlmostEq(got, want) {
		t.Fatalf("world point = %v, want %v", got, want)
	}

	// 父节点变换改变后，全部后代同步更新。
	must(s.SetLocal("root", Transform{Position: Vec3{10, 0, 0}, Rotation: QuatIdentity(), Scale: Vec3{1, 1, 1}}))
	leaf, _ = s.Get("leaf")
	got = leaf.World.TransformPoint(Vec3{1, 0, 0})
	want = Vec3{10, 6, 0}
	if !vecAlmostEq(got, want) {
		t.Fatalf("after parent move, world point = %v, want %v", got, want)
	}
	mid, _ := s.Get("mid")
	if !vecAlmostEq(mid.World.TransformPoint(Vec3{0, 0, 0}), Vec3{10, 2, 0}) {
		t.Fatalf("mid world not updated")
	}
}

func TestIncrementalUpdateOnlyAffectsSubtree(t *testing.T) {
	s := NewScene()
	must := func(err error) {
		t.Helper()
		if err != nil {
			t.Fatal(err)
		}
	}
	must(s.Add("a", Transform{Position: Vec3{1, 0, 0}, Rotation: QuatIdentity(), Scale: Vec3{1, 1, 1}}, AABB{}))
	must(s.Add("b", Transform{Position: Vec3{5, 0, 0}, Rotation: QuatIdentity(), Scale: Vec3{1, 1, 1}}, AABB{}))
	must(s.Add("a1", Transform{Position: Vec3{0, 1, 0}, Rotation: QuatIdentity(), Scale: Vec3{1, 1, 1}}, AABB{}))
	must(s.Attach("a1", "a"))

	bBefore, _ := s.Get("b")
	must(s.SetLocal("a", Transform{Position: Vec3{2, 0, 0}, Rotation: QuatIdentity(), Scale: Vec3{1, 1, 1}}))
	bAfter, _ := s.Get("b")
	if bBefore.World != bAfter.World || bBefore.WorldBox != bAfter.WorldBox {
		t.Fatalf("unrelated node world transform changed")
	}
	a1, _ := s.Get("a1")
	if got := a1.World.TransformPoint(Vec3{}); !vecAlmostEq(got, Vec3{2, 1, 0}) {
		t.Fatalf("descendant not updated, got %v", got)
	}
}

func TestAttachReparentAndCycle(t *testing.T) {
	s := NewScene()
	must := func(err error) {
		t.Helper()
		if err != nil {
			t.Fatal(err)
		}
	}
	ids := []NodeID{"a", "b", "c", "d"}
	for _, id := range ids {
		must(s.Add(id, IdentityTransform(), AABB{}))
	}
	must(s.Attach("b", "a"))
	must(s.Attach("c", "b"))

	// 重挂载：c 从 b 移到 d。
	must(s.Attach("c", "d"))
	b, _ := s.Get("b")
	if len(b.Children) != 0 {
		t.Fatalf("b should have no children after reparent, got %v", b.Children)
	}
	d, _ := s.Get("d")
	if len(d.Children) != 1 || d.Children[0] != "c" {
		t.Fatalf("d children = %v", d.Children)
	}

	// 环：把 a 挂到其后代 b 下，必须被拒绝且结构不变。
	rootsBefore := s.Roots()
	err := s.Attach("a", "b")
	if !errors.Is(err, ErrCycle) {
		t.Fatalf("expected ErrCycle, got %v", err)
	}
	if got := s.Roots(); len(got) != len(rootsBefore) {
		t.Fatalf("roots changed after rejected attach: %v", got)
	}
	a, _ := s.Get("a")
	if a.Parent != "" {
		t.Fatalf("a parent changed after rejected attach")
	}
	// 自挂载也必须被拒绝。
	if err := s.Attach("a", "a"); !errors.Is(err, ErrAttachRoot) {
		t.Fatalf("expected ErrAttachRoot, got %v", err)
	}
	// 挂载到不存在节点。
	if err := s.Attach("a", "ghost"); !errors.Is(err, ErrNodeNotFound) {
		t.Fatalf("expected ErrNodeNotFound, got %v", err)
	}
}

func TestUpdateTransactionCancel(t *testing.T) {
	s := NewScene()
	must := func(err error) {
		t.Helper()
		if err != nil {
			t.Fatal(err)
		}
	}
	must(s.Add("a", IdentityTransform(), AABB{}))
	must(s.Add("b", IdentityTransform(), AABB{}))

	// 取消的更新：场景必须保持原状。
	errCancel := errors.New("cancel")
	err := s.Update(func(tx *Scene) error {
		if err := tx.Attach("b", "a"); err != nil {
			return err
		}
		if err := tx.SetLocal("a", Transform{Position: Vec3{9, 9, 9}, Rotation: QuatIdentity(), Scale: Vec3{1, 1, 1}}); err != nil {
			return err
		}
		return errCancel
	})
	if !errors.Is(err, errCancel) {
		t.Fatalf("expected cancel error, got %v", err)
	}
	a, _ := s.Get("a")
	if a.Local.Position != (Vec3{}) || len(a.Children) != 0 {
		t.Fatalf("cancelled update leaked into scene: %+v", a)
	}
	b, _ := s.Get("b")
	if b.Parent != "" {
		t.Fatalf("cancelled attach leaked into scene")
	}

	// 成功的更新：原子提交。
	must(s.Update(func(tx *Scene) error {
		if err := tx.Attach("b", "a"); err != nil {
			return err
		}
		return tx.SetLocal("a", Transform{Position: Vec3{1, 0, 0}, Rotation: QuatIdentity(), Scale: Vec3{1, 1, 1}})
	}))
	b, _ = s.Get("b")
	if b.Parent != "a" {
		t.Fatalf("committed update not visible")
	}
	if got := b.World.TransformPoint(Vec3{}); !vecAlmostEq(got, Vec3{1, 0, 0}) {
		t.Fatalf("committed world = %v", got)
	}
}

func TestDeterministicTraversalOrder(t *testing.T) {
	build := func() *Scene {
		s := NewScene()
		for _, id := range []NodeID{"r1", "r2", "a", "b", "c"} {
			_ = s.Add(id, IdentityTransform(), AABB{})
		}
		_ = s.Attach("a", "r1")
		_ = s.Attach("b", "r1")
		_ = s.Attach("c", "r2")
		return s
	}
	s1, s2 := build(), build()
	q1 := s1.QueryRegion(AABB{Min: Vec3{-1, -1, -1}, Max: Vec3{1, 1, 1}})
	q2 := s2.QueryRegion(AABB{Min: Vec3{-1, -1, -1}, Max: Vec3{1, 1, 1}})
	if len(q1) != 5 {
		t.Fatalf("expected 5 hits, got %v", q1)
	}
	for i := range q1 {
		if q1[i] != q2[i] {
			t.Fatalf("traversal order not reproducible: %v vs %v", q1, q2)
		}
	}
	want := []NodeID{"r1", "a", "b", "r2", "c"}
	for i, id := range want {
		if q1[i] != id {
			t.Fatalf("order = %v, want %v", q1, want)
		}
	}
}
