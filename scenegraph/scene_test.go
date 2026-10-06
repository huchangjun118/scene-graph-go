package scenegraph

import (
	"context"
	"errors"
	"testing"
)

// 构建固定测试场景：
//
//	root
//	├── a  T(1,0,0)
//	│   └── b  T(0,2,0)
//	└── c  T(0,0,3)
func buildScene(t *testing.T) *Scene {
	t.Helper()
	s := New()
	tx := s.Begin()
	tx.Add("a", RootID, Translation(1, 0, 0), AABB{Vec3{-1, -1, -1}, Vec3{1, 1, 1}})
	tx.Add("b", "a", Translation(0, 2, 0), AABB{Vec3{-1, -1, -1}, Vec3{1, 1, 1}})
	tx.Add("c", RootID, Translation(0, 0, 3), AABB{Vec3{-0.5, -0.5, -0.5}, Vec3{0.5, 0.5, 0.5}})
	if err := tx.Commit(context.Background()); err != nil {
		t.Fatalf("build scene: %v", err)
	}
	return s
}

func mustPoint(t *testing.T, m Mat4, p Vec3, want Vec3) {
	t.Helper()
	got := m.TransformPoint(p)
	if got != want {
		t.Fatalf("TransformPoint(%v) = %v, want %v", p, got, want)
	}
}

func TestTransformPropagation(t *testing.T) {
	s := buildScene(t)
	mustPoint(t, s.Get("a").World(), Vec3{}, Vec3{1, 0, 0})
	mustPoint(t, s.Get("b").World(), Vec3{}, Vec3{1, 2, 0})
	mustPoint(t, s.Get("c").World(), Vec3{}, Vec3{0, 0, 3})

	// 修改 a 的局部变换：b 必须联动，c 不得变化。
	cBefore := s.Get("c").World()
	tx := s.Begin()
	tx.SetLocal("a", Translation(5, 0, 0))
	if err := tx.Commit(context.Background()); err != nil {
		t.Fatal(err)
	}
	mustPoint(t, s.Get("a").World(), Vec3{}, Vec3{5, 0, 0})
	mustPoint(t, s.Get("b").World(), Vec3{}, Vec3{5, 2, 0})
	if got := s.Get("c").World(); got != cBefore {
		t.Fatalf("unrelated node c world changed: %v -> %v", cBefore, got)
	}
}

func TestIncrementalUpdateOnlyAffectsSubtree(t *testing.T) {
	s := buildScene(t)
	aBefore := s.Get("a").World()
	cBefore := s.Get("c").World()
	rootBefore := s.Root().World()

	// 只改叶子 b：a、c、root 的世界变换必须逐元素不变。
	tx := s.Begin()
	tx.SetLocal("b", Translation(0, 9, 0))
	if err := tx.Commit(context.Background()); err != nil {
		t.Fatal(err)
	}
	mustPoint(t, s.Get("b").World(), Vec3{}, Vec3{1, 9, 0})
	if s.Get("a").World() != aBefore {
		t.Fatal("a world changed")
	}
	if s.Get("c").World() != cBefore {
		t.Fatal("c world changed")
	}
	if s.Root().World() != rootBefore {
		t.Fatal("root world changed")
	}
}

func TestAttachReparentsAndPropagates(t *testing.T) {
	s := buildScene(t)
	tx := s.Begin()
	tx.Attach("c", "b")
	if err := tx.Commit(context.Background()); err != nil {
		t.Fatal(err)
	}
	if got := s.Get("c").Parent().ID(); got != "b" {
		t.Fatalf("c parent = %q, want b", got)
	}
	// c 的世界原点 = a(1,0,0) * b(0,2,0) * c(0,0,3)
	mustPoint(t, s.Get("c").World(), Vec3{}, Vec3{1, 2, 3})
	// 移动 a 时 c 作为孙节点也必须联动。
	tx = s.Begin()
	tx.SetLocal("a", Translation(10, 0, 0))
	if err := tx.Commit(context.Background()); err != nil {
		t.Fatal(err)
	}
	mustPoint(t, s.Get("c").World(), Vec3{}, Vec3{10, 2, 3})
}

func TestAttachCycleRejectedAndStructureIntact(t *testing.T) {
	s := buildScene(t)
	revBefore := s.Revision()

	tx := s.Begin()
	tx.Attach("a", "b") // b 是 a 的后代 → 环
	err := tx.Commit(context.Background())
	if !errors.Is(err, ErrCycle) {
		t.Fatalf("err = %v, want ErrCycle", err)
	}
	// 结构保持不变。
	if got := s.Get("a").Parent().ID(); got != RootID {
		t.Fatalf("a parent = %q, want root", got)
	}
	if got := s.Get("b").Parent().ID(); got != "a" {
		t.Fatalf("b parent = %q, want a", got)
	}
	if s.Revision() != revBefore {
		t.Fatal("revision changed after rejected commit")
	}

	// 自己挂自己也是环。
	tx = s.Begin()
	tx.Attach("a", "a")
	if err := tx.Commit(context.Background()); !errors.Is(err, ErrCycle) {
		t.Fatalf("self attach err = %v, want ErrCycle", err)
	}
}

func TestCommitRollbackOnError(t *testing.T) {
	s := buildScene(t)
	revBefore := s.Revision()
	aBefore := s.Get("a").World()

	tx := s.Begin()
	tx.SetLocal("a", Translation(7, 7, 7)) // 先应用一个合法修改
	tx.Remove("ghost")                     // 再制造一个失败
	err := tx.Commit(context.Background())
	if !errors.Is(err, ErrNodeNotFound) {
		t.Fatalf("err = %v, want ErrNodeNotFound", err)
	}
	// 第一个修改必须被回滚。
	if got := s.Get("a").World(); got != aBefore {
		t.Fatalf("a world = %v, want rolled back %v", got, aBefore)
	}
	if s.Revision() != revBefore {
		t.Fatal("revision changed after rolled back commit")
	}
}

func TestCommitCancelledLeavesSceneIntact(t *testing.T) {
	s := buildScene(t)
	revBefore := s.Revision()
	aBefore := s.Get("a").World()

	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	tx := s.Begin()
	tx.SetLocal("a", Translation(7, 7, 7))
	if err := tx.Commit(ctx); !errors.Is(err, context.Canceled) {
		t.Fatalf("err = %v, want context.Canceled", err)
	}
	if got := s.Get("a").World(); got != aBefore {
		t.Fatal("cancelled commit changed the scene")
	}
	if s.Revision() != revBefore {
		t.Fatal("revision changed after cancelled commit")
	}
}

func TestRemoveSubtree(t *testing.T) {
	s := buildScene(t)
	tx := s.Begin()
	tx.Remove("a") // 连带 b
	if err := tx.Commit(context.Background()); err != nil {
		t.Fatal(err)
	}
	if s.Get("a") != nil || s.Get("b") != nil {
		t.Fatal("subtree not fully removed")
	}
	if s.Get("c") == nil {
		t.Fatal("unrelated node removed")
	}
}

func TestDuplicateAddRejected(t *testing.T) {
	s := buildScene(t)
	tx := s.Begin()
	tx.Add("a", RootID, Identity(), AABB{})
	if err := tx.Commit(context.Background()); !errors.Is(err, ErrNodeExists) {
		t.Fatalf("err = %v, want ErrNodeExists", err)
	}
}

func TestTraversalOrderDeterministic(t *testing.T) {
	s := buildScene(t)
	var got []NodeID
	s.VisitAll(func(n *Node) bool {
		got = append(got, n.ID())
		return true
	})
	want := []NodeID{RootID, "a", "b", "c"}
	if len(got) != len(want) {
		t.Fatalf("got %v, want %v", got, want)
	}
	for i := range want {
		if got[i] != want[i] {
			t.Fatalf("got %v, want %v", got, want)
		}
	}
}
