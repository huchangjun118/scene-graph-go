package scenegraph

import (
	"context"
	"testing"
)

func sampleScene(t *testing.T) *Scene {
	t.Helper()
	s := New()
	tx := s.Begin()
	tx.Add("a", RootID, Translation(1, 2, 3), AABB{Vec3{-1, -1, -1}, Vec3{1, 1, 1}})
	tx.Add("b", "a", RotationAxis(Vec3{0, 0, 1}, 1.5), AABB{Vec3{0, 0, 0}, Vec3{2, 2, 2}})
	tx.Add("c", "b", Scaling(2, 2, 2), AABB{Vec3{-1, -1, -1}, Vec3{1, 1, 1}})
	tx.Add("d", RootID, Translation(-5, 0, 0), AABB{Vec3{0, 0, 0}, Vec3{1, 1, 1}})
	if err := tx.Commit(context.Background()); err != nil {
		t.Fatal(err)
	}
	return s
}

func TestExportImportRoundTrip(t *testing.T) {
	src := sampleScene(t)
	snap := src.Export()

	dst := New()
	added, err := dst.Import(context.Background(), snap)
	if err != nil {
		t.Fatal(err)
	}
	if added != 4 {
		t.Fatalf("added = %d, want 4", added)
	}

	// 层级结构、节点标识、世界变换、包围盒必须一致。
	var srcOrder, dstOrder []NodeID
	src.VisitAll(func(n *Node) bool { srcOrder = append(srcOrder, n.ID()); return true })
	dst.VisitAll(func(n *Node) bool { dstOrder = append(dstOrder, n.ID()); return true })
	if len(srcOrder) != len(dstOrder) {
		t.Fatalf("order %v vs %v", srcOrder, dstOrder)
	}
	for i := range srcOrder {
		if srcOrder[i] != dstOrder[i] {
			t.Fatalf("order %v vs %v", srcOrder, dstOrder)
		}
	}
	for _, id := range srcOrder {
		a, b := src.Get(id), dst.Get(id)
		if a.World() != b.World() {
			t.Fatalf("node %q world mismatch", id)
		}
		if a.WorldBounds() != b.WorldBounds() {
			t.Fatalf("node %q world bounds mismatch", id)
		}
		if (a.Parent() == nil) != (b.Parent() == nil) {
			t.Fatalf("node %q parent mismatch", id)
		}
		if a.Parent() != nil && a.Parent().ID() != b.Parent().ID() {
			t.Fatalf("node %q parent mismatch", id)
		}
	}

	// 查询结果一致。
	region := AABB{Vec3{-10, -10, -10}, Vec3{10, 10, 10}}
	qs, qd := ids(src.QueryRegion(region)), ids(dst.QueryRegion(region))
	if len(qs) != len(qd) {
		t.Fatalf("region query mismatch: %v vs %v", qs, qd)
	}
	for i := range qs {
		if qs[i] != qd[i] {
			t.Fatalf("region query mismatch: %v vs %v", qs, qd)
		}
	}
	hs, oks := src.RayCast(Ray{Origin: Vec3{1, 2, -100}, Dir: Vec3{0, 0, 1}})
	hd, okd := dst.RayCast(Ray{Origin: Vec3{1, 2, -100}, Dir: Vec3{0, 0, 1}})
	if oks != okd || (oks && (hs.Node.ID() != hd.Node.ID() || hs.T != hd.T)) {
		t.Fatalf("raycast mismatch: %v vs %v", hs, hd)
	}

	// 重复导出逐字节一致。
	snap2 := dst.Export()
	if len(snap.Nodes) != len(snap2.Nodes) {
		t.Fatal("re-export length mismatch")
	}
	for i := range snap.Nodes {
		if snap.Nodes[i] != snap2.Nodes[i] {
			t.Fatalf("re-export record %d mismatch", i)
		}
	}
}

func TestImportIdempotent(t *testing.T) {
	src := sampleScene(t)
	snap := src.Export()

	dst := New()
	if _, err := dst.Import(context.Background(), snap); err != nil {
		t.Fatal(err)
	}
	revBefore := dst.Revision()

	// 重复导入同一份数据：不得产生重复节点，不得修改既有数据。
	added, err := dst.Import(context.Background(), snap)
	if err != nil {
		t.Fatal(err)
	}
	if added != 0 {
		t.Fatalf("re-import added = %d, want 0", added)
	}
	count := 0
	dst.VisitAll(func(n *Node) bool { count++; return true })
	if count != 5 { // root + 4
		t.Fatalf("node count = %d, want 5", count)
	}
	if dst.Revision() != revBefore {
		t.Fatal("idempotent re-import bumped revision")
	}
}

func TestImportMergesIntoExistingScene(t *testing.T) {
	dst := New()
	tx := dst.Begin()
	tx.Add("existing", RootID, Translation(9, 9, 9), AABB{Vec3{0, 0, 0}, Vec3{1, 1, 1}})
	if err := tx.Commit(context.Background()); err != nil {
		t.Fatal(err)
	}

	snap := sampleScene(t).Export()
	added, err := dst.Import(context.Background(), snap)
	if err != nil {
		t.Fatal(err)
	}
	if added != 4 {
		t.Fatalf("added = %d, want 4", added)
	}
	// 既有节点不受影响。
	mustPoint(t, dst.Get("existing").World(), Vec3{}, Vec3{9, 9, 9})
	if dst.Get("a") == nil {
		t.Fatal("imported node missing")
	}
}

func TestImportBadParentFails(t *testing.T) {
	dst := New()
	snap := Snapshot{Nodes: []NodeRecord{
		{ID: "orphan", Parent: "no-such-parent", Local: Identity()},
	}}
	if _, err := dst.Import(context.Background(), snap); err == nil {
		t.Fatal("want error for unresolvable parent")
	}
	// 场景保持原状。
	count := 0
	dst.VisitAll(func(n *Node) bool { count++; return true })
	if count != 1 {
		t.Fatalf("node count = %d, want 1 (root only)", count)
	}
}
