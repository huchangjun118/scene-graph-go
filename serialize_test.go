package scenegraph

import (
	"math"
	"testing"
)

func buildExportScene(t *testing.T) *Scene {
	t.Helper()
	s := NewScene()
	box := AABBFromCenterExtents(Vec3{}, Vec3{1, 1, 1})
	must := func(err error) {
		t.Helper()
		if err != nil {
			t.Fatal(err)
		}
	}
	must(s.Add("root", Transform{Position: Vec3{1, 2, 3}, Rotation: QuatFromAxisAngle(Vec3{0, 1, 0}, math.Pi/4), Scale: Vec3{2, 2, 2}}, box))
	must(s.Add("a", Transform{Position: Vec3{0, 1, 0}, Rotation: QuatIdentity(), Scale: Vec3{1, 1, 1}}, box))
	must(s.Add("b", Transform{Position: Vec3{1, 0, 0}, Rotation: QuatIdentity(), Scale: Vec3{0.5, 0.5, 0.5}}, box))
	must(s.Add("loose", IdentityTransform(), box))
	must(s.Attach("a", "root"))
	must(s.Attach("b", "a"))
	return s
}

func sceneStateEqual(t *testing.T, a, b *Scene) {
	t.Helper()
	ra, rb := a.Roots(), b.Roots()
	if !sameIDs(ra, rb) {
		t.Fatalf("roots differ: %v vs %v", ra, rb)
	}
	region := AABBFromCenterExtents(Vec3{}, Vec3{100, 100, 100})
	qa, qb := a.QueryRegion(region), b.QueryRegion(region)
	if !sameIDs(qa, qb) {
		t.Fatalf("query ids differ: %v vs %v", qa, qb)
	}
	for _, id := range qa {
		na, _ := a.Get(id)
		nb, _ := b.Get(id)
		if na.Parent != nb.Parent || !sameIDs(na.Children, nb.Children) {
			t.Fatalf("node %v hierarchy differs", id)
		}
		if na.World != nb.World || na.WorldBox != nb.WorldBox || na.Local != nb.Local || na.Bounds != nb.Bounds {
			t.Fatalf("node %v state differs", id)
		}
	}
}

func TestExportImportRoundTrip(t *testing.T) {
	src := buildExportScene(t)
	data, err := src.Export()
	if err != nil {
		t.Fatal(err)
	}
	dst := NewScene()
	if err := dst.Import(data); err != nil {
		t.Fatal(err)
	}
	sceneStateEqual(t, src, dst)

	// 射线查询结果也必须一致。
	ray := Ray{Origin: Vec3{0, 0, -10}, Dir: Vec3{0, 0, 1}}
	idA, tA, okA := src.Raycast(ray)
	idB, tB, okB := dst.Raycast(ray)
	if okA != okB || idA != idB || tA != tB {
		t.Fatalf("raycast differs after restore: %v/%v/%v vs %v/%v/%v", idA, tA, okA, idB, tB, okB)
	}
}

func TestImportIdempotent(t *testing.T) {
	src := buildExportScene(t)
	data, err := src.Export()
	if err != nil {
		t.Fatal(err)
	}
	dst := NewScene()
	if err := dst.Import(data); err != nil {
		t.Fatal(err)
	}
	// 对同一份数据重复导入：不得产生重复节点，状态不变。
	if err := dst.Import(data); err != nil {
		t.Fatal(err)
	}
	if err := dst.Import(data); err != nil {
		t.Fatal(err)
	}
	sceneStateEqual(t, src, dst)
}

func TestImportAtomicOnBadData(t *testing.T) {
	s := buildExportScene(t)
	before, err := s.Export()
	if err != nil {
		t.Fatal(err)
	}
	// 成环的导入数据必须整体失败，场景保持原状。
	bad := []byte(`{"version":1,"nodes":[
		{"id":"x","parent":"y","local":{"Position":{},"Rotation":{"W":1},"Scale":{"X":1,"Y":1,"Z":1}},"bounds":{}},
		{"id":"y","parent":"x","local":{"Position":{},"Rotation":{"W":1},"Scale":{"X":1,"Y":1,"Z":1}},"bounds":{}}
	]}`)
	if err := s.Import(bad); err == nil {
		t.Fatalf("expected cycle import to fail")
	}
	after, err := s.Export()
	if err != nil {
		t.Fatal(err)
	}
	if string(before) != string(after) {
		t.Fatalf("scene changed after failed import")
	}
	// 非法 JSON 与错误版本也必须失败。
	if err := s.Import([]byte("not json")); err == nil {
		t.Fatalf("expected invalid json to fail")
	}
	if err := s.Import([]byte(`{"version":99,"nodes":[]}`)); err == nil {
		t.Fatalf("expected unsupported version to fail")
	}
}

func TestExportDeterministic(t *testing.T) {
	a := buildExportScene(t)
	b := buildExportScene(t)
	da, err := a.Export()
	if err != nil {
		t.Fatal(err)
	}
	db, err := b.Export()
	if err != nil {
		t.Fatal(err)
	}
	if string(da) != string(db) {
		t.Fatalf("export not reproducible:\n%s\n%s", da, db)
	}
}
