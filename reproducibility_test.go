package scenegraph

import (
	"math/rand"
	"testing"
)

// runSeededScenario 用固定种子生成一串确定的操作与查询，
// 返回遍历顺序与各类查询结果，用于验证逐条可复现。
func runSeededScenario(seed int64) (traversal []NodeID, regions [][]NodeID, rays []NodeID) {
	s := NewScene()
	rng := rand.New(rand.NewSource(seed))
	box := AABBFromCenterExtents(Vec3{}, Vec3{1, 1, 1})
	var ids []NodeID
	// 随机构建：随机局部变换、随机挂载（保证无环：只挂到已有节点下）。
	for i := 0; i < 50; i++ {
		id := NodeID("n" + itoa(i))
		t := Transform{
			Position: Vec3{rng.Float64()*20 - 10, rng.Float64()*20 - 10, rng.Float64()*20 - 10},
			Rotation: QuatFromAxisAngle(Vec3{0, 1, 0}, rng.Float64()*3),
			Scale:    Vec3{0.5 + rng.Float64(), 0.5 + rng.Float64(), 0.5 + rng.Float64()},
		}
		if err := s.Add(id, t, box); err != nil {
			panic(err)
		}
		if len(ids) > 0 && rng.Intn(2) == 0 {
			if err := s.Attach(id, ids[rng.Intn(len(ids))]); err != nil {
				panic(err)
			}
		}
		ids = append(ids, id)
	}
	// 随机更新若干节点局部变换。
	for i := 0; i < 20; i++ {
		id := ids[rng.Intn(len(ids))]
		_ = s.SetLocal(id, Transform{
			Position: Vec3{rng.Float64() * 4, 0, 0},
			Rotation: QuatIdentity(),
			Scale:    Vec3{1, 1, 1},
		})
	}
	// 记录确定遍历顺序下的全量查询结果。
	traversal = s.QueryRegion(AABBFromCenterExtents(Vec3{}, Vec3{1000, 1000, 1000}))
	for i := 0; i < 10; i++ {
		c := Vec3{rng.Float64()*20 - 10, rng.Float64()*20 - 10, rng.Float64()*20 - 10}
		regions = append(regions, s.QueryRegion(AABBFromCenterExtents(c, Vec3{3, 3, 3})))
	}
	for i := 0; i < 10; i++ {
		o := Vec3{rng.Float64()*20 - 10, rng.Float64()*20 - 10, rng.Float64()*20 - 10}
		id, _, ok := s.Raycast(Ray{Origin: o, Dir: Vec3{0, 0, -1}})
		if !ok {
			id = "<miss>"
		}
		rays = append(rays, id)
	}
	return traversal, regions, rays
}

func TestReproducibilityWithFixedSeed(t *testing.T) {
	tr1, rg1, ry1 := runSeededScenario(42)
	tr2, rg2, ry2 := runSeededScenario(42)
	if !sameIDs(tr1, tr2) {
		t.Fatalf("traversal order not reproducible")
	}
	for i := range rg1 {
		if !sameIDs(rg1[i], rg2[i]) {
			t.Fatalf("region query %d not reproducible: %v vs %v", i, rg1[i], rg2[i])
		}
	}
	for i := range ry1 {
		if ry1[i] != ry2[i] {
			t.Fatalf("raycast %d not reproducible: %v vs %v", i, ry1[i], ry2[i])
		}
	}
}
