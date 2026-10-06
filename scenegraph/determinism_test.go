package scenegraph

import (
	"context"
	"fmt"
	"math/rand"
	"testing"
)

// runScript 用固定种子生成一串确定性操作并返回所有可观察输出。
func runScript(seed int64) []string {
	rng := rand.New(rand.NewSource(seed))
	s := New()
	ctx := context.Background()
	var out []string

	record := func(tag string) {
		s.VisitAll(func(n *Node) bool {
			out = append(out, fmt.Sprintf("%s node=%s world=%v wb=%v", tag, n.ID(), n.World(), n.WorldBounds()))
			return true
		})
		for _, n := range s.QueryRegion(AABB{Vec3{-50, -50, -50}, Vec3{50, 50, 50}}) {
			out = append(out, fmt.Sprintf("%s region=%s", tag, n.ID()))
		}
		for _, n := range s.QueryFrustum(FrustumFromMatrix(Identity())) {
			out = append(out, fmt.Sprintf("%s frustum=%s", tag, n.ID()))
		}
		if h, ok := s.RayCast(Ray{Origin: Vec3{0.5, 0.5, -100}, Dir: Vec3{0, 0, 1}}); ok {
			out = append(out, fmt.Sprintf("%s ray=%s@%v", tag, h.Node.ID(), h.T))
		} else {
			out = append(out, fmt.Sprintf("%s ray=miss", tag))
		}
	}

	ids := []NodeID{}
	for step := 0; step < 60; step++ {
		tx := s.Begin()
		switch rng.Intn(5) {
		case 0, 1: // add
			id := NodeID(fmt.Sprintf("n%03d", len(ids)))
			parent := RootID
			if len(ids) > 0 {
				parent = ids[rng.Intn(len(ids))]
			}
			h := rng.Float64() * 2
			tx.Add(id, parent,
				Translation(rng.Float64()*20-10, rng.Float64()*20-10, rng.Float64()*20-10),
				AABB{Vec3{-h, -h, -h}, Vec3{h, h, h}})
			ids = append(ids, id)
		case 2: // set local
			if len(ids) > 0 {
				tx.SetLocal(ids[rng.Intn(len(ids))],
					Translation(rng.Float64()*4-2, rng.Float64()*4-2, rng.Float64()*4-2))
			}
		case 3: // attach（可能失败，失败也算确定性输出）
			if len(ids) > 1 {
				a := ids[rng.Intn(len(ids))]
				b := ids[rng.Intn(len(ids))]
				tx.Attach(a, b)
			}
		case 4: // remove
			if len(ids) > 0 {
				i := rng.Intn(len(ids))
				tx.Remove(ids[i])
				ids = append(ids[:i], ids[i+1:]...)
			}
		}
		err := tx.Commit(ctx)
		out = append(out, fmt.Sprintf("step=%d err=%v", step, err))
		// 提交失败后 ids 可能与场景不一致，以场景为准重建。
		ids = ids[:0]
		s.VisitAll(func(n *Node) bool {
			if n.ID() != RootID {
				ids = append(ids, n.ID())
			}
			return true
		})
		record(fmt.Sprintf("step=%d", step))
	}
	return out
}

func TestDeterministicReplay(t *testing.T) {
	a := runScript(42)
	b := runScript(42)
	if len(a) != len(b) {
		t.Fatalf("output length %d vs %d", len(a), len(b))
	}
	for i := range a {
		if a[i] != b[i] {
			t.Fatalf("line %d differs:\n%s\n%s", i, a[i], b[i])
		}
	}
	if len(a) == 0 {
		t.Fatal("empty output")
	}
}
