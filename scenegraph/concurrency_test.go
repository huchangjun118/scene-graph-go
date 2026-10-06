package scenegraph

import (
	"context"
	"fmt"
	"math/rand"
	"sync"
	"testing"
)

// TestConcurrentReadWrite 在多读单写并发下运行，
// 需配合 go test -race 验证无数据竞争。
func TestConcurrentReadWrite(t *testing.T) {
	s := New()
	const n = 64
	tx := s.Begin()
	for i := 0; i < n; i++ {
		id := NodeID(fmt.Sprintf("n%03d", i))
		tx.Add(id, RootID, Translation(float64(i*4), 0, 0), AABB{Vec3{-1, -1, -1}, Vec3{1, 1, 1}})
	}
	if err := tx.Commit(context.Background()); err != nil {
		t.Fatal(err)
	}

	ctx := context.Background()
	var wg sync.WaitGroup
	stop := make(chan struct{})

	// 多个读协程：区域查询 / 视锥查询 / 射线 / 遍历 / 单节点访问器。
	for g := 0; g < 4; g++ {
		wg.Add(1)
		go func(seed int64) {
			defer wg.Done()
			rng := rand.New(rand.NewSource(seed))
			for {
				select {
				case <-stop:
					return
				default:
				}
				switch rng.Intn(5) {
				case 0:
					x := rng.Float64() * 200
					_ = s.QueryRegion(AABB{Vec3{x, -2, -2}, Vec3{x + 10, 2, 2}})
				case 1:
					_ = s.QueryFrustum(FrustumFromMatrix(Identity()))
				case 2:
					_, _ = s.RayCast(Ray{Origin: Vec3{rng.Float64() * 200, 0, -5}, Dir: Vec3{0, 0, 1}})
				case 3:
					s.VisitAll(func(n *Node) bool { return rng.Intn(10) > 0 })
				default:
					id := NodeID(fmt.Sprintf("n%03d", rng.Intn(n)))
					if node := s.Get(id); node != nil {
						_ = node.World()
						_ = node.WorldBounds()
						_ = node.Children()
						_ = node.Parent()
					}
				}
			}
		}(int64(g))
	}

	// 单写协程：交替做合法更新与必然回滚的更新。
	wg.Add(1)
	go func() {
		defer wg.Done()
		for i := 0; i < 200; i++ {
			id := NodeID(fmt.Sprintf("n%03d", i%n))
			tx := s.Begin()
			tx.SetLocal(id, Translation(float64(i%7), 0, 0))
			if i%3 == 0 {
				tx.Remove("ghost") // 触发整体回滚
			}
			_ = tx.Commit(ctx)
		}
		close(stop)
	}()
	wg.Wait()

	// 写协程结束后场景必须处于自洽状态：
	// 每个节点的世界变换等于沿父链手工推导的结果。
	s.VisitAll(func(node *Node) bool {
		want := node.Local()
		if p := node.Parent(); p != nil {
			want = p.World().Mul(node.Local())
		}
		if got := node.World(); got != want {
			t.Errorf("node %q world torn: got %v want %v", node.ID(), got, want)
			return false
		}
		if got, want := node.WorldBounds(), node.LocalBounds().Transformed(node.World()); got != want {
			t.Errorf("node %q world bounds torn: got %v want %v", node.ID(), got, want)
			return false
		}
		return true
	})
}

// TestConcurrentCommits 验证并发提交串行化且计数正确。
func TestConcurrentCommits(t *testing.T) {
	s := New()
	var wg sync.WaitGroup
	const writers = 8
	const perWriter = 25
	for w := 0; w < writers; w++ {
		wg.Add(1)
		go func(w int) {
			defer wg.Done()
			for i := 0; i < perWriter; i++ {
				tx := s.Begin()
				tx.Add(NodeID(fmt.Sprintf("w%02d-%03d", w, i)), RootID, Identity(), AABB{})
				if err := tx.Commit(context.Background()); err != nil {
					t.Error(err)
				}
			}
		}(w)
	}
	wg.Wait()
	if got, want := s.Revision(), uint64(writers*perWriter); got != want {
		t.Fatalf("revision = %d, want %d", got, want)
	}
	count := 0
	s.VisitAll(func(n *Node) bool { count++; return true })
	if want := writers*perWriter + 1; count != want {
		t.Fatalf("node count = %d, want %d", count, want)
	}
}
