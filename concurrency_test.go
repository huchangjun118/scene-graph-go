package scenegraph

import (
	"math/rand"
	"sync"
	"testing"
)

// TestConcurrentReadWrite 在多读单写并发下验证：
// 查询结果始终自洽（世界变换不撕裂），且不出现数据竞争（配合 -race）。
func TestConcurrentReadWrite(t *testing.T) {
	s := NewScene()
	box := AABBFromCenterExtents(Vec3{}, Vec3{1, 1, 1})
	const N = 32
	for i := 0; i < N; i++ {
		id := NodeID("n" + itoa(i))
		if err := s.Add(id, IdentityTransform(), box); err != nil {
			t.Fatal(err)
		}
		if i > 0 {
			if err := s.Attach(id, NodeID("n"+itoa(i-1))); err != nil {
				t.Fatal(err)
			}
		}
	}

	var readers, writer sync.WaitGroup
	stop := make(chan struct{})

	// 多个读者：持续做区域/视锥/射线查询，并校验快照自洽：
	// 同一节点的世界包围盒必须等于其世界矩阵作用于局部包围盒。
	for r := 0; r < 4; r++ {
		readers.Add(1)
		go func(seed int64) {
			defer readers.Done()
			rng := rand.New(rand.NewSource(seed))
			for {
				select {
				case <-stop:
					return
				default:
				}
				c := Vec3{rng.Float64()*40 - 20, rng.Float64()*40 - 20, rng.Float64()*40 - 20}
				_ = s.QueryRegion(AABBFromCenterExtents(c, Vec3{5, 5, 5}))
				_ = s.QueryFrustum(FrustumFromMatrix(Mat4Identity()))
				_, _, _ = s.Raycast(Ray{Origin: c, Dir: Vec3{0, 0, -1}})
				id := NodeID("n" + itoa(rng.Intn(N)))
				if n, ok := s.Get(id); ok {
					want := n.Bounds.Transform(n.World)
					if want != n.WorldBox {
						t.Errorf("torn world state for %v: box=%v want=%v", id, n.WorldBox, want)
						return
					}
				}
			}
		}(int64(r))
	}

	// 单个写者：持续提交局部变换更新与事务。
	writer.Add(1)
	go func() {
		defer writer.Done()
		rng := rand.New(rand.NewSource(99))
		for i := 0; i < 500; i++ {
			id := NodeID("n" + itoa(rng.Intn(N)))
			x := rng.Float64() * 10
			_ = s.SetLocal(id, Transform{Position: Vec3{x, 0, 0}, Rotation: QuatIdentity(), Scale: Vec3{1, 1, 1}})
			// 一半概率取消事务：场景不应停在中间状态。
			_ = s.Update(func(tx *Scene) error {
				if err := tx.SetLocal(id, Transform{Position: Vec3{-x, 0, 0}, Rotation: QuatIdentity(), Scale: Vec3{1, 1, 1}}); err != nil {
					return err
				}
				if i%2 == 0 {
					return errTestCancel
				}
				return nil
			})
		}
	}()

	writer.Wait() // 写者完成固定工作量后通知读者退出
	close(stop)
	readers.Wait()
}

var errTestCancel = errorString("cancel")

type errorString string

func (e errorString) Error() string { return string(e) }

func itoa(i int) string {
	if i == 0 {
		return "0"
	}
	var b [8]byte
	p := len(b)
	for i > 0 {
		p--
		b[p] = byte('0' + i%10)
		i /= 10
	}
	return string(b[p:])
}
