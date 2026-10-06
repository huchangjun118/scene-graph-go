package scenegraph

import (
	"context"
	"fmt"
)

// NodeRecord 是节点的可序列化表示。
type NodeRecord struct {
	ID     NodeID
	Parent NodeID // 根节点的父为 RootID("")
	Local  Mat4
	Bounds AABB
}

// Snapshot 是场景的可序列化快照。
// Nodes 按深度优先、子节点按挂载顺序排列，且父节点一定先于子节点出现，
// 因此对同一场景重复导出结果逐字节一致。
type Snapshot struct {
	Nodes []NodeRecord
}

// Export 导出场景快照；结果顺序确定。
func (s *Scene) Export() Snapshot {
	s.mu.RLock()
	defer s.mu.RUnlock()
	out := Snapshot{}
	var walk func(n *Node)
	walk = func(n *Node) {
		rec := NodeRecord{
			ID:     n.id,
			Local:  n.local,
			Bounds: n.localBounds,
		}
		if n.parent != nil {
			rec.Parent = n.parent.id
		}
		out.Nodes = append(out.Nodes, rec)
		for _, c := range n.children {
			walk(c)
		}
	}
	walk(s.root)
	return out
}

// Import 把快照中的节点合并进场景：
//   - 已存在的 ID（含根节点）原样跳过，不重复创建、不修改既有数据；
//   - 缺失的节点按记录补齐，父节点缺失时返回错误。
//
// 整个导入是原子的：任一记录失败则场景保持原状。
// 返回实际新增的节点数。
func (s *Scene) Import(ctx context.Context, snap Snapshot) (int, error) {
	// 拓扑排序：保证父记录先于子记录加入事务。
	// Export 产物本身已满足该顺序，这里对外部构造的快照做兜底。
	records := make(map[NodeID]NodeRecord, len(snap.Nodes))
	for _, r := range snap.Nodes {
		if r.ID == RootID {
			continue // 根节点永远已存在
		}
		if _, dup := records[r.ID]; dup {
			return 0, fmt.Errorf("scenegraph: duplicate id %q in snapshot", r.ID)
		}
		records[r.ID] = r
	}

	ordered := make([]NodeRecord, 0, len(records))
	done := make(map[NodeID]bool, len(records))
	known := func(id NodeID) bool {
		if id == RootID || done[id] {
			return true
		}
		if s.Get(id) != nil {
			return true
		}
		return false
	}
	for len(ordered) < len(records) {
		progress := false
		for _, r := range snap.Nodes {
			if r.ID == RootID || done[r.ID] {
				continue
			}
			if known(r.Parent) {
				ordered = append(ordered, r)
				done[r.ID] = true
				progress = true
			}
		}
		if !progress {
			return 0, fmt.Errorf("scenegraph: snapshot has unresolvable parent references")
		}
	}

	tx := s.Begin()
	added := 0
	for _, r := range ordered {
		if s.Get(r.ID) != nil {
			continue // 幂等：已存在的节点跳过
		}
		tx.Add(r.ID, r.Parent, r.Local, r.Bounds)
		added++
	}
	if err := tx.Commit(ctx); err != nil {
		return 0, err
	}
	return added, nil
}
