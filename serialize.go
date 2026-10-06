package scenegraph

import (
	"encoding/json"
	"fmt"
)

// exportVersion 为当前导出格式版本。
const exportVersion = 1

// sceneExport 为场景的可序列化表示。
// 节点按场景确定遍历顺序排列，保证固定输入下导出字节流可复现。
type sceneExport struct {
	Version int          `json:"version"`
	Nodes   []nodeExport `json:"nodes"`
}

type nodeExport struct {
	ID     NodeID    `json:"id"`
	Parent NodeID    `json:"parent,omitempty"` // 空表示根节点
	Local  Transform `json:"local"`
	Bounds AABB      `json:"bounds"`
}

// Export 将场景导出为 JSON 字节流，包含全部节点的标识、
// 层级关系、局部变换与包围盒。恢复后层级结构、节点标识
// 与查询结果与导出前一致。
func (s *Scene) Export() ([]byte, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	exp := sceneExport{Version: exportVersion}
	s.walkLocked(func(n *node) {
		var pid NodeID
		if n.parent != nil {
			pid = n.parent.id
		}
		exp.Nodes = append(exp.Nodes, nodeExport{
			ID:     n.id,
			Parent: pid,
			Local:  n.local,
			Bounds: n.bounds,
		})
	})
	return json.Marshal(exp)
}

// Import 从 JSON 字节流恢复场景。
//
// 语义：
//   - 整个导入是原子的：数据非法（格式错误、父节点缺失、成环）时
//     返回错误且场景保持原状。
//   - 导入是幂等的：对同一份数据重复导入不会产生重复节点；
//     已存在的节点按导入数据更新其局部变换、包围盒与挂载关系。
//   - 场景中已存在但不在导入数据中的节点保持不变（合并语义）。
func (s *Scene) Import(data []byte) error {
	var exp sceneExport
	if err := json.Unmarshal(data, &exp); err != nil {
		return fmt.Errorf("scenegraph: import: %w", err)
	}
	if exp.Version != exportVersion {
		return fmt.Errorf("scenegraph: import: unsupported version %d", exp.Version)
	}
	// 在事务副本上应用，任何一步失败都不会污染原场景。
	return s.Update(func(tx *Scene) error {
		// 第一遍：创建或更新节点（先全部就位，再处理挂载）。
		for _, ne := range exp.Nodes {
			if _, ok := tx.nodes[ne.ID]; ok {
				if err := tx.SetLocal(ne.ID, ne.Local); err != nil {
					return err
				}
				if err := tx.SetBounds(ne.ID, ne.Bounds); err != nil {
					return err
				}
			} else if err := tx.Add(ne.ID, ne.Local, ne.Bounds); err != nil {
				return err
			}
		}
		// 第二遍：恢复挂载关系。先将全部导入节点摘为根，
		// 消除现有层级与目标层级之间的祖先干扰（例如导入数据
		// 将现有父子关系颠倒时，直接挂载会误判成环）；
		// 再按导出顺序挂载。导入数据自身成环时 Attach 报错并整体回滚。
		for _, ne := range exp.Nodes {
			if err := tx.Detach(ne.ID); err != nil {
				return err
			}
		}
		for _, ne := range exp.Nodes {
			if ne.Parent == "" {
				continue
			}
			if err := tx.Attach(ne.ID, ne.Parent); err != nil {
				return err
			}
		}
		return nil
	})
}
