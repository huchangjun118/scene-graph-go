// Package scenegraph 提供场景图与空间查询内核。
//
// 坐标约定：右手坐标系，列主序 4x4 矩阵，向量按列向量处理，
// 世界变换 = 父世界变换 * 局部变换。
package scenegraph

import "math"

// Vec3 是三维向量/点。
type Vec3 struct {
	X, Y, Z float64
}

// Add 返回 a+b。
func (a Vec3) Add(b Vec3) Vec3 { return Vec3{a.X + b.X, a.Y + b.Y, a.Z + b.Z} }

// Sub 返回 a-b。
func (a Vec3) Sub(b Vec3) Vec3 { return Vec3{a.X - b.X, a.Y - b.Y, a.Z - b.Z} }

// Scale 返回 a*s。
func (a Vec3) Scale(s float64) Vec3 { return Vec3{a.X * s, a.Y * s, a.Z * s} }

// Dot 返回点积。
func (a Vec3) Dot(b Vec3) float64 { return a.X*b.X + a.Y*b.Y + a.Z*b.Z }

// Cross 返回叉积。
func (a Vec3) Cross(b Vec3) Vec3 {
	return Vec3{
		a.Y*b.Z - a.Z*b.Y,
		a.Z*b.X - a.X*b.Z,
		a.X*b.Y - a.Y*b.X,
	}
}

// Length 返回向量长度。
func (a Vec3) Length() float64 { return math.Sqrt(a.Dot(a)) }

// Normalize 返回单位向量；零向量返回零向量。
func (a Vec3) Normalize() Vec3 {
	l := a.Length()
	if l == 0 {
		return Vec3{}
	}
	return a.Scale(1 / l)
}

// Mat4 是 4x4 列主序矩阵（m[0..3] 为第一列）。
type Mat4 [16]float64

// Identity 返回单位矩阵。
func Identity() Mat4 {
	return Mat4{
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	}
}

// Translation 返回平移矩阵。
func Translation(x, y, z float64) Mat4 {
	m := Identity()
	m[12], m[13], m[14] = x, y, z
	return m
}

// Scaling 返回缩放矩阵。
func Scaling(x, y, z float64) Mat4 {
	m := Identity()
	m[0], m[5], m[10] = x, y, z
	return m
}

// RotationAxis 返回绕轴 axis（需为单位向量）旋转 angle 弧度的矩阵。
func RotationAxis(axis Vec3, angle float64) Mat4 {
	c, s := math.Cos(angle), math.Sin(angle)
	t := 1 - c
	x, y, z := axis.X, axis.Y, axis.Z
	return Mat4{
		t*x*x + c, t*x*y + s*z, t*x*z - s*y, 0,
		t*x*y - s*z, t*y*y + c, t*y*z + s*x, 0,
		t*x*z + s*y, t*y*z - s*x, t*z*z + c, 0,
		0, 0, 0, 1,
	}
}

// Mul 返回 m*o（先应用 o，再应用 m）。
func (m Mat4) Mul(o Mat4) Mat4 {
	var r Mat4
	for c := 0; c < 4; c++ {
		for row := 0; row < 4; row++ {
			r[c*4+row] = m[row]*o[c*4] + m[4+row]*o[c*4+1] + m[8+row]*o[c*4+2] + m[12+row]*o[c*4+3]
		}
	}
	return r
}

// TransformPoint 对点做齐次变换（含透视除法）。
func (m Mat4) TransformPoint(p Vec3) Vec3 {
	x := m[0]*p.X + m[4]*p.Y + m[8]*p.Z + m[12]
	y := m[1]*p.X + m[5]*p.Y + m[9]*p.Z + m[13]
	z := m[2]*p.X + m[6]*p.Y + m[10]*p.Z + m[14]
	w := m[3]*p.X + m[7]*p.Y + m[11]*p.Z + m[15]
	if w != 0 && w != 1 {
		inv := 1 / w
		return Vec3{x * inv, y * inv, z * inv}
	}
	return Vec3{x, y, z}
}

// TransformDir 对方向向量做变换（忽略平移，不做透视除法）。
func (m Mat4) TransformDir(d Vec3) Vec3 {
	return Vec3{
		m[0]*d.X + m[4]*d.Y + m[8]*d.Z,
		m[1]*d.X + m[5]*d.Y + m[9]*d.Z,
		m[2]*d.X + m[6]*d.Y + m[10]*d.Z,
	}
}

// Inverse 返回矩阵的逆；奇异矩阵返回 ok=false。
func (m Mat4) Inverse() (Mat4, bool) {
	// 基于余子式的通用 4x4 求逆（列主序）。
	var inv Mat4
	inv[0] = m[5]*m[10]*m[15] - m[5]*m[11]*m[14] - m[9]*m[6]*m[15] + m[9]*m[7]*m[14] + m[13]*m[6]*m[11] - m[13]*m[7]*m[10]
	inv[4] = -m[4]*m[10]*m[15] + m[4]*m[11]*m[14] + m[8]*m[6]*m[15] - m[8]*m[7]*m[14] - m[12]*m[6]*m[11] + m[12]*m[7]*m[10]
	inv[8] = m[4]*m[9]*m[15] - m[4]*m[11]*m[13] - m[8]*m[5]*m[15] + m[8]*m[7]*m[13] + m[12]*m[5]*m[11] - m[12]*m[7]*m[9]
	inv[12] = -m[4]*m[9]*m[14] + m[4]*m[10]*m[13] + m[8]*m[5]*m[14] - m[8]*m[6]*m[13] - m[12]*m[5]*m[10] + m[12]*m[6]*m[9]
	inv[1] = -m[1]*m[10]*m[15] + m[1]*m[11]*m[14] + m[9]*m[2]*m[15] - m[9]*m[3]*m[14] - m[13]*m[2]*m[11] + m[13]*m[3]*m[10]
	inv[5] = m[0]*m[10]*m[15] - m[0]*m[11]*m[14] - m[8]*m[2]*m[15] + m[8]*m[3]*m[14] + m[12]*m[2]*m[11] - m[12]*m[3]*m[10]
	inv[9] = -m[0]*m[9]*m[15] + m[0]*m[11]*m[13] + m[8]*m[1]*m[15] - m[8]*m[3]*m[13] - m[12]*m[1]*m[11] + m[12]*m[3]*m[9]
	inv[13] = m[0]*m[9]*m[14] - m[0]*m[10]*m[13] - m[8]*m[1]*m[14] + m[8]*m[2]*m[13] + m[12]*m[1]*m[10] - m[12]*m[2]*m[9]
	inv[2] = m[1]*m[6]*m[15] - m[1]*m[7]*m[14] - m[5]*m[2]*m[15] + m[5]*m[3]*m[14] + m[13]*m[2]*m[7] - m[13]*m[3]*m[6]
	inv[6] = -m[0]*m[6]*m[15] + m[0]*m[7]*m[14] + m[4]*m[2]*m[15] - m[4]*m[3]*m[14] - m[12]*m[2]*m[7] + m[12]*m[3]*m[6]
	inv[10] = m[0]*m[5]*m[15] - m[0]*m[7]*m[13] - m[4]*m[1]*m[15] + m[4]*m[3]*m[13] + m[12]*m[1]*m[7] - m[12]*m[3]*m[5]
	inv[14] = -m[0]*m[5]*m[14] + m[0]*m[6]*m[13] + m[4]*m[1]*m[14] - m[4]*m[2]*m[13] - m[12]*m[1]*m[6] + m[12]*m[2]*m[5]
	inv[3] = -m[1]*m[6]*m[11] + m[1]*m[7]*m[10] + m[5]*m[2]*m[11] - m[5]*m[3]*m[10] - m[9]*m[2]*m[7] + m[9]*m[3]*m[6]
	inv[7] = m[0]*m[6]*m[11] - m[0]*m[7]*m[10] - m[4]*m[2]*m[11] + m[4]*m[3]*m[10] + m[8]*m[2]*m[7] - m[8]*m[3]*m[6]
	inv[11] = -m[0]*m[5]*m[11] + m[0]*m[7]*m[9] + m[4]*m[1]*m[11] - m[4]*m[3]*m[9] - m[8]*m[1]*m[7] + m[8]*m[3]*m[5]
	inv[15] = m[0]*m[5]*m[10] - m[0]*m[6]*m[9] - m[4]*m[1]*m[10] + m[4]*m[2]*m[9] + m[8]*m[1]*m[6] - m[8]*m[2]*m[5]

	det := m[0]*inv[0] + m[1]*inv[4] + m[2]*inv[8] + m[3]*inv[12]
	if det == 0 {
		return Mat4{}, false
	}
	det = 1 / det
	for i := range inv {
		inv[i] *= det
	}
	return inv, true
}

// AABB 是轴对齐包围盒，Min/Max 为闭区间端点。
type AABB struct {
	Min, Max Vec3
}

// EmptyAABB 返回空包围盒（Min=+Inf, Max=-Inf）。
func EmptyAABB() AABB {
	inf := math.Inf(1)
	return AABB{Min: Vec3{inf, inf, inf}, Max: Vec3{-inf, -inf, -inf}}
}

// IsEmpty 报告包围盒是否为空。
func (a AABB) IsEmpty() bool {
	return a.Min.X > a.Max.X || a.Min.Y > a.Max.Y || a.Min.Z > a.Max.Z
}

// Union 返回包含 a 与 b 的最小包围盒。
func (a AABB) Union(b AABB) AABB {
	return AABB{
		Min: Vec3{math.Min(a.Min.X, b.Min.X), math.Min(a.Min.Y, b.Min.Y), math.Min(a.Min.Z, b.Min.Z)},
		Max: Vec3{math.Max(a.Max.X, b.Max.X), math.Max(a.Max.Y, b.Max.Y), math.Max(a.Max.Z, b.Max.Z)},
	}
}

// Intersects 报告两个包围盒是否相交（含边界接触）。
func (a AABB) Intersects(b AABB) bool {
	if a.IsEmpty() || b.IsEmpty() {
		return false
	}
	return a.Min.X <= b.Max.X && a.Max.X >= b.Min.X &&
		a.Min.Y <= b.Max.Y && a.Max.Y >= b.Min.Y &&
		a.Min.Z <= b.Max.Z && a.Max.Z >= b.Min.Z
}

// ContainsPoint 报告点是否在包围盒内（含边界）。
func (a AABB) ContainsPoint(p Vec3) bool {
	if a.IsEmpty() {
		return false
	}
	return p.X >= a.Min.X && p.X <= a.Max.X &&
		p.Y >= a.Min.Y && p.Y <= a.Max.Y &&
		p.Z >= a.Min.Z && p.Z <= a.Max.Z
}

// Transformed 返回包围盒经矩阵变换后的轴对齐包围盒。
func (a AABB) Transformed(m Mat4) AABB {
	if a.IsEmpty() {
		return a
	}
	out := EmptyAABB()
	for i := 0; i < 8; i++ {
		p := Vec3{
			a.Min.X, a.Min.Y, a.Min.Z,
		}
		if i&1 != 0 {
			p.X = a.Max.X
		}
		if i&2 != 0 {
			p.Y = a.Max.Y
		}
		if i&4 != 0 {
			p.Z = a.Max.Z
		}
		out = out.Union(AABB{Min: m.TransformPoint(p), Max: m.TransformPoint(p)})
	}
	return out
}

// Ray 是一条射线，Dir 应归一化。
type Ray struct {
	Origin, Dir Vec3
}

// IntersectRay 计算射线与包围盒的求交（slab 法）。
//
// 边界语义（确定行为）：
//   - 射线起点在盒内（含边界）：命中，t = 0。
//   - 射线与盒相切（tmin == tmax）：视为命中。
//   - 方向分量为 0 且起点不在对应 slab 内：未命中。
//   - 命中仅在盒的“背面”（tmax < 0）：未命中。
func (a AABB) IntersectRay(r Ray) (t float64, ok bool) {
	if a.IsEmpty() {
		return 0, false
	}
	if a.ContainsPoint(r.Origin) {
		return 0, true
	}
	tmin, tmax := math.Inf(-1), math.Inf(1)
	axes := [3]struct{ o, d, lo, hi float64 }{
		{r.Origin.X, r.Dir.X, a.Min.X, a.Max.X},
		{r.Origin.Y, r.Dir.Y, a.Min.Y, a.Max.Y},
		{r.Origin.Z, r.Dir.Z, a.Min.Z, a.Max.Z},
	}
	for _, ax := range axes {
		if ax.d == 0 {
			if ax.o < ax.lo || ax.o > ax.hi {
				return 0, false
			}
			continue
		}
		t1 := (ax.lo - ax.o) / ax.d
		t2 := (ax.hi - ax.o) / ax.d
		if t1 > t2 {
			t1, t2 = t2, t1
		}
		if t1 > tmin {
			tmin = t1
		}
		if t2 < tmax {
			tmax = t2
		}
		if tmin > tmax {
			return 0, false
		}
	}
	if tmax < 0 {
		return 0, false
	}
	if tmin < 0 {
		return 0, true
	}
	return tmin, true
}

// Plane 是平面 Normal·p + D = 0，Normal 指向半空间外侧。
type Plane struct {
	Normal Vec3
	D      float64
}

// Frustum 是六个平面围成的视锥，顺序固定为
// Left, Right, Bottom, Top, Near, Far。
type Frustum [6]Plane

// FrustumFromMatrix 从视图-投影组合矩阵提取视锥（Gribb-Hartmann 方法）。
// 约定裁剪空间为 x,y,z ∈ [-w, w]（OpenGL 风格）。
func FrustumFromMatrix(m Mat4) Frustum {
	// 行向量：row(i)[j] = m[j*4+i]
	row := func(i int) [4]float64 {
		return [4]float64{m[i], m[4+i], m[8+i], m[12+i]}
	}
	r0, r1, r2, r3 := row(0), row(1), row(2), row(3)
	combine := func(a, b [4]float64, sa, sb float64) Plane {
		p := Plane{
			Normal: Vec3{sa*a[0] + sb*b[0], sa*a[1] + sb*b[1], sa*a[2] + sb*b[2]},
			D:      sa*a[3] + sb*b[3],
		}
		l := p.Normal.Length()
		if l != 0 {
			p.Normal = p.Normal.Scale(1 / l)
			p.D /= l
		}
		return p
	}
	return Frustum{
		combine(r3, r0, 1, 1),  // Left:   w + x
		combine(r3, r0, 1, -1), // Right:  w - x
		combine(r3, r1, 1, 1),  // Bottom: w + y
		combine(r3, r1, 1, -1), // Top:    w - y
		combine(r3, r2, 1, 1),  // Near:   w + z
		combine(r3, r2, 1, -1), // Far:    w - z
	}
}

// IntersectsAABB 报告包围盒是否与视锥相交或在视锥内。
// 空包围盒永远不相交。
func (f Frustum) IntersectsAABB(a AABB) bool {
	if a.IsEmpty() {
		return false
	}
	for _, p := range f {
		// 取平面正方向上的最远顶点（positive vertex）。
		v := Vec3{a.Min.X, a.Min.Y, a.Min.Z}
		if p.Normal.X >= 0 {
			v.X = a.Max.X
		}
		if p.Normal.Y >= 0 {
			v.Y = a.Max.Y
		}
		if p.Normal.Z >= 0 {
			v.Z = a.Max.Z
		}
		if p.Normal.Dot(v)+p.D < 0 {
			return false
		}
	}
	return true
}
