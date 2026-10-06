// Package scenegraph 提供场景图与空间查询内核。
//
// 坐标约定与查询语义详见 doc.go。
package scenegraph

// Vec3 为三维向量，采用右手坐标系。
type Vec3 struct {
	X, Y, Z float64
}

// Add 返回向量加法结果。
func (a Vec3) Add(b Vec3) Vec3 { return Vec3{a.X + b.X, a.Y + b.Y, a.Z + b.Z} }

// Sub 返回向量减法结果。
func (a Vec3) Sub(b Vec3) Vec3 { return Vec3{a.X - b.X, a.Y - b.Y, a.Z - b.Z} }

// Scale 返回向量数乘结果。
func (a Vec3) Scale(s float64) Vec3 { return Vec3{a.X * s, a.Y * s, a.Z * s} }

// Dot 返回点积。
func (a Vec3) Dot(b Vec3) float64 { return a.X*b.X + a.Y*b.Y + a.Z*b.Z }

// Cross 返回叉积。
func (a Vec3) Cross(b Vec3) Vec3 {
	return Vec3{
		X: a.Y*b.Z - a.Z*b.Y,
		Y: a.Z*b.X - a.X*b.Z,
		Z: a.X*b.Y - a.Y*b.X,
	}
}

// Len 返回向量长度。
func (a Vec3) Len() float64 { return sqrt(a.Dot(a)) }

// Normalize 返回单位向量；零向量返回零向量。
func (a Vec3) Normalize() Vec3 {
	l := a.Len()
	if l == 0 {
		return Vec3{}
	}
	return a.Scale(1 / l)
}

// Quat 为单位四元数 (w + xi + yj + zk)，表示旋转。
type Quat struct {
	W, X, Y, Z float64
}

// QuatIdentity 返回单位四元数。
func QuatIdentity() Quat { return Quat{W: 1} }

// QuatFromAxisAngle 由旋转轴与弧度角构造四元数。
func QuatFromAxisAngle(axis Vec3, angle float64) Quat {
	a := axis.Normalize()
	h := angle / 2
	s := sin(h)
	return Quat{W: cos(h), X: a.X * s, Y: a.Y * s, Z: a.Z * s}
}

// Mul 返回四元数乘法（旋转复合）。
func (q Quat) Mul(r Quat) Quat {
	return Quat{
		W: q.W*r.W - q.X*r.X - q.Y*r.Y - q.Z*r.Z,
		X: q.W*r.X + q.X*r.W + q.Y*r.Z - q.Z*r.Y,
		Y: q.W*r.Y - q.X*r.Z + q.Y*r.W + q.Z*r.X,
		Z: q.W*r.Z + q.X*r.Y - q.Y*r.X + q.Z*r.W,
	}
}

// Rotate 用四元数旋转向量。
func (q Quat) Rotate(v Vec3) Vec3 {
	qv := Vec3{q.X, q.Y, q.Z}
	uv := qv.Cross(v)
	uuv := qv.Cross(uv)
	return v.Add(uv.Scale(2 * q.W)).Add(uuv.Scale(2))
}

// Mat4 为 4x4 列主序仿射变换矩阵（最后一行为 0 0 0 1）。
// M[col*4+row]，与常规图形学约定一致：v' = M * v。
type Mat4 [16]float64

// Mat4Identity 返回单位矩阵。
func Mat4Identity() Mat4 {
	return Mat4{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}
}

// Mat4Compose 由平移、旋转、缩放组合仿射矩阵：M = T * R * S。
func Mat4Compose(t Vec3, q Quat, s Vec3) Mat4 {
	x2, y2, z2 := q.X+q.X, q.Y+q.Y, q.Z+q.Z
	xx, xy, xz := q.X*x2, q.X*y2, q.X*z2
	yy, yz, zz := q.Y*y2, q.Y*z2, q.Z*z2
	wx, wy, wz := q.W*x2, q.W*y2, q.W*z2
	return Mat4{
		// 列主序：每 4 个元素为一列，平移在第 3 列（索引 12-14）。
		(1 - (yy + zz)) * s.X, (xy + wz) * s.X, (xz - wy) * s.X, 0,
		(xy - wz) * s.Y, (1 - (xx + zz)) * s.Y, (yz + wx) * s.Y, 0,
		(xz + wy) * s.Z, (yz - wx) * s.Z, (1 - (xx + yy)) * s.Z, 0,
		t.X, t.Y, t.Z, 1,
	}
}

// Mul 返回矩阵乘法 a*b（先应用 b，再应用 a）。
func (a Mat4) Mul(b Mat4) Mat4 {
	var r Mat4
	for c := 0; c < 4; c++ {
		for row := 0; row < 4; row++ {
			var s float64
			for k := 0; k < 4; k++ {
				s += a[k*4+row] * b[c*4+k]
			}
			r[c*4+row] = s
		}
	}
	return r
}

// TransformPoint 对点应用仿射变换（含平移）。
func (m Mat4) TransformPoint(p Vec3) Vec3 {
	return Vec3{
		X: m[0]*p.X + m[4]*p.Y + m[8]*p.Z + m[12],
		Y: m[1]*p.X + m[5]*p.Y + m[9]*p.Z + m[13],
		Z: m[2]*p.X + m[6]*p.Y + m[10]*p.Z + m[14],
	}
}

// AABB 为轴对齐包围盒，Min/Max 均为闭区间端点。
type AABB struct {
	Min, Max Vec3
}

// EmptyAABB 返回一个“空”包围盒（Min=+Inf, Max=-Inf），用于并集累积。
func EmptyAABB() AABB {
	return AABB{
		Min: Vec3{inf(), inf(), inf()},
		Max: Vec3{-inf(), -inf(), -inf()},
	}
}

// AABBFromCenterExtents 由中心与半尺寸构造包围盒。
func AABBFromCenterExtents(c, e Vec3) AABB {
	return AABB{Min: c.Sub(e), Max: c.Add(e)}
}

// Union 返回两个包围盒的并。
func (a AABB) Union(b AABB) AABB {
	return AABB{
		Min: Vec3{minf(a.Min.X, b.Min.X), minf(a.Min.Y, b.Min.Y), minf(a.Min.Z, b.Min.Z)},
		Max: Vec3{maxf(a.Max.X, b.Max.X), maxf(a.Max.Y, b.Max.Y), maxf(a.Max.Z, b.Max.Z)},
	}
}

// Intersects 判断两个包围盒是否相交（闭区间，接触算相交）。
func (a AABB) Intersects(b AABB) bool {
	return a.Min.X <= b.Max.X && a.Max.X >= b.Min.X &&
		a.Min.Y <= b.Max.Y && a.Max.Y >= b.Min.Y &&
		a.Min.Z <= b.Max.Z && a.Max.Z >= b.Min.Z
}

// Contains 判断点是否在包围盒内（闭区间）。
func (a AABB) Contains(p Vec3) bool {
	return p.X >= a.Min.X && p.X <= a.Max.X &&
		p.Y >= a.Min.Y && p.Y <= a.Max.Y &&
		p.Z >= a.Min.Z && p.Z <= a.Max.Z
}

// Corners 返回包围盒 8 个角点。
func (a AABB) Corners() [8]Vec3 {
	return [8]Vec3{
		{a.Min.X, a.Min.Y, a.Min.Z},
		{a.Max.X, a.Min.Y, a.Min.Z},
		{a.Min.X, a.Max.Y, a.Min.Z},
		{a.Max.X, a.Max.Y, a.Min.Z},
		{a.Min.X, a.Min.Y, a.Max.Z},
		{a.Max.X, a.Min.Y, a.Max.Z},
		{a.Min.X, a.Max.Y, a.Max.Z},
		{a.Max.X, a.Max.Y, a.Max.Z},
	}
}

// Transform 返回包围盒经仿射变换后的轴对齐包围盒（8 角点重取包围）。
func (a AABB) Transform(m Mat4) AABB {
	r := EmptyAABB()
	for _, c := range a.Corners() {
		p := m.TransformPoint(c)
		r = r.Union(AABB{Min: p, Max: p})
	}
	return r
}

// Ray 为三维射线：Origin + t*Dir，t >= 0。
type Ray struct {
	Origin, Dir Vec3
}

// HitAABB 用 slab 法求射线与包围盒的相交参数区间。
// 返回值 ok 表示是否相交；tNear 为进入参数（起点在盒内时为 0），tFar 为离开参数。
// 相切（射线恰好擦过盒面）视为相交。
func HitAABB(r Ray, b AABB) (tNear, tFar float64, ok bool) {
	tNear, tFar = 0, inf()
	axes := [3]struct{ o, d, mn, mx float64 }{
		{r.Origin.X, r.Dir.X, b.Min.X, b.Max.X},
		{r.Origin.Y, r.Dir.Y, b.Min.Y, b.Max.Y},
		{r.Origin.Z, r.Dir.Z, b.Min.Z, b.Max.Z},
	}
	for _, a := range axes {
		if a.d == 0 {
			// 射线与该轴平行：起点必须在 slab 内，否则不相交。
			if a.o < a.mn || a.o > a.mx {
				return 0, 0, false
			}
			continue
		}
		t1 := (a.mn - a.o) / a.d
		t2 := (a.mx - a.o) / a.d
		if t1 > t2 {
			t1, t2 = t2, t1
		}
		if t1 > tNear {
			tNear = t1
		}
		if t2 < tFar {
			tFar = t2
		}
		if tNear > tFar {
			return 0, 0, false
		}
	}
	return tNear, tFar, true
}

// Plane 为平面：法线 N（单位向量）与偏移 D，满足 N·p + D = 0。
// 半空间 N·p + D >= 0 视为平面内侧。
type Plane struct {
	N Vec3
	D float64
}

// Distance 返回点到平面的有符号距离。
func (p Plane) Distance(v Vec3) float64 { return p.N.Dot(v) + p.D }

// Frustum 为视锥，由 6 个平面组成，内侧为各平面正半空间的交集。
type Frustum [6]Plane

// FrustumFromMatrix 由视图-投影组合矩阵提取视锥平面（Gribb-Hartmann 法）。
// 矩阵约定与 Mat4 相同：列主序，clip = M * world。
func FrustumFromMatrix(m Mat4) Frustum {
	row := func(i int) Vec3 { return Vec3{m[i], m[4+i], m[8+i]} }
	w := func(i int) float64 { return m[12+i] }
	r3 := row(3)
	mk := func(n Vec3, d float64) Plane {
		l := n.Len()
		if l == 0 {
			return Plane{N: n, D: d}
		}
		return Plane{N: n.Scale(1 / l), D: d / l}
	}
	return Frustum{
		mk(r3.Add(row(0)), w(3)+w(0)), // left
		mk(r3.Sub(row(0)), w(3)-w(0)), // right
		mk(r3.Add(row(1)), w(3)+w(1)), // bottom
		mk(r3.Sub(row(1)), w(3)-w(1)), // top
		mk(r3.Add(row(2)), w(3)+w(2)), // near
		mk(r3.Sub(row(2)), w(3)-w(2)), // far
	}
}

// IntersectsAABB 保守判定包围盒是否与视锥相交或在视锥内。
// 只要包围盒在任一平面的完全外侧即判定不相交。
func (f Frustum) IntersectsAABB(b AABB) bool {
	for _, p := range f {
		// 取沿平面法线方向最远的顶点（positive vertex）。
		v := b.Min
		if p.N.X >= 0 {
			v.X = b.Max.X
		}
		if p.N.Y >= 0 {
			v.Y = b.Max.Y
		}
		if p.N.Z >= 0 {
			v.Z = b.Max.Z
		}
		if p.Distance(v) < 0 {
			return false
		}
	}
	return true
}
