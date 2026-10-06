package scenegraph

import "math"

func sqrt(x float64) float64    { return math.Sqrt(x) }
func sin(x float64) float64     { return math.Sin(x) }
func cos(x float64) float64     { return math.Cos(x) }
func inf() float64              { return math.Inf(1) }
func minf(a, b float64) float64 { return math.Min(a, b) }
func maxf(a, b float64) float64 { return math.Max(a, b) }
