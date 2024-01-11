/*
The CartoCrow library implements algorithmic geo-visualization methods,
developed at TU Eindhoven.
Copyright (C) 2021  Netherlands eScience Center and TU Eindhoven

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "geometry_renderer.h"
#include <CGAL/number_utils.h>

namespace cartocrow::renderer {

void GeometryRenderer::draw(const Point<Exact>& p) {
	draw(approximate(p));
}

void GeometryRenderer::draw(const Segment<Exact>& s) {
	draw(approximate(s));
}

void GeometryRenderer::draw(const Polygon<Exact>& p) {
	draw(approximate(p));
}

void GeometryRenderer::draw(const PolygonWithHoles<Exact>& p) {
	draw(approximate(p));
}

void GeometryRenderer::draw(const Circle<Exact>& c) {
	draw(approximate(c));
}

void GeometryRenderer::draw(const PolygonSet<Inexact>& ps) {
	std::vector<PolygonWithHoles<Inexact>> polygons;
	ps.polygons_with_holes(std::back_inserter(polygons));
	for (const PolygonWithHoles<Inexact>& p : polygons) {
		draw(p);
	}
}

void GeometryRenderer::draw(const PolygonSet<Exact>& ps) {
	draw(approximate(ps));
}

void GeometryRenderer::draw(const Line<Exact>& l) {
	draw(approximate(l));
}

void GeometryRenderer::draw(const Ray<Exact>& r) {
	draw(approximate(r));
}

void GeometryRenderer::drawText(const Point<Exact>& p, const std::string& s) {
	drawText(approximate(p), s);
}

//QPainter& GeometryRenderer::getQPainter() {
//	throw std::runtime_error("No QPainter.");
//}

void GeometryRenderer::draw(const BezierCurve& c) {
	BezierSpline spline;
	spline.AppendCurve(c);
	draw(spline);
}

void GeometryRenderer::draw(const Polyline<Exact>& p) {
	draw(approximate(p));
}

} // namespace cartocrow::renderer
