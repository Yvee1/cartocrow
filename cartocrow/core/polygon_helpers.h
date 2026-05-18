/*
Copyright (C) 2026  TU Eindhoven

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

#pragma once

#include "core.h"

namespace cartocrow {

template <class Kernel> Point<Kernel> compute_centroid(const Polygon<Kernel>& polygon) {

	int n = polygon.size();

	switch (n) {
	case 0:
		return Point<Kernel>();
	case 1:
		return polygon[0];
	default:
		Number<Kernel> A = 0;
		Number<Kernel> cx = 0;
		Number<Kernel> cy = 0;

		for (int i = 0; i < n; i++) {
			const Point<Kernel>& p = polygon[i == 0 ? n - 1 : i - 1];
			const Point<Kernel>& q = polygon[i];
			Number<Kernel> cross = p.x() * q.y() - p.y() * q.x();
			A += cross;
			cx += (p.x() + q.x()) * cross;
			cy += (p.y() + q.y()) * cross;
		}

		A *= 3;
		cx = cx / A;
		cy = cy / A;
		return Point<Kernel>(cx, cy);
	}
}

template <class Kernel> Point<Kernel> compute_centroid(const std::vector<Polygon<Kernel>>& polygons) {

	// NB: this assumes that the outerboundaries are CCW
	// and the inner boundaries (holes) are CW
	Number<Kernel> cx = 0, cy = 0;
	Number<Kernel> totalarea = 0;
	for (const Polygon<Kernel>& p : polygons) {
		Number<Kernel> a = p.area();
		totalarea += a;
		Point<Kernel> ctr = compute_centroid(p);
		cx += a * ctr.x();
		cy += a * ctr.y();
	}
	cx /= totalarea;
	cy /= totalarea;

	return Point<Kernel>(cx, cy);
}

template <class Kernel> Point<Kernel> compute_centroid(const PolygonWithHoles<Kernel>& polygon) {

	Number<Kernel> cx = 0, cy = 0;
	Number<Kernel> totalarea = 0;

	{
		const Polygon<Kernel>& p = polygon.outer_boundary();
		Number<Kernel> a = CGAL::abs(p.area());
		totalarea += a;
		Point<Kernel> ctr = compute_centroid(p);
		cx += a * ctr.x();
		cy += a * ctr.y();
	}

	for (const Polygon<Kernel>& p : polygon.holes()) {
		Number<Kernel> a = -1 * CGAL::abs(p.area());
		totalarea += a;
		Point<Kernel> ctr = compute_centroid(p);
		cx += a * ctr.x();
		cy += a * ctr.y();
	}

	cx /= totalarea;
	cy /= totalarea;		

	return Point<Kernel>(cx, cy);
}

} // namespace cartocrow