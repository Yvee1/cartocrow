#ifndef CARTOCROW_CAVC_HELPERS_HPP
#define CARTOCROW_CAVC_HELPERS_HPP

#include "../types.h"
#include "cavc/include/cavc/polylineoffset.hpp"

namespace cartocrow::simplesets {
// X monotone curves as input
template <class InputIterator>
cavc::Polyline<double> cavcPolyline(InputIterator start, InputIterator end, bool closed) {
	cavc::Polyline<double> polyline;
	auto processCurve = [&polyline](const X_monotone_curve_2& xmCurve) {
		Point<Inexact> s = approximateAlgebraic(xmCurve.source());
		Point<Inexact> t = approximateAlgebraic(xmCurve.target());
		if (xmCurve.is_linear()) {
			polyline.addVertex(s.x(), s.y(), 0);
		} else {
			auto circle = approximate(xmCurve.supporting_circle());
			auto center = circle.center();
			auto mid = CGAL::midpoint(s, t);
			auto d = sqrt(CGAL::squared_distance(mid, center));
			Number<Inexact> r = sqrt(circle.squared_radius());
			auto bulge = (r - d) / sqrt(CGAL::squared_distance(mid, s));
			auto orientation = xmCurve.orientation();
			auto sign = orientation == CGAL::COUNTERCLOCKWISE ? 1 : -1;
			polyline.addVertex(s.x(), s.y(), sign * bulge);
		}
	};

	for (auto cit = start; cit != end; ++cit) {
		processCurve(*cit);
	}

	if (closed) {
		polyline.isClosed() = true;
	} else {
		auto last = end;
		--last;
		Point<Inexact> t = approximateAlgebraic(last->target());
		polyline.addVertex(t.x(), t.y(), 0);
	}

	return polyline;
}
}

#endif //CARTOCROW_CAVC_HELPERS_HPP
