#ifndef CARTOCROW_TRANSFORM_HELPERS_H
#define CARTOCROW_TRANSFORM_HELPERS_H

#include "core.h"
#include "rectangle_helpers.h"

namespace cartocrow {
template <class K>
PolygonWithHoles<K> transform(const CGAL::Aff_transformation_2<K> &t, const PolygonWithHoles<K> &pwh) {
	Polygon<K> outerT;
	if (!pwh.is_unbounded()) {
		outerT = transform(t, pwh.outer_boundary());
	}
	std::vector <Polygon<K>> holesT;
	for (const auto &h: pwh.holes()) {
		holesT.push_back(transform(t, h));
	}
	return {outerT, holesT.begin(), holesT.end()};
}

template <class K>
PolygonSet<K> transform(const CGAL::Aff_transformation_2<K> &t, const PolygonSet<K> &ps) {
	PolygonSet<K> transformed;
	std::vector<PolygonWithHoles<K>> pwhs;
	ps.polygons_with_holes(std::back_inserter(pwhs));
	for (const auto& pwh : pwhs) {
		transformed.insert(transform(t, pwh));
	}
	return transformed;
}

template <class K>
CGAL::Aff_transformation_2<K> fitInto(const Rectangle<K>& toFit, const Rectangle<K>& into) {
	CGAL::Aff_transformation_2<K> move1(CGAL::TRANSLATION, CGAL::ORIGIN - centroid(toFit));
	CGAL::Aff_transformation_2<K> move2(CGAL::TRANSLATION, centroid(into) - CGAL::ORIGIN);
	CGAL::Aff_transformation_2<K> scale(CGAL::SCALING, std::min(width(into) / width(toFit), height(into) / height(toFit)));
	return move2 * scale * move1;
}

CGAL::Aff_transformation_2<Inexact> fitInto(const Box& toFit, const Box& into);
CGAL::Aff_transformation_2<Inexact> fitInto(const Rectangle<Inexact>& toFit, const Box& into);
CGAL::Aff_transformation_2<Inexact> fitInto(const Box& toFit, const Rectangle<Inexact>& into);
}

#endif //CARTOCROW_TRANSFORM_HELPERS_H
