#pragma once

#include "core.h"
#include "segment_delaunay_graph_helpers_cgal_adapted.h"

namespace cartocrow {
/// Exposes the p1 and p2 member variables.
/// Not needed anymore in CGAL 6.0 and up (https://github.com/CGAL/cgal/pull/7976)
template <class Gt>
class Open_Parabola_segment_2 : public CGAL::Parabola_segment_2<Gt> {
  public:
	CGAL::Parabola_segment_2<Gt>::Point_2 get_p1() {
		return this->p1;
	}
	CGAL::Parabola_segment_2<Gt>::Point_2 get_p2() {
		return this->p2;
	}
};

template <class SDG>
std::variant<typename SDG::Geom_traits::Point_2, typename SDG::Geom_traits::Segment_2>
site_projection(const SDG& delaunay, const typename SDG::Edge& edge, const typename SDG::Site_2& site) {
	if (site.is_point()) {
		return { site.point() };
	} else {
		using Gt = SDG::Geom_traits;
		typename Gt::Segment_2 s;
		CGAL::Parabola_segment_2<Gt> ps;
		CGAL::Object o = delaunay.primal(edge);

		// Ray and line cases cannot occur because they require both sites to be a point
		if (CGAL::assign(s, o)) {
			auto start = site.segment().supporting_line().projection(s.source());
			auto end = site.segment().supporting_line().projection(s.end());
			return { Segment<Inexact>(start, end) };
		}
		else if (CGAL::assign(ps, o)) {
			// Roundabout way to obtain start and end of parabolic segment because they are protected -_-
			Open_Parabola_segment_2 ops{ps};
			auto p1 = ops.get_p1();
			auto p2 = ops.get_p2();

			if (std::isnan(p1.x()) || std::isnan(p2.x())) {
				return {typename Gt::Segment_2(typename Gt::Point_2(0.0, 0.0), typename Gt::Point_2(1.0, 0.0))};
			}

			auto start = site.segment().supporting_line().projection(p1);
			auto end = site.segment().supporting_line().projection(p2);

			return {Segment<Inexact>(start, end)};
		}
		else {
			throw std::runtime_error("Impossible: a segment Voronoi edge is neither a line segment nor a parabolic "
			                         "segment, but at least one of its sites is a line segment.");
		}
	}
}
}