#include "segment_delaunay_graph_helpers.h"

namespace cartocrow {
CGAL::Parabola_segment_2<CGAL::Segment_Delaunay_graph_traits_2<Inexact>>
approximate(const CGAL::Parabola_segment_2<CGAL::Segment_Delaunay_graph_traits_2<Exact>>& ps) {
	return CGAL::Parabola_segment_2<CGAL::Segment_Delaunay_graph_traits_2<Inexact>>(approximate(ps.center()), approximate(ps.line()), approximate(ps.p1), approximate(ps.p2));
}
}