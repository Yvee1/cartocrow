#include "pseudotriangulation.h"

namespace cartocrow::kinetic_kelp {
bool circlePointLiesOnArc(const Point<Exact>& point, const RationalCircularArc& arc) {
	auto sd = (arc.source - arc.circle.center).direction();
	auto td = (arc.target - arc.circle.center).direction();
	auto d = (point - arc.circle.center).direction();
	return arc.orientation == CGAL::COUNTERCLOCKWISE ? d.counterclockwise_in_between(sd, td) : d.counterclockwise_in_between(td, sd);
}
}