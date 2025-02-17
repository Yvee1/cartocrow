#ifndef CARTOCROW_TYPES_H
#define CARTOCROW_TYPES_H

#include "cartocrow/circle_segment_helpers/circle_tangents.h"

namespace cartocrow::kinetic_kelp {
using PointId = int;
using MSTEdge = std::pair<PointId, PointId>; // first should have the lower id.
using MST = std::vector<MSTEdge>;

struct Orbit {
	PointId pointId;
	Number<Exact> innerRadius;
	Number<Exact> outerRadius;
	CGAL::Orientation dir;
//	RationalRadiusCircle outerCircle(const InputInstance& input) const {
//		return {input[pointId].point, outerRadius};
//	}
//	RationalRadiusCircle innerCircle(const InputInstance& input) const {
//		return {input[pointId].point, innerRadius};
//	}
};

struct EdgeTopology {
	PointId source;
	PointId target;
	std::vector<Orbit> orbits;
};

using ElbowId = std::pair<MSTEdge, int>;
using StraightId = std::pair<MSTEdge, int>;
}

#endif //CARTOCROW_TYPES_H
