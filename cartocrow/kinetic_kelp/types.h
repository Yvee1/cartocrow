#ifndef CARTOCROW_TYPES_H
#define CARTOCROW_TYPES_H

#include "cartocrow/circle_segment_helpers/circle_tangents.h"

namespace cartocrow::kinetic_kelp {
using PointId = int;
using MSTEdge = std::pair<PointId, PointId>;
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

namespace std {
template <>
struct hash<cartocrow::kinetic_kelp::MSTEdge>
{
    std::size_t operator()(const cartocrow::kinetic_kelp::MSTEdge& edge) const
    {
        // Compute individual hash values for member variables
        // http://stackoverflow.com/a/1646913/126995
        std::size_t res = 17;
        res = res * 31 + hash<int>{}(edge.first);
        res = res * 31 + hash<int>{}(edge.second);
        return res;
    }
};

template <>
struct hash<cartocrow::kinetic_kelp::ElbowId>
{
    std::size_t operator()(const cartocrow::kinetic_kelp::ElbowId& elbowId) const
    {
        // Compute individual hash values for member variables
        // http://stackoverflow.com/a/1646913/126995
        std::size_t res = 17;
        res = res * 31 + hash<cartocrow::kinetic_kelp::MSTEdge>{}(elbowId.first);
        res = res * 31 + hash<int>{}(elbowId.second);
        return res;
    }
};
}

#endif //CARTOCROW_TYPES_H
