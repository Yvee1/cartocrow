#ifndef CARTOCROW_STATE_H
#define CARTOCROW_STATE_H

#include "cartocrow/core/core.h"

#include "cartocrow/circle_segment_helpers/circle_tangents.h"

#include "cat_point.h"
#include "input_instance.h"
#include "trajectory.h"

namespace cartocrow::kinetic_kelp {
using MSTEdge = std::pair<VertexId, VertexId>; // first should have the lower id.
using MST = std::vector<MSTEdge>;

struct Orbit {
    VertexId vertexId;
    Number<Exact> innerRadius;
    Number<Exact> outerRadius;
    CGAL::Orientation dir;
    RationalRadiusCircle outerCircle(const InputInstance& input) const {
        return {input[vertexId].point, outerRadius};
    }
    RationalRadiusCircle innerCircle(const InputInstance& input) const {
        return {input[vertexId].point, innerRadius};
    }
};

struct EdgeTopology {
    VertexId source;
    VertexId target;
    std::vector<Orbit> orbits;
};

using ElbowId = std::pair<MSTEdge, int>;
using StraightId = std::pair<MSTEdge, int>;

struct State {
    std::vector<MST> msts;
    std::map<MSTEdge, EdgeTopology> edgeTopology;
};

struct Settings {
    Number<Exact> vertexRadius = 1.0;
    Number<Exact> edgeWidth = 0.3;
};
}

#endif //CARTOCROW_STATE_H
