#ifndef CARTOCROW_STATE_GEOMETRY_H
#define CARTOCROW_STATE_GEOMETRY_H

#include "../core/cs_types.h"

#include "state.h"
#include "input_instance.h"

namespace cartocrow::kinetic_kelp {
using EdgeGeometry = CSPolygon;

struct StateGeometry {
    std::map<MSTEdge, CSPolygon> edgeGeometry;
    std::map<VertexId, Circle<Exact>> vertexGeometry;
    std::vector<CSPolygonSet> mstGeometry;
};

CSPolygon edgeToGeometry(const EdgeTopology& edge, const InputInstance& input, const Settings& innerHalf);
StateGeometry stateToGeometry(const State& state, const InputInstance& input, const Settings& settings);
}

#endif //CARTOCROW_STATE_GEOMETRY_H
