#ifndef CARTOCROW_STATE_H
#define CARTOCROW_STATE_H

#include "cartocrow/core/core.h"

#include "cartocrow/circle_segment_helpers/circle_tangents.h"

#include "cat_point.h"
#include "input_instance.h"
#include "trajectory.h"
#include "types.h"

namespace cartocrow::kinetic_kelp {
struct State {
    std::vector<MST> msts;
    std::map<MSTEdge, EdgeTopology> edgeTopology;
	std::vector<std::list<ElbowId>> pointIdToElbows;
};

struct Settings {
    Number<Exact> vertexRadius = 1.0;
    Number<Exact> edgeWidth = 0.3;
};
}

#endif //CARTOCROW_STATE_H
