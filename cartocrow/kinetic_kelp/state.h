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
    std::vector<std::list<MSTEdge>> pointIdToEdges;
};
}

#endif //CARTOCROW_STATE_H
