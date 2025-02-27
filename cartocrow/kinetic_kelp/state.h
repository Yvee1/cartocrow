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

	std::pair<PointId, PointId> straightEndpoints(const StraightId& straightId) const {
		auto [edge, orbitIt] = straightId;
		auto& orbits = edgeTopology.at(edge).orbits;
		PointId straightTarget;
		if (orbitIt == orbits.end()) {
			straightTarget = edge.second;
		} else {
			straightTarget = orbits.back().pointId;
		}
		PointId straightSource;
		if (orbitIt == orbits.begin()) {
			straightSource = edge.first;
		} else {
			--orbitIt;
			straightSource = orbitIt->pointId;
		}
		return {straightSource, straightTarget};
	}
};
}

#endif //CARTOCROW_STATE_H
