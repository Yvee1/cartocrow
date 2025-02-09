#ifndef CARTOCROW_ROUTE_EDGES_H
#define CARTOCROW_ROUTE_EDGES_H

#include "input_instance.h"
#include "state.h"
#include "cartocrow/renderer/geometry_renderer.h"

namespace cartocrow::kinetic_kelp {
void routeEdges(const InputInstance& input, const Settings& settings, renderer::GeometryRenderer& renderer);
}

#endif //CARTOCROW_ROUTE_EDGES_H
