#include "state_geometry_painting.h"

#include "cartocrow/renderer/cs_render_helpers.h"

using namespace cartocrow::renderer;

namespace cartocrow::kinetic_kelp {
StateGeometryPainting::StateGeometryPainting(std::shared_ptr<StateGeometry> stateGeometry) :
        m_stateGeometry(std::move(stateGeometry)) {}

void StateGeometryPainting::paint(GeometryRenderer& renderer) const {
    renderer.setMode(GeometryRenderer::stroke);
    renderer.setStroke(Color(0, 0, 0), 1.0);
    for (const auto& [edge, csPolygon] : m_stateGeometry->edgeGeometry) {
        renderer.draw(renderPath(csPolygon));
    }
    for (const auto& [vId, circle] : m_stateGeometry->vertexGeometry) {
        renderer.draw(circle);
    }
}
}