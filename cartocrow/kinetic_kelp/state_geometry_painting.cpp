#include "state_geometry_painting.h"

#include "cartocrow/renderer/cs_render_helpers.h"

using namespace cartocrow::renderer;

namespace cartocrow::kinetic_kelp {
StateGeometryPainting::StateGeometryPainting(std::shared_ptr<StateGeometry> stateGeometry) :
        m_stateGeometry(std::move(stateGeometry)) {}

void StateGeometryPainting::paint(GeometryRenderer& renderer) const {
    renderer.setMode(GeometryRenderer::stroke);
    renderer.setStroke(Color(0, 0, 0), 1.0);
    for (const auto& [edge, geom] : m_stateGeometry->edgeGeometry) {
        renderer.draw(renderPath(geom.csPolygon()));
		renderer.setMode(GeometryRenderer::stroke | GeometryRenderer::fill);
		renderer.setFill(Color(200, 200, 255));
		for (const auto& elbow : geom.elbows) {
			renderer.draw(renderPath(elbow.csPolygon()));
		}
		renderer.setFill(Color(200, 255, 200));
		for (const auto& straight : geom.straights) {
			renderer.draw(renderPath(straight.csPolygon()));
		}
		renderer.setFill(Color(255, 200, 200));
		renderer.draw(renderPath(geom.startTerminal.csPolygon()));
		renderer.draw(renderPath(geom.endTerminal.csPolygon()));
    }
	renderer.setMode(GeometryRenderer::stroke);
    for (const auto& [vId, circle] : m_stateGeometry->vertexGeometry) {
        renderer.draw(circle);
    }
}
}