#include "state_geometry_painting.h"

#include "cartocrow/circle_segment_helpers/cs_render_helpers.h"

using namespace cartocrow::renderer;

namespace cartocrow::kinetic_kelp {
StateGeometryPainting::StateGeometryPainting(std::shared_ptr<StateGeometry> stateGeometry, double strokeWidth) :
        m_stateGeometry(std::move(stateGeometry)), m_strokeWidth(strokeWidth) {}

void StateGeometryPainting::paint(GeometryRenderer& renderer) const {
    renderer.setMode(GeometryRenderer::stroke);
    renderer.setStroke(Color(0, 0, 0), m_strokeWidth);
    for (const auto& [edge, geom] : m_stateGeometry->edgeGeometry) {
		renderer.setMode(GeometryRenderer::stroke | GeometryRenderer::fill);
        renderer.setFillOpacity(150);
		renderer.setFill(Color(121, 186, 122));
		for (const auto& elbow : geom.elbows) {
			renderer.draw(renderPath(elbow.csPolygon()));
		}
		renderer.setFill(Color(179, 179, 179));
		for (const auto& straight : geom.straights) {
			renderer.draw(renderPath(straight.csPolygon()));
		}
		renderer.setFill(Color(179, 148, 108));
		renderer.draw(renderPath(geom.startTerminal.csPolygon()));
		renderer.draw(renderPath(geom.endTerminal.csPolygon()));
        renderer.setFillOpacity(255);
    }
	renderer.setMode(GeometryRenderer::stroke);
    for (const auto& [vId, circle] : m_stateGeometry->vertexGeometry) {
        renderer.draw(circle.circle());
    }
}
}