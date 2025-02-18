#include "pseudotriangulation_painting.h"

using namespace cartocrow::renderer;

namespace cartocrow::kinetic_kelp {
void PseudotriangulationPainting::paint(GeometryRenderer &renderer) const {
	renderer.setMode(GeometryRenderer::stroke);
	renderer.setStroke(Color(0, 102, 202), 3.0);
	for (const auto& [_, t] : m_ptg->m_tangents) {
		renderer.draw(t.polyline());
	}

	renderer.setStroke(Color(0, 0, 0), 3.0);
	for (const auto& [_, obj] : m_ptg->m_tangentObject) {
		if (auto cp = std::get_if<RationalRadiusCircle>(&obj)) {
			renderer.draw(cp->circle());
		} else {
			auto p = std::get<Point<Exact>>(obj);
			renderer.draw(p);
		}
	}
}
}