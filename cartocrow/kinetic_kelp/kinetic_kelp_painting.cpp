#include "kinetic_kelp_painting.h"

#include "cartocrow/circle_segment_helpers/cs_render_helpers.h"
#include "cartocrow/circle_segment_helpers/cavc_helpers.h"

using namespace cartocrow::renderer;

namespace cartocrow::kinetic_kelp {
KineticKelpPainting::KineticKelpPainting(std::shared_ptr<std::vector<Kelp>> kelps, std::shared_ptr<InputInstance> input, DrawSettings ds)
    : m_kelps(std::move(kelps)), m_input(std::move(input)), m_drawSettings(std::move(ds)) {};

void KineticKelpPainting::paint(renderer::GeometryRenderer &renderer) const {
    if (m_kelps->empty()) return;
    renderer.setMode(GeometryRenderer::stroke | GeometryRenderer::fill);
    renderer.setStroke(Color(0, 0, 0), m_drawSettings.strokeWidth, true);
    for (int k = 0; k < m_input->numCategories(); ++k) {
        renderer.setFill(m_drawSettings.colors[k]);
        renderer.draw(renderPath(m_kelps->at(k).polygonSet()));
    }
    for (const auto& cp : m_input->catPoints()) {
        renderer.setFill(Color(0, 0, 0));
        renderer.draw(Circle<Exact>(cp.point, m_drawSettings.markRadius));
    }
}
}