#include "kinetic_kelp_painting.h"

#include "cartocrow/renderer/cs_render_helpers.h"
#include "cartocrow/core/cavc_helpers.h"

using namespace cartocrow::renderer;

namespace cartocrow::kinetic_kelp {
KineticKelpPainting::KineticKelpPainting(std::shared_ptr<StateGeometry> stateGeometry, std::shared_ptr<InputInstance> input, DrawSettings ds)
    : m_stateGeometry(std::move(stateGeometry)), m_input(std::move(input)), m_drawSettings(std::move(ds)) {};

void KineticKelpPainting::paint(renderer::GeometryRenderer &renderer) const {
    renderer.setMode(GeometryRenderer::stroke | GeometryRenderer::fill);
    renderer.setStroke(Color(0, 0, 0), m_drawSettings.strokeWidth, true);
    for (int k = 0; k < m_input->numCategories(); ++k) {
        renderer.setFill(m_drawSettings.colors[k]);
        auto mst = m_stateGeometry->mstGeometry[k];
        auto smoothed = approximateClosing(mst, m_drawSettings.smoothing);
        smoothed.join(mst);
        renderer.draw(renderPath(smoothed));
    }
    for (const auto& cp : m_input->catPoints()) {
        renderer.setFill(Color(0, 0, 0));
        renderer.draw(Circle<Exact>(cp.point, m_drawSettings.markRadius));
    }
}
}