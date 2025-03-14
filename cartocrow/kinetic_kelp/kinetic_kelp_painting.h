#ifndef CARTOCROW_KELP_PAINTING_H
#define CARTOCROW_KELP_PAINTING_H

#include "state_geometry.h"
#include "draw_settings.h"

#include "../renderer/geometry_painting.h"

namespace cartocrow::kinetic_kelp {
class KineticKelpPainting : public renderer::GeometryPainting {
public:
    KineticKelpPainting(std::shared_ptr<std::vector<Kelp>> kelps, std::shared_ptr<InputInstance> input, std::shared_ptr<DrawSettings> ds);
    void paint(renderer::GeometryRenderer &renderer) const override;
private:
    std::shared_ptr<InputInstance> m_input;
    std::shared_ptr<std::vector<Kelp>> m_kelps;
    std::shared_ptr<DrawSettings> m_drawSettings;
};
}

#endif //CARTOCROW_KELP_PAINTING_H
