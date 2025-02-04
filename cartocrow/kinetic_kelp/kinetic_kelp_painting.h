#ifndef CARTOCROW_KELP_PAINTING_H
#define CARTOCROW_KELP_PAINTING_H

#include "state_geometry.h"
#include "../renderer/geometry_painting.h"

namespace cartocrow::kinetic_kelp {
class KineticKelpPainting : public renderer::GeometryPainting {
public:
    struct DrawSettings {
        double markRadius = 1.0;
        double strokeWidth = 1.0;
        double smoothing = 0.2;
        std::vector<Color> colors;
    };

    KineticKelpPainting(std::shared_ptr<StateGeometry> stateGeometry, std::shared_ptr<InputInstance> input, DrawSettings ds);
    void paint(renderer::GeometryRenderer &renderer) const override;
private:
    std::shared_ptr<InputInstance> m_input;
    std::shared_ptr<StateGeometry> m_stateGeometry;
    DrawSettings m_drawSettings;
};
}

#endif //CARTOCROW_KELP_PAINTING_H
