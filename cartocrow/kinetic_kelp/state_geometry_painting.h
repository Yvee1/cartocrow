#ifndef CARTOCROW_STATE_GEOMETRY_PAINTING_H
#define CARTOCROW_STATE_GEOMETRY_PAINTING_H

#include "state_geometry.h"

#include "../renderer/geometry_painting.h"

namespace cartocrow::kinetic_kelp {
class StateGeometryPainting : public renderer::GeometryPainting {
public:
    StateGeometryPainting(std::shared_ptr<StateGeometry> stateGeometry);
    void paint(renderer::GeometryRenderer &renderer) const override;
private:
    std::shared_ptr<StateGeometry> m_stateGeometry;
};
}

#endif //CARTOCROW_STATE_GEOMETRY_PAINTING_H
