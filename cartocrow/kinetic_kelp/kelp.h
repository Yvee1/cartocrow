#ifndef CARTOCROW_KELP_H
#define CARTOCROW_KELP_H

#include "../circle_segment_helpers/cs_types.h"

namespace cartocrow::kinetic_kelp {
class Kelp {
public:
    Kelp(CSPolygonWithHoles polygon, double smoothing);
    CSPolygonWithHoles polygon() const;

private:
    CSPolygonWithHoles m_smoothedPolygon;
};
}

#endif //CARTOCROW_KELP_H
