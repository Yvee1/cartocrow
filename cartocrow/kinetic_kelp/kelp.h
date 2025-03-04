#ifndef CARTOCROW_KELP_H
#define CARTOCROW_KELP_H

#include "../circle_segment_helpers/cs_types.h"

namespace cartocrow::kinetic_kelp {
class Kelp {
public:
    Kelp(const CSPolygonSet& polygonSet, double smoothing);
    CSPolygonSet polygonSet() const;

private:
    CSPolygonSet m_smoothedPolygonSet;
};
}

#endif //CARTOCROW_KELP_H
