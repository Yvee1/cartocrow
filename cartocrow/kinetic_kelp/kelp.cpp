#include "kelp.h"

#include "../circle_segment_helpers/cavc_helpers.h"

namespace cartocrow::kinetic_kelp {
Kelp::Kelp(const CSPolygonSet& polygonSet, double smoothing) {
    m_smoothedPolygonSet = approximateClosing(polygonSet, smoothing);
}

CSPolygonSet Kelp::polygonSet() const {
    return m_smoothedPolygonSet;
}
}
