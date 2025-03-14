#include "kelp.h"

#include "../circle_segment_helpers/cavc_helpers.h"

namespace cartocrow::kinetic_kelp {
Kelp::Kelp(const CSPolygonSet& polygonSet, std::optional<double> smoothing) {
    if (smoothing.has_value()) {
        m_kelp = approximateClosing(polygonSet, *smoothing);
    } else {
        m_kelp = polygonSet;
    }
}

CSPolygonSet Kelp::polygonSet() const {
    return m_kelp;
}
}
