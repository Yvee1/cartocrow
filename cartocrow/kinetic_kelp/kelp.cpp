#include "kelp.h"

#include "../circle_segment_helpers/cavc_helpers.h"

namespace cartocrow::kinetic_kelp {
Kelp::Kelp(CSPolygonWithHoles polygon, double smoothing) {
    CSPolygonSet polygonSet(polygon);
    std::vector<CSPolygonWithHoles> smoothedPolygons;
    auto smoothedSet = approximateClosing(polygonSet, smoothing);
    smoothedSet.polygons_with_holes(std::back_inserter(smoothedPolygons));
    if (smoothedPolygons.size() != 1) {
        throw std::runtime_error("A kelp cannot consist of multiple components!");
    }
    m_smoothedPolygon = smoothedPolygons[0];
}

CSPolygonWithHoles Kelp::polygon() const {
    return m_smoothedPolygon;
}
}
