#ifndef CARTOCROW_KELP_H
#define CARTOCROW_KELP_H

#include "../circle_segment_helpers/cs_types.h"

namespace cartocrow::kinetic_kelp {
class Kelp {
public:
    Kelp(const CSPolygonSet& polygonSet, std::optional<double> smoothing = std::nullopt);
    CSPolygonSet polygonSet() const;

private:
    CSPolygonSet m_kelp;
};
}

#endif //CARTOCROW_KELP_H
