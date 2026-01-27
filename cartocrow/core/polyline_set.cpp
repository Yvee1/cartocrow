#include "polyline_set.h"

namespace cartocrow {
PolylineSet<Inexact>
approximate(const PolylineSet<Exact>& pls) {
    std::vector<Polyline<Inexact>> plsInexact;

    for (const auto& pl : pls.polylines) {
        plsInexact.push_back(approximate(pl));
    }

    return {plsInexact};
}
}
