#ifndef CARTOCROW_DRAW_SETTINGS_H
#define CARTOCROW_DRAW_SETTINGS_H

#include "cartocrow/core/core.h"

namespace cartocrow::kinetic_kelp {
struct DrawSettings {
    double markRadius = 1.0;
    double strokeWidth = 1.0;
    double smoothing = 0.2;
    std::vector<Color> colors;
};
}

#endif //CARTOCROW_DRAW_SETTINGS_H
