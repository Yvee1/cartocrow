#ifndef CARTOCROW_DRAW_SETTINGS_H
#define CARTOCROW_DRAW_SETTINGS_H

#include "cartocrow/core/core.h"

namespace cartocrow::kinetic_kelp {
struct DrawSettings {
    double markRadius = 1.0;
    double strokeWidth = 1.0;
    double smoothing = 0.2;
    std::vector<Color> colors;

    Color getColor(int category) const {
        Color fillColor;
        if (category >= colors.size() || category < 0) {
            std::cerr << "Warning! No color specified for category " << category << std::endl;
            fillColor = Color{240, 240, 240};
        } else {
            fillColor = colors[category];
        }
        return fillColor;
    }
};
}

#endif //CARTOCROW_DRAW_SETTINGS_H
