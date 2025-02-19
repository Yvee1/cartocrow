#ifndef CARTOCROW_SETTINGS_H
#define CARTOCROW_SETTINGS_H

#include "cartocrow/core/core.h"

namespace cartocrow::kinetic_kelp {
struct Settings {
    Number<Exact> vertexRadius = 1.0;
    Number<Exact> edgeWidth = 0.3;
};
}

#endif //CARTOCROW_SETTINGS_H
