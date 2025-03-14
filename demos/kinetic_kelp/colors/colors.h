#ifndef CARTOCROW_COLORS_H
#define CARTOCROW_COLORS_H

#include "cartocrow/core/core.h"

namespace cartocrow::CB {
	extern Color light_blue;
	extern Color blue;
    extern Color light_green;
    extern Color green;
    extern Color light_red;
    extern Color red;
    extern Color light_orange;
    extern Color orange;
    extern Color light_purple;
    extern Color purple;
    extern std::vector<Color> lights;
}

namespace cartocrow::tableau {
    extern std::vector<Color> alternatingLightDark;
    extern std::vector<Color> firstLightThenDark;
}

#endif //CARTOCROW_COLORS_H
