#ifndef CARTOCROW_PARSE_INPUT_H
#define CARTOCROW_PARSE_INPUT_H

#include "cat_point.h"
#include "moving_cat_point.h"
#include <filesystem>

namespace cartocrow::kinetic_kelp {
std::vector<CatPoint> parseCatPoints(const std::string& s);
std::vector<MovingCatPoint> parseMovingPoints(const std::filesystem::path& filePath, double secondsBetweenVertices = 1.0);
}

#endif //CARTOCROW_PARSE_INPUT_H
