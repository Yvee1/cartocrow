#ifndef CARTOCROW_INPUT_INSTANCE_H
#define CARTOCROW_INPUT_INSTANCE_H

#include "cat_point.h"

namespace cartocrow::kinetic_kelp {
using VertexId = int;

class InputInstance {
public:
    InputInstance() = default;
    InputInstance(std::vector<CatPoint> catPoints);

    int numCategories() const;
    int size() const;
    const CatPoint& operator[](int i) const;
    const std::vector<CatPoint>& catPoints() const;

    const std::vector<VertexId>& category(int k) const;
private:
    std::vector<CatPoint> m_catPoints;
    std::vector<std::vector<VertexId>> m_cats;
    int m_numCats;
};
}

#endif //CARTOCROW_INPUT_INSTANCE_H
