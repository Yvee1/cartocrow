#include "input_instance.h"

namespace cartocrow::kinetic_kelp {
InputInstance::InputInstance(std::vector<CatPoint> catPoints): m_catPoints(std::move(catPoints)) {
    m_numCats = 0;
    for (int i = 0; i < m_catPoints.size(); ++i) {
        auto& cp = m_catPoints[i];
        if (cp.category >= m_numCats) {
            m_numCats = cp.category + 1;
        }
        while (m_cats.size() < m_numCats) {
            m_cats.emplace_back();
        }
        m_cats[cp.category].push_back(i);
    }
}

int InputInstance::numCategories() const {
    return m_numCats;
}

int InputInstance::size() const {
    return m_catPoints.size();
}

const std::vector<CatPoint>& InputInstance::catPoints() const {
    return m_catPoints;
}

const CatPoint& InputInstance::operator[](int i) const {
    return m_catPoints.at(i);
}

const std::vector<int>& InputInstance::category(int k) const {
    return m_cats[k];
}
}