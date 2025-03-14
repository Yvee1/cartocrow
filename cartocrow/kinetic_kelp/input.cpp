#include "input.h"

namespace cartocrow::kinetic_kelp {
Input::Input(std::vector<MovingCatPoint> movingCatPoints): m_movingCatPoints(std::move(movingCatPoints)) {
    m_numCats = 0;

    m_tStart = std::numeric_limits<double>::infinity();
    m_tEnd = - std::numeric_limits<double>::infinity();
    for (int i = 0; i < m_movingCatPoints.size(); ++i) {
        auto& cp = m_movingCatPoints[i];
        auto [tStart, tEnd] = cp.trajectory.timespan();
        if (tStart < m_tStart) {
            m_tStart = tStart;
        }
        if (tEnd > m_tEnd) {
            m_tEnd = tEnd;
        }
        if (cp.category >= m_numCats) {
            m_numCats = cp.category + 1;
        }
        while (m_cats.size() < m_numCats) {
            m_cats.emplace_back();
        }
        m_cats[cp.category].push_back(i);
    }
}

int Input::numCategories() const {
    return m_numCats;
}

int Input::size() const {
    return m_movingCatPoints.size();
}

std::pair<double, double> Input::timespan() const {
    return {m_tStart, m_tEnd};
}

const std::vector<MovingCatPoint>& Input::movingCatPoints() const {
    return m_movingCatPoints;
}

const MovingCatPoint& Input::operator[](int i) const {
    return m_movingCatPoints.at(i);
}

const std::vector<PointId>& Input::category(int k) const {
    return m_cats[k];
}

InputInstance Input::instance(double time, bool snap) const {
	std::vector<CatPoint> catPoints;
	for (const auto& mcp : m_movingCatPoints) {
        auto [tStart, tEnd] = mcp.trajectory.timespan();
        if (snap || tStart <= time && time <= tEnd) {
            catPoints.emplace_back(mcp.category, pretendExact(mcp.trajectory.posAtTime(time)));
        }
	}
	return {catPoints};
}
}