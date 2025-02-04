#include "trajectory.h"

namespace cartocrow::kinetic_kelp {
Point<Inexact> Trajectory::posAtTime(double time) const {
    auto firstPast = m_points.end();

    for (auto it = m_points.begin(); it != m_points.end(); ++it) {
        auto& [t, point] = *it;
        if (t == time) {
            return it->point;
        }
        if (t > time) {
            firstPast = it;
            break;
        }
    }

    if (firstPast == m_points.begin()) {
//        std::cerr << "Trajectory starts after requested time " << time << std::endl;
        return firstPast->point;
    } else if (firstPast == m_points.end()) {
//        std::cerr << "Trajectory ends before requested time " << time << std::endl;
        return (--firstPast)->point;
    }

    auto firstBefore = firstPast;
    --firstBefore;
    double ratio = 1.0 - (time - firstBefore->time) / (firstPast->time - firstBefore->time);
    const Point<Inexact>& p1 = firstBefore->point;
    const Point<Inexact>& p2 = firstPast->point;

    return {ratio * p1.x() + (1-ratio) * p2.x(), ratio * p1.y() + (1-ratio) * p2.y()};
}

Polyline<Inexact> Trajectory::polyline() const {
    std::vector<Point<Inexact>> points;
    for (const auto& [_, p] : m_points) {
        points.push_back(p);
    }
    return {points.begin(), points.end()};
}

std::pair<double, double> Trajectory::timespan() const {
    return {m_points.front().time, m_points.back().time};
}
}