#ifndef CARTOCROW_TRAJECTORY_H
#define CARTOCROW_TRAJECTORY_H

#include "../core/core.h"
#include "../core/polyline.h"

namespace cartocrow::kinetic_kelp {
class TimePoint {
public:
    double time;
    Point<Inexact> point;
    TimePoint(double t, Point<Inexact> p) : time(t), point(p) {}
    bool operator==(const TimePoint&) const = default;
    TimePoint transform(CGAL::Aff_transformation_2<Inexact>& t) const { return {time, point.transform(t)}; }
};

class Trajectory {
public:
    std::vector<TimePoint> m_points;

    Trajectory() = default;

    template <class InputIterator>
    Trajectory(InputIterator begin, InputIterator end) : m_points(begin, end) {}
    Point<Inexact> posAtTime(double time) const;
    bool operator==(const Trajectory&) const = default;

    Polyline<Inexact> polyline() const;
    std::pair<double, double> timespan() const;

    Trajectory transform(CGAL::Aff_transformation_2<Inexact>& t) const;
};
}

#endif //CARTOCROW_TRAJECTORY_H
