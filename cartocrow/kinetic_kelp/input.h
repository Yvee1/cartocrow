#ifndef CARTOCROW_INPUT_H
#define CARTOCROW_INPUT_H

#include "moving_cat_point.h"
#include "input_instance.h"

namespace cartocrow::kinetic_kelp {

class Input {
public:
    Input() = default;
    Input(std::vector<MovingCatPoint> catPoints);

    int numCategories() const;
    int size() const;
    std::pair<double, double> timespan() const;

    const MovingCatPoint& operator[](int i) const;
    const std::vector<MovingCatPoint>& movingCatPoints() const;

    const std::vector<PointId>& category(int k) const;
    /// If snap, then every moving point will be in the input instance.
    /// Moving points with no measurements before (after) time will be placed at their initial (final) position.
    InputInstance instance(double time, bool snap) const;
private:
    std::vector<MovingCatPoint> m_movingCatPoints;
    std::vector<std::vector<PointId>> m_cats;
    int m_numCats;
    double m_tStart;
    double m_tEnd;
};
}
#endif //CARTOCROW_INPUT_H
