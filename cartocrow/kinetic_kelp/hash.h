#ifndef CARTOCROW_HASH_H
#define CARTOCROW_HASH_H

#include "cartocrow/circle_segment_helpers/circle_tangents.h"

namespace std {
template <>
struct hash<cartocrow::Number<cartocrow::Exact>>
{
    std::size_t operator()(const cartocrow::Number<cartocrow::Exact>& x) const
    {
        return hash<double>{}(CGAL::to_double(x));
    }
};

template <>
struct hash<cartocrow::Point<cartocrow::Exact>>
{
    std::size_t operator()(const cartocrow::Point<cartocrow::Exact>& p) const
    {
        // Compute individual hash values for first, second and third
        // http://stackoverflow.com/a/1646913/126995
        std::size_t res = 17;
        res = res * 31 + hash<cartocrow::Number<cartocrow::Exact>>{}(p.x());
        res = res * 31 + hash<cartocrow::Number<cartocrow::Exact>>{}(p.y());
        return res;
    }
};

template <>
struct hash<cartocrow::RationalRadiusCircle>
{
    std::size_t operator()(const cartocrow::RationalRadiusCircle& c) const
    {
        // Compute individual hash values for first, second and third
        // http://stackoverflow.com/a/1646913/126995
        std::size_t res = 17;
        res = res * 31 + hash<cartocrow::Point<cartocrow::Exact>>{}(c.center);
        res = res * 31 + hash<cartocrow::Number<cartocrow::Exact>>{}(c.radius);
        return res;
    }
};
}

#endif //CARTOCROW_HASH_H
