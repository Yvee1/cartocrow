#ifndef CARTOCROW_ROUTE_EDGES_H
#define CARTOCROW_ROUTE_EDGES_H

#include <utility>

#include "input_instance.h"
#include "state.h"
#include "cartocrow/renderer/geometry_renderer.h"
#include "cartocrow/circle_segment_helpers/circle_tangents.h"

namespace cartocrow::kinetic_kelp {
class RoutingObject;

enum TangentType {
    Outer1,
    Outer2,
    Inner1,
    Inner2,
    PointCircle1,
    PointCircle2,
    CirclePoint1,
    CirclePoint2,
    PointPoint,
};

class RoutingTangent {
  public:
    RoutingTangent(RationalTangent geom, std::shared_ptr<RoutingObject> source, std::shared_ptr<RoutingObject> target, TangentType type) :
        geom(std::move(geom)), source(std::move(source)), target(std::move(target)), type(type) {}
    RationalTangent geom;
    std::shared_ptr<RoutingObject> source;
    std::shared_ptr<RoutingObject> target;
    TangentType type;
};

class RoutingObject {
  public:
    RoutingObject(std::variant<RationalRadiusCircle, Point<Exact>> geom, VertexId vertex) :
        geom(std::move(geom)), vertex(vertex) {};
    bool operator==(const RoutingObject& other) const = default;
    std::variant<RationalRadiusCircle, Point<Exact>> geom;
    VertexId vertex;
};

class RoutingVertex {
  public:
    RoutingVertex() = default;
    RoutingVertex(std::shared_ptr<RoutingObject> object, Point<Exact> point) : object(std::move(object)), point(std::move(point)) {};
    std::shared_ptr<RoutingObject> object;
    Point<Exact> point;
};

class RoutingEdge {
  public:
    RoutingEdge() = default;
    RoutingEdge(std::variant<RationalTangent, CSCurve> geom, double weight) : geom(std::move(geom)), weight(weight) {};
    std::variant<RationalTangent, CSCurve> geom;
    double weight;
};

struct RoutingPath {
    std::vector<long unsigned int> path;
    double weight;
};

template <class OutputIterator>
void tangents(const std::shared_ptr<RoutingObject> one, const std::shared_ptr<RoutingObject>& other, OutputIterator out) {
    if (auto cp1 = std::get_if<RationalRadiusCircle>(&one->geom)) {
        auto c1 = *cp1;
        if (auto cp2 = std::get_if<RationalRadiusCircle>(&other->geom)) {
            auto c2 = *cp2;
            auto outer = rationalBitangents(c1, c2, false);
            auto inner = rationalBitangents(c1, c2, true);
            if (outer.has_value()) {
                *out++ = RoutingTangent(outer->first, one, other, Outer1);
                *out++ = RoutingTangent(outer->second, one, other, Outer2);
            }
            if (inner.has_value()) {
                *out++ = RoutingTangent(inner->first, one, other, Inner1);
                *out++ = RoutingTangent(inner->second, one, other, Inner2);
            }
        } else {
            auto p2 = std::get<Point<Exact>>(other->geom);
            auto ts = rationalTangents(p2, c1);
            if (ts.has_value()) {
                *out++ = RoutingTangent(ts->first, other, one, PointCircle1);
                *out++ = RoutingTangent(ts->second, other, one, PointCircle2);
            }
        }
    } else {
        auto p1 = std::get<Point<Exact>>(one->geom);
        if (auto p2p = std::get_if<RationalRadiusCircle>(&other->geom)) {
            tangents(other, one, out);
        }
    }
}

State routeEdges(const InputInstance& input, const Settings& settings, renderer::GeometryRenderer& renderer);
}

#endif //CARTOCROW_ROUTE_EDGES_H
