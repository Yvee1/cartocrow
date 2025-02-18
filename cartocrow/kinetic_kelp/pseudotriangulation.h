#ifndef CARTOCROW_PSEUDOTRIANGULATION_H
#define CARTOCROW_PSEUDOTRIANGULATION_H

#include "input_instance.h"
#include "types.h"
#include "state_geometry.h"
#include "hash.h"

#include "cartocrow/circle_segment_helpers/circle_tangents.h"
#include "cartocrow/circle_segment_helpers/cs_render_helpers.h"
#include "cartocrow/renderer/geometry_renderer.h"

#include <CGAL/Boolean_set_operations_2.h>

namespace cartocrow::kinetic_kelp {
enum TangentType {
	Outer1,
	Outer2,
	Inner1,
	Inner2,
	PointCircle1,
	PointCircle2,
    PointPoint,
};

struct RationalCircularArc {
	RationalRadiusCircle circle;
	Point<Exact> source;
	Point<Exact> target;
	CGAL::Orientation orientation;
	bool operator==(const RationalCircularArc& other) const = default;
};
}

namespace std {
template <>
struct hash<cartocrow::kinetic_kelp::RationalCircularArc>
{
	std::size_t operator()(const cartocrow::kinetic_kelp::RationalCircularArc& ca) const
	{
		// Compute individual hash values for first, second and third
		// http://stackoverflow.com/a/1646913/126995
		std::size_t res = 17;
		res = res * 31 + hash<cartocrow::RationalRadiusCircle>{}(ca.circle);
		res = res * 31 + hash<cartocrow::Point<cartocrow::Exact>>{}(ca.source);
		res = res * 31 + hash<cartocrow::Point<cartocrow::Exact>>{}(ca.target);
		return res;
	}
};
}

namespace cartocrow::kinetic_kelp {
bool circlePointLiesOnArc(const Point<Exact> &point, const RationalCircularArc &arc);

class Pseudotriangulation {
public:
    enum TangentObjectType {
        Circle,
//        Elbow,
        CircleStraight1,
        CircleStraight2,
        CircleCircle1,
        CircleCircle2,
    };
    struct TangentObject {
        TangentObjectType type;
        PointId pointId;
//        std::optional<ElbowId> elbowId;
        std::optional<StraightId> straightId;

        bool circleStraight() { return type == CircleStraight1 || type == CircleStraight2; }

        TangentObject(PointId pointId) : pointId(pointId), type(Circle), straightId(std::nullopt) {};
        TangentObject(PointId pointId, StraightId straightId, bool one) : pointId(pointId), straightId(straightId), type(one ? CircleStraight1 : CircleStraight2) {};

        bool operator==(const TangentObject& other) const = default;
    };
    struct Tangent {
        TangentType type;
        std::shared_ptr<TangentObject> source;
        std::shared_ptr<TangentObject> target;

        bool operator==(const Tangent& other) const = default;
    };
    struct TangentPart;
    // probably need Object and ObjectType, and store in ObjectPart
    struct ObjectPart {
        std::shared_ptr<TangentPart> next;
        std::variant<StraightId, ElbowId, PointId> object;
    };
    struct TangentPart {
        std::shared_ptr<ObjectPart> next;
        std::shared_ptr<Tangent> tangent;
    };
    enum VertexType {
        TangentEndpoint,
        CircleElbow,
        CircleStraight,
        CircleCircle,
    };
    struct Vertex {
        VertexType type;
        PointId pointId;
        std::optional<ElbowId> elbowId;
        std::optional<StraightId> straightId;
        bool operator==(const Vertex& other) const = default;
    };
    struct Edge {
        std::vector<ObjectPart> objectParts;
        std::vector<TangentPart> tangentParts;
        std::variant<std::shared_ptr<ObjectPart>, std::shared_ptr<TangentPart>> firstPart;
        std::shared_ptr<Vertex> source;
        std::shared_ptr<Vertex> target;
        bool operator==(const Edge& other) const = default;
    };

    std::vector<std::shared_ptr<TangentObject>> m_tangentObjects;
    std::vector<std::shared_ptr<Tangent>> m_tangents;
    std::vector<std::shared_ptr<Vertex>> m_vertices;
    std::vector<std::shared_ptr<Edge>> m_edges;
};
}

namespace std {
template <>
struct hash<cartocrow::kinetic_kelp::Pseudotriangulation::TangentObject>
{
    std::size_t operator()(const cartocrow::kinetic_kelp::Pseudotriangulation::TangentObject& to) const
    {
        // Compute individual hash values for member variables
        // http://stackoverflow.com/a/1646913/126995
        std::size_t res = 17;
        res = res * 31 + hash<int>{}(static_cast<int>(to.type));
        res = res * 31 + hash<int>{}(to.pointId);
        res = res * 31 + hash<std::optional<cartocrow::kinetic_kelp::StraightId>>{}(to.straightId);
        return res;
    }
};

template <>
struct hash<cartocrow::kinetic_kelp::Pseudotriangulation::Tangent>
{
    std::size_t operator()(const cartocrow::kinetic_kelp::Pseudotriangulation::Tangent& t) const
    {
        // Compute individual hash values for member variables
        // http://stackoverflow.com/a/1646913/126995
        std::size_t res = 17;
        res = res * 31 + hash<int>{}(static_cast<int>(t.type));
        res = res * 31 + hash<std::shared_ptr<cartocrow::kinetic_kelp::Pseudotriangulation::TangentObject>>{}(t.source);
        res = res * 31 + hash<std::shared_ptr<cartocrow::kinetic_kelp::Pseudotriangulation::TangentObject>>{}(t.target);
        return res;
    }
};


template <>
struct hash<cartocrow::kinetic_kelp::Pseudotriangulation::Vertex>
{
    std::size_t operator()(const cartocrow::kinetic_kelp::Pseudotriangulation::Vertex& v) const
    {
        // Compute individual hash values for member variables
        // http://stackoverflow.com/a/1646913/126995
        std::size_t res = 17;
        res = res * 31 + hash<int>{}(static_cast<int>(v.type));
        res = res * 31 + hash<int>{}(v.pointId);
        res = res * 31 + hash<std::optional<cartocrow::kinetic_kelp::ElbowId>>{}(v.elbowId);
        res = res * 31 + hash<std::optional<cartocrow::kinetic_kelp::StraightId>>{}(v.straightId);
        return res;
    }
};
}

namespace cartocrow::kinetic_kelp {
template <class OutputIterator>
void intersectionPoints(const CSPolygon& one, const CSPolygon& other, OutputIterator out) {
    using Intersection = boost::variant<std::pair<OneRootPoint, ArrCSTraits::Multiplicity>, CSXMCurve>;
    std::vector<Intersection> intersections;

    ArrCSTraits traits;
    auto intersect = traits.intersect_2_object();
    for (auto cit1 = one.curves_begin(); cit1 != one.curves_end(); ++cit1) {
        auto& c1 = *cit1;
        for (auto cit2 = other.curves_begin(); cit2 != other.curves_end(); ++cit2) {
            auto& c2 = *cit2;
            intersect(c1, c2, std::back_inserter(intersections));
        }
    }

    for (const auto& inter : intersections) {
        if (inter.type() == typeid(std::pair<OneRootPoint, ArrCSTraits::Multiplicity>)) {
            auto pt = boost::get<std::pair<OneRootPoint, ArrCSTraits::Multiplicity>>(inter).first;
            *out++ = pt;
        }
    }
}

bool liesOnHalf(const OneRootPoint& pt, const Straight& s, bool firstHalf);

bool doIntersect(const RationalTangent& rt1, const RationalTangent& rt2);

class PseudotriangulationGeometry {
  public:
    using TangentObject = Pseudotriangulation::TangentObject;
    using Tangent = Pseudotriangulation::Tangent;
    using Vertex = Pseudotriangulation::Vertex;
	using TangentObjectGeometry = std::variant<Point<Exact>, RationalRadiusCircle>;

	std::unordered_map<TangentObject, TangentObjectGeometry> m_tangentObject;
	std::unordered_map<Tangent, RationalTangent> m_tangents;
	std::unordered_map<Vertex, Point<Exact>> m_vertices;

    using TangentObjectWG = std::pair<std::shared_ptr<TangentObject>, TangentObjectGeometry>;

    // returns std::pair<Tangent, RationalTangent>
    template <class OutputIterator>
    static void tangents(TangentObjectWG one, TangentObjectWG other, OutputIterator out) {
        if (auto cp1 = std::get_if<RationalRadiusCircle>(&one.second)) {
            auto c1 = *cp1;
            if (auto cp2 = std::get_if<RationalRadiusCircle>(&other.second)) {
                auto c2 = *cp2;
                auto outer = rationalBitangents(c1, c2, false);
                auto inner = rationalBitangents(c1, c2, true);
                if (outer.has_value()) {
                    *out++ = std::pair(Tangent(Outer1, one.first, other.first), outer->first);
                    *out++ = std::pair(Tangent(Outer2, one.first, other.first), outer->second);
                }
                if (inner.has_value()) {
                    *out++ = std::pair(Tangent(Inner1, one.first, other.first), inner->first);
                    *out++ = std::pair(Tangent(Inner2, one.first, other.first), inner->second);
                }
//            } else if (auto cap2 = std::get_if<RationalCircularArc>(&other.second)) {
//                auto ca2 = *cap2;
//                auto outer = rationalBitangents(c1, ca2.circle, false);
//                auto inner = rationalBitangents(c1, ca2.circle, true);
//                if (outer.has_value()) {
//                    auto [t1, t2] = *outer;
//                    if (circlePointLiesOnArc(t1.target(), ca2)) {
//                        *out++ = std::pair(Tangent(Outer1, one.first, other.first), t1);
//                    }
//                    if (circlePointLiesOnArc(t2.target(), ca2)) {
//                        *out++ = std::pair(Tangent(Outer2, one.first, other.first), t2);
//                    }
//                }
//                if (inner.has_value()) {
//                    auto [t1, t2] = *inner;
//                    if (circlePointLiesOnArc(t1.target(), ca2)) {
//                        *out++ = std::pair(Tangent(Inner1, one.first, other.first), t1);
//                    }
//                    if (circlePointLiesOnArc(t2.target(), ca2)) {
//                        *out++ = std::pair(Tangent(Inner2, one.first, other.first), t2);
//                    }
//                }
            } else {
                auto p2 = std::get<Point<Exact>>(other.second);
                auto ts = rationalTangents(p2, c1);
                if (ts.has_value()) {
                    *out++ = std::pair(Tangent(PointCircle1, other.first, one.first), ts->first);
                    *out++ = std::pair(Tangent(PointCircle2, other.first, one.first), ts->second);
                }
            }
//        } else if (auto cap1 = std::get_if<RationalCircularArc>(&one.second)) {
//            auto ca1 = *cap1;
//            if (auto cap2 = std::get_if<RationalCircularArc>(&other.second)) {
//                auto ca2 = *cap2;
//                auto outer = rationalBitangents(ca1.circle, ca2.circle, false);
//                auto inner = rationalBitangents(ca1.circle, ca2.circle, true);
//                if (outer.has_value()) {
//                    auto [t1, t2] = *outer;
//                    if (circlePointLiesOnArc(t1.source(), ca1) && circlePointLiesOnArc(t1.target(), ca2)) {
//                        *out++ = std::pair(Tangent(Outer1, one.first, other.first), t1);
//                    }
//                    if (circlePointLiesOnArc(t2.source(), ca1) && circlePointLiesOnArc(t2.target(), ca2)) {
//                        *out++ = std::pair(Tangent(Outer2, one.first, other.first), t2);
//                    }
//                }
//                if (inner.has_value()) {
//                    auto [t1, t2] = *inner;
//                    if (circlePointLiesOnArc(t1.source(), ca1) && circlePointLiesOnArc(t1.target(), ca2)) {
//                        *out++ = std::pair(Tangent(Inner1, one.first, other.first), t1);
//                    }
//                    if (circlePointLiesOnArc(t2.source(), ca1) && circlePointLiesOnArc(t2.target(), ca2)) {
//                        *out++ = std::pair(Tangent(Inner2, one.first, other.first), t2);
//                    }
//                }
//            } else if (auto p2p = std::get_if<Point<Exact>>(&other.second)) {
//                auto ts = rationalTangents(*p2p, ca1.circle);
//                if (ts.has_value()) {
//                    auto [t1, t2] = *ts;
//                    if (circlePointLiesOnArc(t1.target(), ca1)) {
//                        *out++ = std::pair(Tangent(PointCircle1, other.first, one.first), t1);
//                    }
//                    if (circlePointLiesOnArc(t2.target(), ca1)) {
//                        *out++ = std::pair(Tangent(PointCircle2, other.first, one.first), t2);
//                    }
//                }
//            } else {
//                auto c2 = std::get<RationalRadiusCircle>(other.second);
//                auto outer = rationalBitangents(ca1.circle, c2, false);
//                auto inner = rationalBitangents(ca1.circle, c2, true);
//                if (outer.has_value()) {
//                    auto [t1, t2] = *outer;
//                    if (circlePointLiesOnArc(t1.source(), ca1)) {
//                        *out++ = std::pair(Tangent(Outer1, one.first, other.first), t1);
//                    }
//                    if (circlePointLiesOnArc(t2.source(), ca1)) {
//                        *out++ = std::pair(Tangent(Outer2, one.first, other.first), t2);
//                    }
//                }
//                if (inner.has_value()) {
//                    auto [t1, t2] = *inner;
//                    if (circlePointLiesOnArc(t1.source(), ca1)) {
//                        *out++ = std::pair(Tangent(Inner1, one.first, other.first), t1);
//                    }
//                    if (circlePointLiesOnArc(t2.source(), ca1)) {
//                        *out++ = std::pair(Tangent(Inner2, one.first, other.first), t2);
//                    }
//                }
//            }
        } else {
            auto p1 = std::get<Point<Exact>>(one.second);
            if (auto p2p = std::get_if<Point<Exact>>(&other.second)) {
                *out++ = std::pair(Tangent(PointPoint, one.first, other.first), RationalTangent(Segment<Exact>(p1, *p2p)));
            } else if (auto c2p = std::get_if<RationalRadiusCircle>(&other.second)) {
                auto c2 = *c2p;
                auto ts = rationalTangents(p1, c2);
                if (ts.has_value()) {
                    *out++ = std::pair(Tangent(PointCircle1, one.first, other.first), ts->first);
                    *out++ = std::pair(Tangent(PointCircle2, one.first, other.first), ts->second);
                }
            } else {
                throw std::runtime_error("Impossible");
            }
//                auto ca2 = std::get<RationalCircularArc>(other.second);
//                auto ts = rationalTangents(p1, ca2.circle);
//                if (ts.has_value()) {
//                    auto [t1, t2] = *ts;
//                    if (circlePointLiesOnArc(t1.target(), ca2)) {
//                        *out++ = std::pair(Tangent(PointCircle1, other.first, one.first), t1);
//                    }
//                    if (circlePointLiesOnArc(t2.target(), ca2)) {
//                        *out++ = std::pair(Tangent(PointCircle2, other.first, one.first), t2);
//                    }
//                }
//            }
        }
    }

    static bool free(const RationalTangent& rt, const CSPolygon& obstacle);
    static bool free(const RationalTangent& rt, const CSPolygonWithHoles& obstacle);

    static std::pair<Pseudotriangulation, PseudotriangulationGeometry> pseudotriangulationTangents(const State& state, const StateGeometry& stateGeometry);
};
}

#endif //CARTOCROW_PSEUDOTRIANGULATION_H