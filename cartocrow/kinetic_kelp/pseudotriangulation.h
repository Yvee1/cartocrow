#ifndef CARTOCROW_PSEUDOTRIANGULATION_H
#define CARTOCROW_PSEUDOTRIANGULATION_H

#include "input_instance.h"
#include "types.h"
#include "state_geometry.h"

#include "cartocrow/circle_segment_helpers/circle_tangents.h"

namespace cartocrow::kinetic_kelp {
enum TangentType {
	Outer1,
	Outer2,
	Inner1,
	Inner2,
	PointCircle1,
	PointCircle2,
};

struct RationalCircularArc {
	RationalRadiusCircle circle;
	Point<Exact> source;
	Point<Exact> target;
	CGAL::Orientation orientation;
	bool operator==(const RationalCircularArc& other) const = default;
};
}

//namespace std {
//template <>
//struct hash<cartocrow::kinetic_kelp::RationalCircularArc>
//{
//	std::size_t operator()(const cartocrow::kinetic_kelp::RationalCircularArc& ca) const
//	{
//		// Compute individual hash values for first, second and third
//		// http://stackoverflow.com/a/1646913/126995
//		std::size_t res = 17;
//		res = res * 31 + hash<cartocrow::RationalRadiusCircle>{}(ca.circle);
//		res = res * 31 + hash<cartocrow::Point<cartocrow::Exact>>{}(ca.source);
//		res = res * 31 + hash<cartocrow::Point<cartocrow::Exact>>{}(ca.target);
//		return res;
//	}
//};
//}

namespace cartocrow::kinetic_kelp {
bool circlePointLiesOnArc(const Point<Exact>& point, const RationalCircularArc& arc);

class Pseudotriangulation {
  public:
	enum TangentObjectType {
		Circle,
		Elbow,
		CircleStraight1,
		CircleStraight2,
	};
	class TangentObject {
		TangentObjectType type;
		PointId pointId;
		std::optional<ElbowId> elbowId;
		std::optional<StraightId> straightId;
	};
	class Tangent {
		TangentType type;
		TangentObject* source;
		TangentObject* target;
	};
	class TangentPart;
	// probably need Object and ObjectType, and store in ObjectPart
	class ObjectPart {
		TangentPart* next;
		std::variant<StraightId, ElbowId, PointId> object;
	};
	class TangentPart {
		ObjectPart* next;
		Tangent* tangent;
	};
	enum VertexType {
		TangentEndpoint,
		CircleElbow,
		CircleStraight,
	};
	class Vertex {
		VertexType type;
		PointId pointId;
		std::optional<ElbowId> elbowId;
		std::optional<StraightId> straightId;
	};
	class Edge {
		std::vector<ObjectPart> objectParts;
		std::vector<TangentPart> tangentParts;
		std::variant<ObjectPart*, TangentPart*> firstPart;
		Vertex* source;
		Vertex* target;
	};

	std::vector<TangentObject> m_tangentObjects;
	std::vector<Tangent> m_tangents;
	std::vector<Vertex> m_vertices;
	std::vector<Edge> m_edges;
};

class PseudotriangulationGeometry {
	using TangentObjectGeometry = std::variant<Point<Exact>, RationalRadiusCircle, RationalCircularArc>;
	std::unordered_map<Pseudotriangulation::TangentObject, TangentObjectGeometry> m_tangentObject;
	std::unordered_map<Pseudotriangulation::Tangent, RationalTangent> m_tangents;
	std::unordered_map<Pseudotriangulation::Vertex, Point<Exact>> m_vertices;
};

// returns std::pair<Tangent, RationalTangent>
//template <class OutputIterator>
//void tangents(const Object& one, const Object& other, OutputIterator out) {
//	if (auto cp1 = std::get_if<RationalRadiusCircle>(&one->geom)) {
//		auto c1 = *cp1;
//		if (auto cp2 = std::get_if<RationalRadiusCircle>(&other->geom)) {
//			auto c2 = *cp2;
//			auto outer = rationalBitangents(c1, c2, false);
//			auto inner = rationalBitangents(c1, c2, true);
//			if (outer.has_value()) {
//				*out++ = Tangent(outer->first, one, other, Outer1);
//				*out++ = Tangent(outer->second, one, other, Outer2);
//			}
//			if (inner.has_value()) {
//				*out++ = Tangent(inner->first, one, other, Inner1);
//				*out++ = Tangent(inner->second, one, other, Inner2);
//			}
//		} else if (auto cap2 = std::get_if<RationalCircularArc>(&other->geom)) {
//			auto ca2 = *cap2;
//			auto outer = rationalBitangents(c1, ca2.circle, false);
//			auto inner = rationalBitangents(c1, ca2.circle, true);
//			if (outer.has_value()) {
//				auto [t1, t2] = *outer;
//				if (circlePointLiesOnArc(t1.target(), ca2)) {
//					*out++ = Tangent(t1, one, other, Outer1);
//				}
//				if (circlePointLiesOnArc(t2.target(), ca2)) {
//					*out++ = Tangent(t2, one, other, Outer2);
//				}
//			}
//			if (inner.has_value()) {
//				auto [t1, t2] = *inner;
//				if (circlePointLiesOnArc(t1.target(), ca2)) {
//					*out++ = Tangent(t1, one, other, Inner1);
//				}
//				if (circlePointLiesOnArc(t2.target(), ca2)) {
//					*out++ = Tangent(t2, one, other, Inner2);
//				}
//			}
//		} else {
//			auto p2 = std::get<Point<Exact>>(other->geom);
//			auto ts = rationalTangents(p2, c1);
//			if (ts.has_value()) {
//				*out++ = Tangent(ts->first, other, one, PointCircle1);
//				*out++ = Tangent(ts->second, other, one, PointCircle2);
//			}
//		}
//	} else if (auto cap1 = std::get_if<RationalCircularArc>(&one->geom)) {
//		auto ca1 = *cap1;
//		if (auto cap2 = std::get_if<RationalCircularArc>(&other->geom)) {
//			auto ca2 = *cap2;
//			auto outer = rationalBitangents(ca1.circle, ca2.circle, false);
//			auto inner = rationalBitangents(ca1.circle, ca2.circle, true);
//			if (outer.has_value()) {
//				auto [t1, t2] = *outer;
//				if (circlePointLiesOnArc(t1.source(), ca1) && circlePointLiesOnArc(t1.target(), ca2)) {
//					*out++ = Tangent(t1, one, other, Outer1);
//				}
//				if (circlePointLiesOnArc(t2.source(), ca1) && circlePointLiesOnArc(t2.target(), ca2)) {
//					*out++ = Tangent(t2, one, other, Outer2);
//				}
//			}
//			if (inner.has_value()) {
//				auto [t1, t2] = *inner;
//				if (circlePointLiesOnArc(t1.source(), ca1) && circlePointLiesOnArc(t1.target(), ca2)) {
//					*out++ = Tangent(t1, one, other, Inner1);
//				}
//				if (circlePointLiesOnArc(t2.source(), ca1) && circlePointLiesOnArc(t2.target(), ca2)) {
//					*out++ = Tangent(t2, one, other, Inner2);
//				}
//			}
//		} else if (auto p2p = std::get_if<Point<Exact>>(&other->geom)) {
//			auto ts = rationalTangents(ca1.circle, *p2p);
//			if (ts.has_value()) {
//				auto [t1, t2] = *ts;
//				if (circlePointLiesOnArc(t1.source(), ca1)) {
//					*out++ = Tangent(t1, one, other, CirclePoint1);
//				}
//				if (circlePointLiesOnArc(t2.source(), ca1)) {
//					*out++ = Tangent(t2, one, other, CirclePoint2);
//				}
//			}
//		} else {
//			tangents(other, one, out);
//		}
//	} else {
//		auto p1 = std::get<Point<Exact>>(one->geom);
//		if (auto p2p = std::get_if<Point<Exact>>(&other->geom)) {
//			*out++ = Tangent(RationalTangent(Segment<Exact>(p1, *p2p)), one, other, PointPoint);
//		} else {
//			tangents(other, one, out);
//		}
//	}
//}

template <class OutputIterator>
void pseudotriangulationTangents(const StateGeometry& stateGeometry, OutputIterator out) {
}
}

#endif //CARTOCROW_PSEUDOTRIANGULATION_H