#ifndef CARTOCROW_PSEUDOTRIANGULATION_H
#define CARTOCROW_PSEUDOTRIANGULATION_H

#include "input_instance.h"
#include "types.h"
#include "state_geometry.h"
#include "hash.h"
#include "input.h"

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

class Pseudotriangulation;

struct Certificate {
	virtual bool valid(Pseudotriangulation& pt, const State& state, const InputInstance& input, const Settings& settings) = 0;
};

class PseudotriangulationGeometry;

class Pseudotriangulation {
public:
    enum TangentObjectType {
        Circle,
        CircleStraight1, // intersection with right part of straight, viewed from this object.
        CircleStraight2, // intersection with left part of straight, viewed from this object.
        CircleCircle1,
        CircleCircle2,
    };
    struct TangentObject {
        TangentObjectType type;
        PointId pointId;
        std::optional<StraightId> straightId;

        bool circleStraight() { return type == CircleStraight1 || type == CircleStraight2; }
		bool circleTangent() { return type == Circle; }

        TangentObject(PointId pointId) : pointId(pointId), type(Circle), straightId(std::nullopt) {};
        TangentObject(PointId pointId, StraightId straightId, bool one) : pointId(pointId), straightId(straightId), type(one ? CircleStraight1 : CircleStraight2) {};
		TangentObject(const TangentObject& obj, const State& oldState, const State& newState) {
			type = obj.type;
			pointId = obj.pointId;
			if (!obj.straightId.has_value()) {
				straightId = std::nullopt;
			} else {
				auto [edge, oldOrbitIt] = *obj.straightId;
				auto& oldOrbits = oldState.edgeTopology.at(edge).orbits;
				auto& newOrbits = newState.edgeTopology.at(edge).orbits;
				if (oldOrbits.end() == oldOrbitIt) {
					std::cout << "!!!" << std::endl;
				}
				auto i = std::distance(oldOrbits.begin(), oldOrbitIt);
				auto newOrbitIt = std::next(newOrbits.begin(), i);
				straightId = {edge, newOrbitIt};
			}
		}

        bool operator==(const TangentObject& other) const = default;
        std::optional<Orbit> elbowPoint(const State& state) {
            if (!straightId.has_value()) return std::nullopt;
            auto [edge, orbitIt] = *straightId;
            if (pointId == edge.first || pointId == edge.second) return std::nullopt;
            auto [sId, tId] = state.straightEndpoints(*straightId);
            if (pointId == sId) {
                --orbitIt;
            }
            auto orbit = *orbitIt;
            if (orbit.dir == CGAL::COUNTERCLOCKWISE) {
                if (pointId == tId) {
                    if (type == CircleStraight2) {
                        return orbit;
                    }
                } else {
                    if (type == CircleStraight1) {
                        return orbit;
                    }
                }
            } else {
                if (pointId == tId) {
                    if (type == CircleStraight1) {
                        return orbit;
                    }
                } else {
                    if (type == CircleStraight2) {
                        return orbit;
                    }
                }
            }
            return std::nullopt;
        }
    };
    struct Tangent {
        TangentType type;
        std::shared_ptr<TangentObject> source;
        std::shared_ptr<TangentObject> target;

		std::shared_ptr<TangentObject> endpoint(PointId pointId) {
			if (source->pointId == pointId) {
				return source;
			}
			if (target->pointId == pointId) {
				return target;
			}
			throw std::invalid_argument("The provided pointId is not an endpoint of the tangent.");
		}

		std::shared_ptr<TangentObject> otherEndpoint(PointId pointId) {
			if (source->pointId == pointId) {
				return target;
			}
			if (target->pointId == pointId) {
				return source;
			}
			throw std::invalid_argument("The provided pointId is not an endpoint of the tangent.");
		}

		std::shared_ptr<TangentObject> endpoint(bool target) {
			return target ? this->target : this->source;
		}

		std::shared_ptr<TangentObject> otherEndpoint(bool target) {
			return target ? this->source : this->target;
		}

		bool circleBitangent() const {
			return type == Outer1 || type == Outer2 || type == Inner1 || type == Inner2;
		}

        bool operator==(const Tangent& other) const = default;
    };

    std::vector<std::shared_ptr<TangentObject>> m_tangentObjects;
    std::vector<std::shared_ptr<Tangent>> m_tangents;
	std::vector<std::list<std::shared_ptr<Tangent>>> m_pointIdToTangents; // sorted on angle

    typedef CGAL::Circulator_from_container<std::list<std::shared_ptr<Tangent>>> TangentCirculator;

    /// Certifies that consecutive tangents t1 and t2 are in the correct order on pointId.
	struct TangentEndpointCertificate : public Certificate {
		PointId pointId;
		std::shared_ptr<Tangent> t1;
		std::shared_ptr<Tangent> t2;

		bool operator==(const TangentEndpointCertificate& other) {
			return pointId == other.pointId && *t1 == *other.t1 && *t2 == *other.t2;
		}

		bool t1SubsetOft2;

        void setPseudotriangulation(Pseudotriangulation* pseudotriangulationPointer);
		bool valid(Pseudotriangulation& pt, const State& state, const InputInstance& input, const Settings& settings) override;
		bool valid(Pseudotriangulation& pt, const State& state, const PseudotriangulationGeometry& ptg);
		bool valid(Pseudotriangulation& pt, const State& state, const RationalTangent& t1G, const RationalTangent& t2G);

		TangentEndpointCertificate(PointId pointId, std::shared_ptr<Tangent> t1, std::shared_ptr<Tangent> t2)
		    : pointId(pointId), t1(t1), t2(t2) {};
	};

	std::vector<TangentEndpointCertificate> m_tangentEndpointCertificates;
	void fix(TangentEndpointCertificate& certificate, State& state, const Settings& settings);
	void removeTangent(std::shared_ptr<Tangent> t);
	void maybeAddCertificate(PointId pId, const std::shared_ptr<Tangent>& t1, const std::shared_ptr<Tangent>& t2, const State& state);
    std::pair<std::shared_ptr<Pseudotriangulation::Tangent>, std::shared_ptr<Pseudotriangulation::Tangent>> neighbouringTangents(PointId pId, const std::shared_ptr<Tangent>& t);
    std::shared_ptr<Pseudotriangulation::Tangent> previousTangent(PointId pId, const std::shared_ptr<Tangent>& t);
    std::shared_ptr<Pseudotriangulation::Tangent> nextTangent(PointId pId, const std::shared_ptr<Tangent>& t);
    std::optional<std::shared_ptr<Tangent>> edgeOfStraight(const std::shared_ptr<Tangent>& t);
	TangentCirculator tangentCirculator(PointId pId, const std::shared_ptr<Tangent>& t);
	std::shared_ptr<Pseudotriangulation::TangentObject> circleTangentObject(PointId pointId) const;

    CGAL::Orientation sourceOrientation(const std::shared_ptr<Tangent>& t, const State& state) {
        if (t->source->elbowPoint(state).has_value() && edgeOfStraight(t).has_value()) {
            if (t->source->type == CircleStraight2) {
                return CGAL::CLOCKWISE;
            } else {
                assert(t->source->type == CircleStraight1);
                return CGAL::COUNTERCLOCKWISE;
            }
        }

        // todo incorrect
        if (t->type == Outer1 || t->type == Inner1) {
            return CGAL::COUNTERCLOCKWISE;
        }
        if (t->type == Outer2 || t->type == Inner2) {
            return CGAL::CLOCKWISE;
        }
        return CGAL::COLLINEAR; /// todo
        throw std::invalid_argument("Unimplemented enum type");
    }

    CGAL::Orientation targetOrientation(const std::shared_ptr<Tangent>& t, const State& state) {
        if (t->target->elbowPoint(state).has_value() && edgeOfStraight(t).has_value()) {
            if (t->target->type == CircleStraight2) {
                return CGAL::CLOCKWISE;
            } else {
                assert(t->target->type == CircleStraight1);
                return CGAL::COUNTERCLOCKWISE;
            }
        }

        if (t->type == Outer1 || t->type == Inner2 || t->type == PointCircle1) {
            return CGAL::CLOCKWISE;
        }
        if (t->type == Outer2 || t->type == Inner1 || t->type == PointCircle2) {
            return CGAL::COUNTERCLOCKWISE;
        }
        return CGAL::COLLINEAR;
        throw std::invalid_argument("Unimplemented enum type");
    }

    CGAL::Orientation orientation(const std::shared_ptr<Tangent>& t, bool target, const State& state) {
        return target ? targetOrientation(t, state) : sourceOrientation(t, state);
    }

    CGAL::Orientation orientation(const std::shared_ptr<Tangent>& t, PointId pId, const State& state) {
        return t->source->pointId == pId ? sourceOrientation(t, state) : targetOrientation(t, state);
    }

	Pseudotriangulation() = default;

	Pseudotriangulation(const Pseudotriangulation& pt, const State& oldState, const State& newState) {
		struct HashTangentObject {
			std::size_t operator()(const cartocrow::kinetic_kelp::Pseudotriangulation::TangentObject& to) const
			{
				// Compute individual hash values for member variables
				// http://stackoverflow.com/a/1646913/126995
				std::size_t res = 17;
				res = res * 31 + std::hash<int>{}(static_cast<int>(to.type));
				res = res * 31 + std::hash<int>{}(to.pointId);
				//        res = res * 31 + hash<std::optional<cartocrow::kinetic_kelp::StraightId>>{}(to.straightId);
				return res;
			}
		};

		struct HashTangent {
			std::size_t operator()(const cartocrow::kinetic_kelp::Pseudotriangulation::Tangent& t) const
			{
				// Compute individual hash values for member variables
				// http://stackoverflow.com/a/1646913/126995
				std::size_t res = 17;
				res = res * 31 + std::hash<int>{}(static_cast<int>(t.type));
				res = res * 31 + std::hash<std::shared_ptr<cartocrow::kinetic_kelp::Pseudotriangulation::TangentObject>>{}(t.source);
				res = res * 31 + std::hash<std::shared_ptr<cartocrow::kinetic_kelp::Pseudotriangulation::TangentObject>>{}(t.target);
				return res;
			};
		};

		std::cout << "Copying" << std::endl;
		std::unordered_map<TangentObject, std::shared_ptr<TangentObject>, HashTangentObject> objMap;
		for (const auto& obj : pt.m_tangentObjects) {
			m_tangentObjects.push_back(std::make_shared<TangentObject>(*obj, oldState, newState));
			objMap[*obj] = m_tangentObjects.back();
		}
		std::unordered_map<Tangent, std::shared_ptr<Tangent>, HashTangent> tMap;
		for (const auto& t : pt.m_tangents) {
			auto sObj = objMap[*t->source];
			auto tObj = objMap[*t->target];
			m_tangents.push_back(std::make_shared<Tangent>(t->type, sObj, tObj));
			tMap[*t] = m_tangents.back();
		}
		for (const auto& ts : pt.m_pointIdToTangents) {
			auto& newTs = m_pointIdToTangents.emplace_back();
			for (const auto& t : ts) {
				newTs.push_back(tMap[*t]);
			}
		}
		for (const auto& c : pt.m_tangentEndpointCertificates) {
			auto& t1 = tMap[*c.t1];
			auto& t2 = tMap[*c.t2];
			m_tangentEndpointCertificates.emplace_back(c.pointId, t1, t2);
		}
	}
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
//        res = res * 31 + hash<std::optional<cartocrow::kinetic_kelp::StraightId>>{}(to.straightId);
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
}

namespace cartocrow::kinetic_kelp {
template <class OutputIterator, class CSPoly1, class CSPoly2>
void intersectionPoints(const CSPoly1& one, const CSPoly2& other, OutputIterator out) {
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

	std::vector<OneRootPoint> intersectionPoints;
    for (const auto& inter : intersections) {
        if (inter.type() == typeid(std::pair<OneRootPoint, ArrCSTraits::Multiplicity>)) {
            auto pt = boost::get<std::pair<OneRootPoint, ArrCSTraits::Multiplicity>>(inter).first;
			intersectionPoints.push_back(pt);
        }
    }

	std::sort(intersectionPoints.begin(), intersectionPoints.end(), [](const OneRootPoint& pt1, const OneRootPoint& pt2) {
		if (pt1.x() < pt2.x()) {
			return true;
		} else if (pt1.x() > pt2.x()) {
			return false;
		} else {
			return pt1.y() < pt2.y();
		}
	});
	intersectionPoints.erase(std::unique(intersectionPoints.begin(), intersectionPoints.end()), intersectionPoints.end());

	for (const auto& ip : intersectionPoints) {
		*out++ = ip;
	}
}

bool liesOnHalf(const OneRootPoint& pt, const Straight& s, bool firstHalf);

bool doIntersect(const RationalTangent& rt1, const RationalTangent& rt2);

class PseudotriangulationGeometry {
  public:
    using TangentObject = Pseudotriangulation::TangentObject;
    using Tangent = Pseudotriangulation::Tangent;
	using TangentObjectGeometry = std::variant<Point<Exact>, RationalRadiusCircle>;

	std::unordered_map<TangentObject, TangentObjectGeometry> m_tangentObject;
	std::unordered_map<Tangent, RationalTangent> m_tangents;

    static TangentObjectGeometry geometry(const TangentObject& tangentObject, const State& state, const StateGeometry& stateGeometry, const InputInstance& input);
    static std::optional<RationalTangent> geometry(const Tangent& tangent, const TangentObjectGeometry& source, const TangentObjectGeometry& target);
	static std::optional<RationalTangent> geometry(const Tangent& tangent, const State& state, const StateGeometry& stateGeometry, const InputInstance& input);

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
            } else {
                auto p2 = std::get<Point<Exact>>(other.second);
                auto ts = rationalTangents(p2, c1);
                if (ts.has_value()) {
                    *out++ = std::pair(Tangent(PointCircle1, other.first, one.first), ts->first);
                    *out++ = std::pair(Tangent(PointCircle2, other.first, one.first), ts->second);
                }
            }
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
        }
    }

    static bool free(const RationalTangent& rt, const CSPolygon& obstacle);
    static bool free(const RationalTangent& rt, const CSPolygonWithHoles& obstacle);

	Point<Exact> tangentEndpoint(const Tangent& tangent, PointId pointId) {
		if (tangent.source->pointId == pointId) {
			return m_tangents[tangent].source();
		}
		if (tangent.target->pointId == pointId) {
			return m_tangents[tangent].target();
		}
		throw std::invalid_argument("The provided pointId is not an endpoint of the tangent.");
	}

	Point<Exact> otherTangentEndpoint(const Tangent& tangent, PointId pointId) {
		if (tangent.source->pointId == pointId) {
			return m_tangents[tangent].target();
		}
		if (tangent.target->pointId == pointId) {
			return m_tangents[tangent].source();
		}
		throw std::invalid_argument("The provided pointId is not an endpoint of the tangent.");
	}

    static std::pair<Pseudotriangulation, PseudotriangulationGeometry> pseudotriangulationTangents(const State& state, const StateGeometry& stateGeometry);
    PseudotriangulationGeometry() = default;
    PseudotriangulationGeometry(const Pseudotriangulation& pt, const State& state, const StateGeometry& stateGeometry, const InputInstance& input);
};
}

#endif //CARTOCROW_PSEUDOTRIANGULATION_H