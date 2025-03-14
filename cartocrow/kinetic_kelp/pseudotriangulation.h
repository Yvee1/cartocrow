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

#include <utility>

namespace cartocrow::kinetic_kelp {
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
class PseudotriangulationGeometry;

class Pseudotriangulation {
public:
    enum TangentObjectType {
        Circle,
        CircleStraight1, // intersection with right part of straight, viewed from this object.
        CircleStraight2, // intersection with left part of straight, viewed from this object.
        IncidentStraights, // intersection of two straights incident to the same point
        CircleCircle1,
        CircleCircle2,
    };
    struct TangentObject {
        TangentObjectType type;
        PointId pointId;
        std::optional<StraightId> straightId;
        std::optional<StraightId> otherStraightId;

        bool circleStraight() const { return type == CircleStraight1 || type == CircleStraight2; }
		bool circleTangent() const { return type == Circle; }
        bool point() const { return type != Circle; }

        TangentObject(PointId pointId) : pointId(pointId), type(Circle), straightId(std::nullopt) {};
        TangentObject(PointId pointId, StraightId straightId, bool one) : pointId(pointId), straightId(straightId), type(one ? CircleStraight1 : CircleStraight2) {};
        TangentObject(PointId pointId, StraightId straightId, StraightId otherStraightId) : pointId(pointId), straightId(straightId), otherStraightId(otherStraightId), type(IncidentStraights) {};
		TangentObject(const TangentObject& obj, const State& oldState, const State& newState) {
			type = obj.type;
			pointId = obj.pointId;
			if (!obj.straightId.has_value()) {
				straightId = std::nullopt;
			} else {
				auto [edge, oldOrbitIt] = *obj.straightId;
				auto& oldOrbits = oldState.edgeTopology.at(edge).orbits;
				auto& newOrbits = newState.edgeTopology.at(edge).orbits;
				auto i = std::distance(oldOrbits.begin(), oldOrbitIt);
				auto newOrbitIt = std::next(newOrbits.begin(), i);
				straightId = {edge, newOrbitIt};
			}
            if (!obj.otherStraightId.has_value()) {
                otherStraightId = std::nullopt;
            } else {
                auto [edge, oldOrbitIt] = *obj.otherStraightId;
                auto& oldOrbits = oldState.edgeTopology.at(edge).orbits;
                auto& newOrbits = newState.edgeTopology.at(edge).orbits;
                auto i = std::distance(oldOrbits.begin(), oldOrbitIt);
                auto newOrbitIt = std::next(newOrbits.begin(), i);
                otherStraightId = {edge, newOrbitIt};
            }
		}

        bool operator==(const TangentObject& other) const = default;

        // If this tangent object is a circle-straight intersection point that lies on the outer part of an elbow
        // then the function returns the orbit that corresponds to that elbow.
        std::optional<Orbit> elbowPoint(const State& state) {
            if (!straightId.has_value()) return std::nullopt;
            if (otherStraightId.has_value()) return std::nullopt;
            auto [edge, orbitIt] = *straightId;
            if (pointId == edge.first || pointId == edge.second) return std::nullopt;
            auto [sId, tId] = state.straightEndpoints(*straightId);
            if (pointId == sId) {
                --orbitIt;
            }
            // check whether this is the outer elbow.
            if (state.pointIdToElbows[pointId].back().second != orbitIt) return std::nullopt;
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
        bool edgeOfStraight = false;

		std::shared_ptr<TangentObject>& endpoint(PointId pointId) {
			if (source->pointId == pointId) {
				return source;
			}
			if (target->pointId == pointId) {
				return target;
			}
			throw std::invalid_argument("The provided pointId is not an endpoint of the tangent.");
		}

		std::shared_ptr<TangentObject>& otherEndpoint(PointId pointId) {
			if (source->pointId == pointId) {
				return target;
			}
			if (target->pointId == pointId) {
				return source;
			}
			throw std::invalid_argument("The provided pointId is not an endpoint of the tangent.");
		}

		std::shared_ptr<TangentObject>& endpoint(bool target) {
			return target ? this->target : this->source;
		}

		std::shared_ptr<TangentObject>& otherEndpoint(bool target) {
			return target ? this->source : this->target;
		}

		bool circleBitangent() const {
			return type == Outer1 || type == Outer2 || type == Inner1 || type == Inner2;
		}

        bool innerBitangent() const {
            return type == Inner1 || type == Inner2;
        }

        bool incidentTo(PointId pId) const {
            return source->pointId == pId || target->pointId == pId;
        }

        bool incidentTo(const std::shared_ptr<TangentObject>& tObj) const {
            return source == tObj || target == tObj;
        }

        StraightId straight() const {
            assert(edgeOfStraight);
            if (source->circleStraight()) {
                return *source->straightId;
            }
            if (target->circleStraight()) {
                return *target->straightId;
            }
            assert(source->type == IncidentStraights);
            assert(target->type == IncidentStraights);
            auto ss1 = *source->straightId;
            auto ss2 = *source->otherStraightId;
            auto ts1 = *target->straightId;
            auto ts2 = *target->otherStraightId;
            if (ss1 == ts1) {
                return ss1;
            }
            if (ss1 == ts2) {
                return ss1;
            }
            if (ss2 == ts1) {
                return ss2;
            }
            if (ss2 == ts2) {
                return ss2;
            }
            throw std::runtime_error("Could not determine straight of a tangent.");
        }

        bool operator==(const Tangent& other) const = default;
    };

    std::vector<std::shared_ptr<TangentObject>> m_tangentObjects;
    std::vector<std::shared_ptr<Tangent>> m_tangents;
	std::vector<std::list<std::shared_ptr<Tangent>>> m_pointIdToTangents; // sorted on angle

    typedef CGAL::Circulator_from_container<std::list<std::shared_ptr<Tangent>>> TangentCirculator;

    /// Certifies that consecutive tangents t1 and t2 are in the correct order on pointId.
	struct ConsecutiveCertificate {
		PointId pointId;
		std::shared_ptr<Tangent> t1;
		std::shared_ptr<Tangent> t2;

        ConsecutiveCertificate(PointId pointId, std::shared_ptr<Tangent> t1, std::shared_ptr<Tangent> t2) :
            pointId(pointId), t1(std::move(t1)), t2(std::move(t2)) {};

        bool operator==(const ConsecutiveCertificate& other) const = default;
		bool t1SubsetOft2;

		bool valid(Pseudotriangulation& pt, const State& state, const InputInstance& input, const Settings& settings);
		bool valid(Pseudotriangulation& pt, const State& state, const RationalTangent& t1G, const RationalTangent& t2G, const Point<Exact>& point);
        bool valid(Pseudotriangulation& pt, const State& state, const PseudotriangulationGeometry& ptg, const InputInstance& input);
        bool usesTangent(const std::shared_ptr<Tangent>& tangent) const { return t1 == tangent || t2 == tangent; }
        bool usesTangentObject(const std::shared_ptr<TangentObject>& tangentObject) const { return false; }
	};

    /// Certifies that the endpoint of t on pointId is an intersection point, not a tangent point of the circle.
    struct PointCertificate {
        PointId pointId;
        std::shared_ptr<Tangent> t;
        PointCertificate(PointId pointId, std::shared_ptr<Tangent> t) : pointId(pointId), t(std::move(t)) {};
        bool operator==(const PointCertificate& other) const = default;
        bool valid(const PseudotriangulationGeometry& ptg, const InputInstance& input);
        bool valid(const RationalTangent& tG, const Point<Exact>& point);
        bool valid(Pseudotriangulation& pt, const State& state, const PseudotriangulationGeometry& ptg, const InputInstance& input);
        bool usesTangent(const std::shared_ptr<Tangent>& tangent) const { return t == tangent; }
        bool usesTangentObject(const std::shared_ptr<TangentObject>& tangentObject) const { return false; }
    };

    /// Certifies that tangent t exists.
    struct ExistenceCertificate {
        std::shared_ptr<Tangent> t;
        ExistenceCertificate(std::shared_ptr<Tangent> t) : t(std::move(t)) {}
        bool operator==(const ExistenceCertificate& other) const = default;
        bool valid(Pseudotriangulation& pt, const State& state, const PseudotriangulationGeometry& ptg, const InputInstance& input);
        bool usesTangent(const std::shared_ptr<Tangent>& tangent) const { return t == tangent; }
        bool usesTangentObject(const std::shared_ptr<TangentObject>& tangentObject) const { return false; }
    };

    struct IncidentStraightsOutsideCircleCertificate {
        std::shared_ptr<TangentObject> tObj;
        IncidentStraightsOutsideCircleCertificate(std::shared_ptr<TangentObject> tObj) : tObj(std::move(tObj)) {}
        bool operator==(const IncidentStraightsOutsideCircleCertificate& other) const = default;
        bool usesTangent(const std::shared_ptr<Tangent>& tangent) const { return false; }
        bool usesTangentObject(const std::shared_ptr<TangentObject>& tangentObject) const { return tObj == tangentObject; }
        bool valid(Pseudotriangulation& pt, const State& state, const PseudotriangulationGeometry& ptg, const InputInstance& input);
    };

    using Certificate = std::variant<ConsecutiveCertificate, PointCertificate, ExistenceCertificate, IncidentStraightsOutsideCircleCertificate>;

    static bool usesTangent(const Certificate& certificate, const std::shared_ptr<Tangent>& t);
    static bool usesTangentObject(const Certificate& certificate, const std::shared_ptr<TangentObject>& tObj);
    bool valid(Certificate& certificate, const State& state, const PseudotriangulationGeometry& ptg, const InputInstance& input);

    /// List of certificates. Needs to be checked front to back as the ExistenceCertificates are in the front,
    /// and other certificates rely on their relevant tangents existence.
    std::list<Certificate> m_certificates;
    void addTangent(const std::shared_ptr<Tangent>& t);
    void addTangentObject(const std::shared_ptr<TangentObject>& tObj);
    void fix(Certificate& certificate, State& state, const Settings& settings);
	void fix(ConsecutiveCertificate& certificate, State& state, const Settings& settings);
    void fix(PointCertificate& certificate, State& state, const Settings& settings);
    void fix(ExistenceCertificate& certificate, State& state, const Settings& settings);
    void fix(IncidentStraightsOutsideCircleCertificate& certificate, State& state, const Settings& settings);
    void addAndRemove(ConsecutiveCertificate& certificate, State& state, const std::shared_ptr<Tangent>& oldTangent, const std::shared_ptr<Tangent>& newTangent);
    /// Snap endpoint t1->endpoint(pId0)
    void snapTangentToPoint(State& state, PointId pId0, const std::shared_ptr<Tangent>& t1, const std::shared_ptr<Tangent>& t2);
    void handleIntersectingIncidentStraights(ConsecutiveCertificate& certificate, State& state);
    void splitStraight(State& state, const Settings& settings, const std::shared_ptr<Tangent>& longer, const std::shared_ptr<Tangent>& shorter, const std::shared_ptr<Tangent>& otherEdge, PointId pId0, StraightId oldStraight);
    void collapseElbow(ConsecutiveCertificate& certificate, State& state);
	void removeTangent(std::shared_ptr<Tangent> t);
    void removeTangentObject(std::shared_ptr<TangentObject> tObj);
	void maybeAddCertificate(PointId pId, const std::shared_ptr<Tangent>& t1, const std::shared_ptr<Tangent>& t2, const State& state);
    std::pair<std::shared_ptr<Pseudotriangulation::Tangent>, std::shared_ptr<Pseudotriangulation::Tangent>> neighbouringTangents(PointId pId, const std::shared_ptr<Tangent>& t);
    std::shared_ptr<Pseudotriangulation::Tangent> previousTangent(PointId pId, const std::shared_ptr<Tangent>& t);
    std::shared_ptr<Pseudotriangulation::Tangent> nextTangent(PointId pId, const std::shared_ptr<Tangent>& t);
    std::optional<std::shared_ptr<Tangent>> otherEdgeOfStraight(const std::shared_ptr<Tangent>& t);
	TangentCirculator tangentCirculator(PointId pId, const std::shared_ptr<Tangent>& t);
    TangentCirculator tangentsCirculator(PointId pId);
	std::shared_ptr<Pseudotriangulation::TangentObject> circleTangentObject(PointId pointId) const;

    CGAL::Orientation sourceOrientation(const std::shared_ptr<Tangent>& t, const State& state) {
        if (t->source->elbowPoint(state).has_value()) {
            if (t->source->type == CircleStraight2) {
                return CGAL::CLOCKWISE;
            } else {
                assert(t->source->type == CircleStraight1);
                return CGAL::COUNTERCLOCKWISE;
            }
        }

        if (t->type == Outer1 || t->type == Inner1 || t->type == CirclePoint1) {
            return CGAL::COUNTERCLOCKWISE;
        }
        if (t->type == Outer2 || t->type == Inner2 || t->type == CirclePoint2) {
            return CGAL::CLOCKWISE;
        }
        return CGAL::COLLINEAR;
    }

    CGAL::Orientation targetOrientation(const std::shared_ptr<Tangent>& t, const State& state) {
        if (t->target->elbowPoint(state).has_value()) {
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

		std::unordered_map<TangentObject, std::shared_ptr<TangentObject>, HashTangentObject> objMap;
		for (const auto& obj : pt.m_tangentObjects) {
			m_tangentObjects.push_back(std::make_shared<TangentObject>(*obj, oldState, newState));
			objMap[*obj] = m_tangentObjects.back();
		}
		std::unordered_map<Tangent, std::shared_ptr<Tangent>, HashTangent> tMap;
		for (const auto& t : pt.m_tangents) {
			auto sObj = objMap[*t->source];
			auto tObj = objMap[*t->target];
			m_tangents.push_back(std::make_shared<Tangent>(t->type, sObj, tObj, t->edgeOfStraight));
			tMap[*t] = m_tangents.back();
		}
		for (const auto& ts : pt.m_pointIdToTangents) {
			auto& newTs = m_pointIdToTangents.emplace_back();
			for (const auto& t : ts) {
				newTs.push_back(tMap[*t]);
			}
		}
		for (const auto& c : pt.m_certificates) {
            if (auto ccP = std::get_if<ConsecutiveCertificate>(&c)) {
                auto& t1 = tMap[*ccP->t1];
                auto& t2 = tMap[*ccP->t2];
                m_certificates.emplace_back(ConsecutiveCertificate(ccP->pointId, t1, t2));
            } else if (auto pcP = std::get_if<PointCertificate>(&c)) {
                auto& t = tMap[*pcP->t];
                m_certificates.emplace_back(PointCertificate(pcP->pointId, t));
            } else if (auto ecP = std::get_if<ExistenceCertificate>(&c)) {
                auto& t = tMap[*ecP->t];
                m_certificates.emplace_back(ExistenceCertificate(t));
            } else if (auto isoccP = std::get_if<IncidentStraightsOutsideCircleCertificate>(&c)) {
                auto& tObj = objMap[*isoccP->tObj];
                m_certificates.emplace_back(IncidentStraightsOutsideCircleCertificate(tObj));
            } else {
                throw std::runtime_error("Unhandled certificate type.");
            }
		}
	}
};

std::string name(TangentType tt);
std::string name(Pseudotriangulation::TangentObjectType tot);
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

    static std::optional<TangentObjectGeometry> geometry(const TangentObject& tangentObject, const State& state, const StateGeometry& stateGeometry, const InputInstance& input);
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

	Point<Exact> tangentEndpoint(const Tangent& tangent, PointId pointId) const {
		if (tangent.source->pointId == pointId) {
			return m_tangents.at(tangent).source();
		}
		if (tangent.target->pointId == pointId) {
			return m_tangents.at(tangent).target();
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