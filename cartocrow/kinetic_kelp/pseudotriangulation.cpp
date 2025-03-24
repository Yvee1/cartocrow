#include "pseudotriangulation.h"
#include "cartocrow/circle_segment_helpers/cs_polyline_helpers.h"
#include "cartocrow/circle_segment_helpers/poly_line_gon_intersection.h"

#include <future>
#include <utility>

namespace cartocrow::kinetic_kelp {
std::ostream& operator<<(std::ostream& os, const Pseudotriangulation::TangentObject& t) {
    os << "TangentObject(" << name(t.type) << ", pId: " << t.pointId <<
       ", straight: " << (t.straightId ? std::to_string(t.straightId->first.first) + " " + std::to_string(t.straightId->first.second) : "no") <<
       ", other straight: " << (t.otherStraightId ? std::to_string(t.otherStraightId->first.first) + " " + std::to_string(t.otherStraightId->first.second) : "no") << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Pseudotriangulation::Tangent& t) {
    os << name(t.type) << (t.edgeOfStraight ? " and edge of straight" : "") << "," << *t.source << " -> " << *t.target;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Pseudotriangulation::ConsecutiveCertificate& c)
{
    os << "pId: " << c.pointId << " t1(" << *c.t1 << ") t2(" << *c.t2 << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Pseudotriangulation::PointCertificate& c)
{
    os << "pId: " << c.pointId << " t(" << *c.t << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Pseudotriangulation::IncidentStraightsOutsideCircleCertificate& c)
{
    os << "tObj(" << *c.tObj << ")";
    return os;
}

bool circlePointLiesOnArc(const Point<Exact>& point, const RationalCircularArc& arc) {
	auto sd = (arc.source - arc.circle.center).direction();
	auto td = (arc.target - arc.circle.center).direction();
	auto d = (point - arc.circle.center).direction();
	return arc.orientation == CGAL::COUNTERCLOCKWISE ? d.counterclockwise_in_between(sd, td) : d.counterclockwise_in_between(td, sd);
}

bool liesOnHalf(const OneRootPoint& pt, const Straight& s, bool firstHalf) {
    auto pl = firstHalf ? s.firstHalf.polyline() : s.secondHalf.polyline();
    for (auto eit = pl.edges_begin(); eit != pl.edges_end(); ++eit) {
        CSXMCurve xmCurve(eit->source(), eit->target());
        if (liesOn(pt, xmCurve)) return true;
    }
    return false;
}

bool PseudotriangulationGeometry::free(const cartocrow::RationalTangent& rt, const CSPolygon& obstacle) {
    auto pl = polylineToCSPolyline(rt.polyline());
    const auto& pgn = obstacle;
    auto plBbox = CGAL::bbox_2(pl.curves_begin(), pl.curves_end());
    auto pgnBbox = CGAL::bbox_2(pgn.curves_begin(), pgn.curves_end());
    plBbox.dilate(100);
    pgnBbox.dilate(100);
    bool overlap = do_overlap(plBbox, pgnBbox);
    if (overlap && !intersection(pl, pgn, true).empty()) {
        return false;
    }
    return true;
}

bool PseudotriangulationGeometry::free(const cartocrow::RationalTangent& rt, const CSPolygonWithHoles& obstacle) {
    auto pl = polylineToCSPolyline(rt.polyline());
    const auto& pgn = obstacle;
    auto plBbox = CGAL::bbox_2(pl.curves_begin(), pl.curves_end());
    auto pgnBbox = CGAL::bbox_2(pgn.outer_boundary().curves_begin(), pgn.outer_boundary().curves_end());
    plBbox.dilate(100);
    pgnBbox.dilate(100);
    bool overlap = do_overlap(plBbox, pgnBbox);
    if (overlap && !intersection(pl, pgn, true).empty()) {
        return false;
    }
    return true;
}

bool doIntersect(const RationalTangent& rt1, const RationalTangent& rt2) {
    auto pl1 = rt1.polyline();
    auto pl2 = rt2.polyline();

    auto pl1S = pl1.source();
    auto pl2S = pl2.source();
    auto pl1T = pl1.target();
    auto pl2T = pl2.target();

    if (pl1S == pl2S || pl1T == pl2T || pl1S == pl2T || pl1T == pl2S) return false;

    for (auto eit1 = pl1.edges_begin(); eit1 != pl1.edges_end(); ++eit1) {
        for (auto eit2 = pl2.edges_begin(); eit2 != pl2.edges_end(); ++eit2) {
            if (CGAL::do_intersect(*eit1, *eit2)) {
                return true;
            }
        }
    }

    return false;
}

std::pair<Pseudotriangulation, PseudotriangulationGeometry> PseudotriangulationGeometry::initialize(InputInstance& input, State& state, const StateGeometry& stateGeometry, int k) {
    Pseudotriangulation pt;
    pt.m_k = k;
    PseudotriangulationGeometry ptg;

    for (const auto& [pId, circle] : stateGeometry.vertexGeometry) {
        pt.m_tangentObjects.push_back(std::make_shared<TangentObject>(pId));
        auto& elbows = state.pointIdToElbows[pId];

        RationalRadiusCircle circleGeometry;
        if (elbows.empty()) {
            circleGeometry = circle;
        } else {
            auto& last = elbows.back();
            auto orbit = stateGeometry.elbow(last, state).orbit();
            auto r = orbit.outerRadius;
            circleGeometry = RationalRadiusCircle(circle.center, r);
        }
        ptg.m_tangentObject[*(pt.m_tangentObjects.back())] = circleGeometry;
        auto& incidentEdges = state.pointIdToEdges[pId];
        for (const auto& edge : incidentEdges) {
            if (input[edge.first].category != k) continue;
			auto& orbits = state.edgeTopology.at(edge).orbits;
            auto orbitsIt = edge.first == pId ? orbits.begin() : orbits.end();
            auto straightId = std::pair(edge, orbitsIt);
            auto straight = stateGeometry.straight(straightId, state);
            std::vector<OneRootPoint> ipts;
            intersectionPoints(straight.csPolygon(), circleToCSPolygon(circleGeometry.circle()), std::back_inserter(ipts));
            for (const auto& ipt : ipts) {
                Point<Exact> approx = pretendExact(approximateOneRootPoint(ipt));

                auto pId2 = orbitsIt == orbits.end() ? edge.second : orbitsIt->pointId;
				auto rev = pId == pId2;
				auto thisPoint = straight.backboneEndpoint(rev);
				auto otherPoint = straight.backboneEndpoint(!rev);

				auto orient = CGAL::orientation(thisPoint, otherPoint, approx);
				bool one = orient == CGAL::CLOCKWISE;

                pt.m_tangentObjects.push_back(std::make_shared<TangentObject>(pId, straightId, one));
                assert(!ptg.m_tangentObject.contains(*(pt.m_tangentObjects.back())));
                ptg.m_tangentObject[*(pt.m_tangentObjects.back())] = approx;
            }
        }

        auto& orbitElbows = state.pointIdToElbows[pId];
        for (const auto& elbowId : orbitElbows) {
            if (input[elbowId.first.first].category != k) continue;
            auto elbow = stateGeometry.elbow(elbowId, state);
			auto& [edge, orbitIt] = elbowId;
            auto orbitItNext = orbitIt;
            ++orbitItNext;
			auto& orbits = state.edgeTopology.at(edge).orbits;
            for (const auto& [straight, straightId] : {std::pair(*(elbow.prev), StraightId{edge, orbitIt}), std::pair(*(elbow.next), StraightId{edge, orbitItNext})}) {
                std::vector<OneRootPoint> ipts;
                intersectionPoints(straight.csPolygon(), circleToCSPolygon(circleGeometry.circle()), std::back_inserter(ipts));
                for (const auto &ipt: ipts) {
                    Point<Exact> approx = pretendExact(approximateOneRootPoint(ipt));

					auto orbitsIt = straightId.second;

					auto pId2 = orbitsIt == orbits.end() ? edge.second : orbitsIt->pointId;
					auto rev = pId == pId2;
					auto thisPoint = straight.backboneEndpoint(rev);
					auto otherPoint = straight.backboneEndpoint(!rev);

					auto orient = CGAL::orientation(thisPoint, otherPoint, approx);
					bool one = orient == CGAL::CLOCKWISE;

					pt.m_tangentObjects.push_back(std::make_shared<TangentObject>(pId, straightId, one));
                    assert(!ptg.m_tangentObject.contains(*(pt.m_tangentObjects.back())));
                    ptg.m_tangentObject[*(pt.m_tangentObjects.back())] = approx;
                }
            }
        }
        // todo circle-circle intersections
    }

    // The tangents in the final pseudotriangulation.
    // We declare it here already as we will add tangents for each side of a straight.
    std::vector<std::pair<Tangent, RationalTangent>> finalTangents;

    // All tangents between tangent objects
    std::vector<std::pair<Tangent, RationalTangent>> allTangents;
    for (int objIndex1 = 0; objIndex1 < pt.m_tangentObjects.size(); ++objIndex1) {
        auto& obj1 = pt.m_tangentObjects[objIndex1];
        auto& obj1G = ptg.m_tangentObject[*obj1];
        for (int objIndex2 = objIndex1 + 1; objIndex2 < pt.m_tangentObjects.size(); ++objIndex2) {
            auto& obj2 = pt.m_tangentObjects[objIndex2];
            auto& obj2G = ptg.m_tangentObject[*obj2];
            if (obj1->pointId == obj2->pointId) continue;
            if (obj1->circleStraight() && obj2->circleStraight()) {
                // Is there edge between?
                auto sId1 = *(obj1->straightId);
                auto sId2 = *(obj2->straightId);
                if (sId1 == sId2) {
                    // obj1 and obj2 are the circle-straight intersection points
                    // arising from the same straight but different circles.
                    // if the points belong to the same part of the straight then
                    // we connect them.
                    if (obj1->type != obj2->type) {
                        auto p1 = std::get<Point<Exact>>(ptg.m_tangentObject[*obj1]);
                        auto p2 = std::get<Point<Exact>>(ptg.m_tangentObject[*obj2]);
                        Tangent straightTangent(PointPoint, obj1, obj2, true);
                        finalTangents.emplace_back(straightTangent, RationalTangent(Segment<Exact>(p1, p2)));
                    }
                    continue;
                }
            }
			if (obj1->elbowPoint(state) || obj2->elbowPoint(state)) continue;
            tangents(std::pair(obj1, obj1G), std::pair(obj2, obj2G), std::back_inserter(allTangents));
        }
    }

    auto task = [&input, k, &allTangents, &state, &stateGeometry](int iStart, int iEnd) {
        std::vector<std::pair<Tangent, RationalTangent>> freeTangents;
        for (int tangentIndex = iStart; tangentIndex < iEnd; ++tangentIndex) {
            const auto& t = allTangents[tangentIndex];
            bool f = true;
            for (const auto& [pId, circle]: stateGeometry.vertexGeometry) {
                auto& elbows = state.pointIdToElbows[pId];

                RationalRadiusCircle circleGeometry;
                if (elbows.empty()) {
                    circleGeometry = circle;
                } else {
                    auto& last = elbows.back();
                    auto& orbit = stateGeometry.elbow(last, state).orbit();
                    auto r = orbit.outerRadius;
                    circleGeometry = RationalRadiusCircle(circle.center, r);
                }

                if (pId == t.first.source->pointId || pId == t.first.target->pointId) {
                    CSPolygonSet perforatedCircle(circleToCSPolygon(circleGeometry.circle()));
                    if (pId == t.first.source->pointId) {
                        Circle<Exact> hole(t.second.source(), CGAL::square(M_EPSILON));
                        perforatedCircle.difference(circleToCSPolygon(hole));
                    }
                    if (pId == t.first.target->pointId) {
                        Circle<Exact> hole(t.second.target(), CGAL::square(M_EPSILON));
                        perforatedCircle.difference(circleToCSPolygon(hole));
                    }
                    std::vector<CSPolygonWithHoles> result;
                    perforatedCircle.polygons_with_holes(std::back_inserter(result));
                    assert(result.size() == 1);
                    if (!free(t.second, result[0])) {
                        f = false;
                        break;
                    }
                } else if (!free(t.second, circleToCSPolygon(circleGeometry.circle()))) {
                    f = false;
                    break;
                }
            }
            if (!f) continue;
            for (const auto& [edge, edgeGeometry]: stateGeometry.edgeGeometry) {
                if (input[edge.first].category != k) continue;
                auto sourceStraightId = t.first.source->straightId;
                auto targetStraightId = t.first.target->straightId;
                auto special = sourceStraightId.has_value() && sourceStraightId->first == edge ||
                               targetStraightId.has_value() && targetStraightId->first == edge;
                auto edgeGeom = edgeGeometry.csPolygon();

                if (special) {
                    CSPolygonSet pgnSet(edgeGeom);
                    if (sourceStraightId.has_value() && sourceStraightId->first == edge) {
                        auto epsCircle = Circle<Exact>(t.second.source(), CGAL::square(M_EPSILON));
                        pgnSet.difference(circleToCSPolygon(epsCircle));
                    }
                    if (targetStraightId.has_value() && targetStraightId->first == edge) {
                        auto epsCircle = Circle<Exact>(t.second.target(), CGAL::square(M_EPSILON));
                        pgnSet.difference(circleToCSPolygon(epsCircle));
                    }
                    std::vector<CSPolygonWithHoles> pgns;
                    pgnSet.polygons_with_holes(std::back_inserter(pgns));

                    assert(pgns.size() == 1);
                    if (!free(t.second, pgns[0])) {
                        f = false;
                        break;
                    }
                } else {
                    if (!free(t.second, edgeGeom)) {
                        f = false;
                        break;
                    }
                }
            }
            if (!f) continue;
            freeTangents.push_back(t);
        }
        return freeTangents;
    };

    using Result = std::vector<std::pair<Tangent, RationalTangent>>;
    std::vector<std::future<Result>> results;
    int n = allTangents.size();
    int nThreads = std::min(128, n);
    double step = n / static_cast<double>(nThreads);
    for (int i = 0; i < n / step; ++i) {
        int iStart = std::ceil(i * step);
        int iEnd = std::ceil((i + 1) * step);
        results.push_back(std::async(std::launch::async, task, iStart, iEnd));
    }

    std::vector<std::pair<Tangent, RationalTangent>> freeTangents;

    for (auto& futureResult : results) {
        Result result = futureResult.get();
        std::copy(result.begin(), result.end(), std::back_inserter(freeTangents));
    }

    std::sort(freeTangents.begin(), freeTangents.end(), [](const auto& t1, const auto& t2) {
        RationalTangent rt1 = t1.second;
        RationalTangent rt2 = t2.second;
        return CGAL::squared_distance(rt1.source(), rt1.target()) > CGAL::squared_distance(rt2.source(), rt2.target());
    });

    finalTangents.push_back(freeTangents.back());
    freeTangents.pop_back();

    while (!freeTangents.empty()) {
        auto t = freeTangents.back();
        freeTangents.pop_back();

        bool disjoint = true;
        for (const auto& other : finalTangents) {
            if (doIntersect(t.second, other.second)) {
                disjoint = false;
                break;
            }
        }

        if (disjoint)
            finalTangents.push_back(t);
    }

	// Add tangents to pt and ptg
	pt.m_pointIdToTangents.resize(stateGeometry.vertexGeometry.size());
    for (const auto& [t, tg] : finalTangents) {
		auto tP = std::make_shared<Tangent>(t);
        pt.m_tangents.push_back(tP);
		auto sPID = t.source->pointId;
		auto tPID = t.target->pointId;
		pt.m_pointIdToTangents[sPID].push_back(tP);
		pt.m_pointIdToTangents[tPID].push_back(tP);
        ptg.m_tangents[t] = tg;
    }

	// Sort tangents around each point, and make certificates
	for (int pId = 0; pId < pt.m_pointIdToTangents.size(); ++pId) {
		auto& tangents = pt.m_pointIdToTangents[pId];
		auto& point = stateGeometry.vertexGeometry.at(pId).center;
		tangents.sort([pId, &ptg, point](const std::shared_ptr<Tangent>& t1, const std::shared_ptr<Tangent>& t2) {
			auto p1 = ptg.tangentEndpoint(*t1, pId);
		  	auto p2 = ptg.tangentEndpoint(*t2, pId);
			auto d1 = (p1 - point).direction();
		  	auto d2 = (p2 - point).direction();
			if (d1 < d2) {
				return true;
			} else if (d1 > d2) {
				return false;
			} else {
				auto q1 = ptg.otherTangentEndpoint(*t1, pId);
				auto q2 = ptg.otherTangentEndpoint(*t2, pId);
				d1 = (q1 - p1).direction();
				d2 = (q2 - p2).direction();
				auto ref = (point - p1).direction();
				return d1.counterclockwise_in_between(ref, d2);
			}
		});

		for (auto t1It = tangents.begin(); t1It != tangents.end(); ++t1It) {
			auto t2It = t1It;
			++t2It;
			if (t2It == tangents.end()) {
				t2It = tangents.begin();
			}
			auto t1 = *t1It;
			auto t2 = *t2It;
			pt.maybeAddCertificate(pId, t1, t2, state);
		}
	}

    for (int pId = 0; pId < pt.m_pointIdToTangents.size(); ++pId) {
        auto& tangents = pt.m_pointIdToTangents[pId];
        for (const auto& t : tangents) {
            if (!t->endpoint(pId)->point()) continue;
            if (t->edgeOfStraight) continue;
            pt.m_certificates.emplace_back(Pseudotriangulation::PointCertificate(pId, t));
        }
    }

    for (const auto& t : pt.m_tangents) {
        pt.m_certificates.push_front(Pseudotriangulation::ExistenceCertificate(t));
    }

    return {pt, ptg};
}

std::optional<PseudotriangulationGeometry::TangentObjectGeometry>
PseudotriangulationGeometry::geometry(const TangentObject& tangentObject, const State& state, const StateGeometry& stateGeometry, const InputInstance& input) {
    auto pId = tangentObject.pointId;
    auto& elbows = state.pointIdToElbows[pId];
    RationalRadiusCircle circleGeometry;
    if (elbows.empty()) {
        circleGeometry = stateGeometry.vertexGeometry.at(pId);
    } else {
        auto r = stateGeometry.elbow(elbows.back(), state).orbit().outerRadius;
        circleGeometry = RationalRadiusCircle(input[tangentObject.pointId].point, r);
    }

    if (tangentObject.type == Pseudotriangulation::TangentObjectType::Circle) {
        return circleGeometry;
    }

    if (tangentObject.straightId.has_value() && !tangentObject.otherStraightId.has_value()) {
        auto straightId = *(tangentObject.straightId);
        const auto& straight = stateGeometry.straight(straightId, state);
		auto& edge = straightId.first;
		auto& orbits = state.edgeTopology.at(edge).orbits;
        std::vector<OneRootPoint> ipts;
        intersectionPoints(straight.csPolygon(), circleToCSPolygon(circleGeometry.circle()), std::back_inserter(ipts));
        for (const auto& ipt: ipts) {
			Point<Exact> approx = pretendExact(approximateOneRootPoint(ipt));

			auto orbitsIt = straightId.second;
            auto pId2 = orbitsIt == orbits.end() ? edge.second : orbitsIt->pointId;

			auto rev = pId == pId2;
			auto thisPoint = straight.backboneEndpoint(rev);
			auto otherPoint = straight.backboneEndpoint(!rev);

			auto orient = CGAL::orientation(thisPoint, otherPoint, approx);
			bool one = orient == CGAL::CLOCKWISE;

            if (one && tangentObject.type == Pseudotriangulation::CircleStraight1 ||
                !one && tangentObject.type == Pseudotriangulation::CircleStraight2) {
                return approx;
            }
        }
        return std::nullopt;
    }

    if (tangentObject.straightId.has_value() && tangentObject.otherStraightId.has_value()) {
        auto straightId = *(tangentObject.straightId);
        const auto& straight = stateGeometry.straight(straightId, state);
        auto otherStraightId = *(tangentObject.otherStraightId);
        const auto& otherStraight = stateGeometry.straight(otherStraightId, state);

        auto csPl1 = polylineToCSPolyline(straight.firstHalf.polyline());
        auto csPl2 = polylineToCSPolyline(straight.secondHalf.polyline());
        auto csPl3 = polylineToCSPolyline(otherStraight.firstHalf.polyline());
        auto csPl4 = polylineToCSPolyline(otherStraight.secondHalf.polyline());

        std::vector<OneRootPoint> ipts;
        intersectionPoints(csPl1, csPl3, std::back_inserter(ipts));
        intersectionPoints(csPl1, csPl4, std::back_inserter(ipts));
        intersectionPoints(csPl2, csPl3, std::back_inserter(ipts));
        intersectionPoints(csPl2, csPl4, std::back_inserter(ipts));

        assert(ipts.size() <= 1);
        if (ipts.empty())
            return std::nullopt;
        return pretendExact(approximateOneRootPoint(ipts[0]));
    }

    throw std::runtime_error("Unexpected tangent object type.");
}

std::optional<RationalTangent>
PseudotriangulationGeometry::geometry(const Tangent& tangent, const TangentObjectGeometry& source, const TangentObjectGeometry& target) {
    if (tangent.source->type == Pseudotriangulation::TangentObjectType::Circle &&
       tangent.target->type == Pseudotriangulation::TangentObjectType::Circle) {
        auto c1 = std::get<RationalRadiusCircle>(source);
        auto c2 = std::get<RationalRadiusCircle>(target);
        if (tangent.type == Outer1 || tangent.type == Outer2) {
            auto outer = rationalBitangents(c1, c2, false);
            if (!outer.has_value()) return std::nullopt;
            if (tangent.type == Outer1)
                return outer->first;
            return outer->second;
        }
        if (tangent.type == Inner1 || tangent.type == Inner2) {
            auto inner = rationalBitangents(c1, c2, true);
            if (!inner.has_value()) return std::nullopt;
            if (tangent.type == Inner1)
                return inner->first;
            return inner->second;
        }
    }

    if (tangent.type == PointPoint) {
        return RationalTangent({std::get<Point<Exact>>(source), std::get<Point<Exact>>(target)});
    }

    if (auto p1P = std::get_if<Point<Exact>>(&source)) {
        auto p1 = *p1P;
        auto c2 = std::get<RationalRadiusCircle>(target);
        auto ts = rationalTangents(p1, c2);
        if (!ts.has_value()) return std::nullopt;
        if (tangent.type == PointCircle1) {
            return ts->first;
        }
        if (tangent.type == PointCircle2) {
            return ts->second;
        }
    }

    if (auto p2P = std::get_if<Point<Exact>>(&target)) {
        auto p2 = *p2P;
        auto c1 = std::get<RationalRadiusCircle>(source);
        auto ts = rationalTangents(c1, p2);
        if (!ts.has_value()) return std::nullopt;
        if (tangent.type == CirclePoint1) {
            return ts->first;
        }
        if (tangent.type == CirclePoint2) {
            return ts->second;
        }
    }

    throw std::runtime_error("Unexpected tangent type.");
}

std::optional<RationalTangent>
PseudotriangulationGeometry::geometry(const Tangent& tangent, const State& state, const StateGeometry& stateGeometry, const InputInstance& input) {
	auto sObj = geometry(*tangent.source, state, stateGeometry, input);
	auto tObj = geometry(*tangent.target, state, stateGeometry, input);
    if (sObj.has_value() && tObj.has_value())
	    return geometry(tangent, *sObj, *tObj);
    return std::nullopt;
}

PseudotriangulationGeometry::PseudotriangulationGeometry(const Pseudotriangulation& pt, const State& state, const StateGeometry& stateGeometry, const InputInstance& input) {
    for (const auto& tObj : pt.m_tangentObjects) {
        try {
            auto g = geometry(*tObj, state, stateGeometry, input);
            if (g.has_value()) {
                m_tangentObject[*tObj] = *g;
            } else {
                std::cerr << "Tangent object does not exist!" << std::endl;
            }
        } catch (...) {
            std::cerr << "TangentObjectGeometry computation failed!" << std::endl;
        }
    }
    for (const auto& t : pt.m_tangents) {
        auto ts = *(t->source);
        auto tt = *(t->target);
        bool success = false;
        if (m_tangentObject.contains(ts) && m_tangentObject.contains(tt)) {
            auto sObj = m_tangentObject[ts];
            auto tObj = m_tangentObject[tt];
            auto g = geometry(*t, sObj, tObj);
            if (g.has_value()) {
                m_tangents[*t] = *g;
                success = true;
            }
        }
        if (!success) {
            std::cerr << "Tangent " << *t << " does not exist!" << std::endl;
        }
    }
}

bool
Pseudotriangulation::PointCertificate::valid(const RationalTangent& tG, const Point<Exact>& point) {
    bool tR = t->target->pointId == pointId;
    auto v = tG.target() - tG.source();
    if (tR) {
        v = -v;
    }

    auto perp = ((tR ? tG.target() : tG.source()) - point).perpendicular(CGAL::CLOCKWISE);
    return CGAL::determinant(perp, v) > 0;
}

bool
Pseudotriangulation::PointCertificate::valid(const PseudotriangulationGeometry& ptg, const InputInstance& input) {
    if (!ptg.m_tangents.contains(*t))
        throw std::runtime_error("Tangent does not exist!");
    auto& point = input[pointId].point;
    return valid(ptg.m_tangents.at(*t), point);
}

bool
Pseudotriangulation::PointCertificate::valid(Pseudotriangulation& pt, const State& state, const PseudotriangulationGeometry& ptg, const InputInstance& input) {
    return valid(ptg, input);
}

bool
Pseudotriangulation::ExistenceCertificate::valid(Pseudotriangulation& pt, const State& state, const PseudotriangulationGeometry& ptg, const InputInstance& input) {
    bool incidentStraightsHit = false;
    if (t->source->type == IncidentStraights || t->target->type == IncidentStraights) {
        auto& inter = t->source->type == IncidentStraights ? t->source : t->target;
        auto& other = t->source == inter ? t->target : t->source;
        auto ep = other->elbowPoint(state);
        if (ep.has_value()) {
            incidentStraightsHit = true;
        }
    }
    // todo handle all cases of colliding circles.
    if (t->innerBitangent() || incidentStraightsHit) {
        return ptg.m_tangents.contains(*t);
    }
    return true;
}

bool
Pseudotriangulation::IncidentStraightsOutsideCircleCertificate::valid(Pseudotriangulation& pt, const State& state, const PseudotriangulationGeometry& ptg, const InputInstance& input) {
    assert(tObj->type == IncidentStraights);
    if (!ptg.m_tangentObject.contains(*tObj)) return true;
    auto p = std::get<Point<Exact>>(ptg.m_tangentObject.at(*tObj));
    auto circleObj = pt.circleTangentObject(tObj->pointId);
    auto circle = std::get<RationalRadiusCircle>(ptg.m_tangentObject.at(*circleObj));
    return CGAL::squared_distance(circle.center, p) >= CGAL::square(circle.radius);
}

bool
Pseudotriangulation::InnerElbowCertificate::valid(Pseudotriangulation& pt, const State& state, const PseudotriangulationGeometry& ptg, const InputInstance& input) {
	return valid(ptg.m_tangents.at(*t1), ptg.m_tangents.at(*t2));
}

bool
Pseudotriangulation::InnerElbowCertificate::valid(const RationalTangent& t1G, const RationalTangent& t2G) {
	bool t1R = t1->target->pointId == pointId;
	bool t2R = t2->target->pointId == pointId;
	auto v1 = t1G.target() - t1G.source();
	auto v2 = t2G.target() - t2G.source();
	if (t1R) {
		v1 = -v1;
	}
	if (t2R) {
		v2 = -v2;
	}
	return CGAL::determinant(v1, v2) < 0;
}

bool
Pseudotriangulation::ConsecutiveCertificate::valid(Pseudotriangulation& pt, const State& state, const RationalTangent& t1G, const RationalTangent& t2G, const Point<Exact>& point) {
	bool t1R = t1->target->pointId == pointId;
	bool t2R = t2->target->pointId == pointId;
	auto v1 = t1G.target() - t1G.source();
	auto v2 = t2G.target() - t2G.source();
	if (t1R) {
		v1 = -v1;
	}
	if (t2R) {
		v2 = -v2;
	}

    auto q = t1R ? t1G.target() : t1G.source();
    auto r = t2R ? t2G.target() : t2G.source();
    bool valid;
    // Endpoints of tangents in correct order around the circle?
    auto endpointOrientation = CGAL::orientation(point, q, r);
    if (endpointOrientation == CGAL::COUNTERCLOCKWISE) {
        valid = true;
    } else if (endpointOrientation == CGAL::CLOCKWISE) {
        valid = false;
    } else {
        // In this case, the two tangents have the same endpoint on this circle.
        // Do the targets have the right orientation?
	    if (pt.orientation(t1, t1R, state) == CGAL::CLOCKWISE && pt.orientation(t2, t2R, state) == CGAL::COUNTERCLOCKWISE ||
	        pt.orientation(t2, t2R, state) == CGAL::CLOCKWISE && pt.orientation(t1, t1R, state) == CGAL::COUNTERCLOCKWISE) {
	    	valid = CGAL::determinant(v1, v2) < 0;
	    } else {
	    	valid = CGAL::determinant(v1, v2) > 0;
	    }
    }

	t1SubsetOft2 = v1.squared_length() < v2.squared_length();

	return valid;
}

bool
Pseudotriangulation::ConsecutiveCertificate::valid(Pseudotriangulation& pt, const State& state, const InputInstance& input, const Settings& settings) {
	StateGeometry stateGeometry(state, input, settings);
	auto t1G = PseudotriangulationGeometry::geometry(*t1, state, stateGeometry, input);
    if (!t1G.has_value()) {
        throw std::runtime_error("Tangent does not exist!");
    }
	auto t2G = PseudotriangulationGeometry::geometry(*t2, state, stateGeometry, input);
    if (!t2G.has_value()) {
        throw std::runtime_error("Tangent does not exist!");
    }
    auto& point = input[pointId].point;
	return valid(pt, state, *t1G, *t2G, point);
}

bool Pseudotriangulation::ConsecutiveCertificate::valid(Pseudotriangulation& pt, const State& state, const PseudotriangulationGeometry& ptg, const InputInstance& input) {
    if (!ptg.m_tangents.contains(*t1)) {
        throw std::runtime_error("Tangent does not exist!");
    }
	std::optional<RationalTangent> t2G;
    if (!ptg.m_tangents.contains(*t2)) {
        throw std::runtime_error("Tangent does not exist!");
    }
    auto& point = input[pointId].point;

	return valid(pt, state, ptg.m_tangents.at(*t1), ptg.m_tangents.at(*t2), point);
}

bool Pseudotriangulation::valid(Pseudotriangulation::Certificate& certificate, const State& state,
                                const PseudotriangulationGeometry& ptg, const InputInstance& input) {
    return std::visit([&](auto& c) { return c.valid(*this, state, ptg, input); }, certificate);
}

std::string name(TangentType tt) {
	switch (tt) {
	case Outer1:
		return "Outer1";
	case Outer2:
		return "Outer2";
	case Inner1:
		return "Inner1";
	case Inner2:
		return "Inner2";
	case PointCircle1:
		return "PointCircle1";
	case PointCircle2:
		return "PointCircle2";
    case CirclePoint1:
        return "CirclePoint1";
    case CirclePoint2:
        return "CirclePoint2";
	case PointPoint:
		return "PointPoint";
	}
	throw std::invalid_argument("Unimplemented handling of a tangent type.");
}

std::string name(Pseudotriangulation::TangentObjectType tot) {
    switch(tot) {
    case Pseudotriangulation::Circle:
        return "Circle";
    case Pseudotriangulation::CircleStraight1:
        return "CircleStraight1";
    case Pseudotriangulation::CircleStraight2:
        return "CircleStraight2";
    case Pseudotriangulation::CircleCircle1:
        return "CircleCircle1";
    case Pseudotriangulation::CircleCircle2:
        return "CircleCircle2";
    case Pseudotriangulation::IncidentStraights:
        return "IncidentStraights";
    }
    throw std::invalid_argument("Unimplemented handling of a tangent object type.");
}

bool Pseudotriangulation::usesTangent(const Pseudotriangulation::Certificate& certificate, const std::shared_ptr<Pseudotriangulation::Tangent>& t) {
    return std::visit([&t](const auto& c) {
        return c.usesTangent(t);
    }, certificate);
}

bool Pseudotriangulation::usesTangentObject(const Pseudotriangulation::Certificate& certificate, const std::shared_ptr<Pseudotriangulation::TangentObject>& tObj) {
    return std::visit([&tObj](const auto& c) {
        return c.usesTangentObject(tObj);
    }, certificate);
}

void Pseudotriangulation::removeTangentObject(std::shared_ptr<TangentObject> tObj) {
    std::cout << "[fix " << m_k << "] Removing tangent object " << *tObj << std::endl;
    m_tangentObjects.erase(std::remove(m_tangentObjects.begin(), m_tangentObjects.end(), tObj), m_tangentObjects.end());
    m_certificates.erase(std::remove_if(m_certificates.begin(), m_certificates.end(), [&](const auto& c) {
        return usesTangentObject(c, tObj);
    }), m_certificates.end());
    tObj->deleted = true;
}

void Pseudotriangulation::removeTangent(std::shared_ptr<Tangent> t) {
	std::cout << "[fix " << m_k << "] Removing tangent " << *t << std::endl;

	auto it = std::remove(m_tangents.begin(), m_tangents.end(), t);
	if (it == m_tangents.end()) {
		std::cerr << "[fix " << m_k << "] Could not find tangent to delete" << std::endl;
	}
	m_tangents.erase(it, m_tangents.end());
	auto itC = std::remove_if(m_certificates.begin(), m_certificates.end(), [&t](const Certificate& c) {
        return usesTangent(c, t);
	});
	m_certificates.erase(itC, m_certificates.end());

	auto& tsS = m_pointIdToTangents[t->source->pointId];
	tsS.erase(std::remove(tsS.begin(), tsS.end(), t), tsS.end());
	auto& tsT = m_pointIdToTangents[t->target->pointId];
	tsT.erase(std::remove(tsT.begin(), tsT.end(), t), tsT.end());
}

TangentType tangentType(CGAL::Orientation or1, CGAL::Orientation or2, bool outerElbowPoint1, bool outerElbowPoint2) {
    if (outerElbowPoint1) {
        or1 = CGAL::COLLINEAR;
    }
    if (outerElbowPoint2) {
        or2 = CGAL::COLLINEAR;
    }
	if (or1 == CGAL::COUNTERCLOCKWISE && or2 == CGAL::CLOCKWISE) {
		return Outer1;
	}
	if (or1 == CGAL::CLOCKWISE && or2 == CGAL::COUNTERCLOCKWISE) {
		return Outer2;
	}
	if (or1 == CGAL::COUNTERCLOCKWISE && or2 == CGAL::COUNTERCLOCKWISE) {
		return Inner1;
	}
	if (or1 == CGAL::CLOCKWISE && or2 == CGAL::CLOCKWISE) {
		return Inner2;
	}
	if (or1 == CGAL::COLLINEAR && or2 == CGAL::CLOCKWISE) {
		return PointCircle1;
	}
	if (or1 == CGAL::COLLINEAR && or2 == CGAL::COUNTERCLOCKWISE) {
		return PointCircle2;
	}
    if (or1 == CGAL::CLOCKWISE && or2 == CGAL::COLLINEAR) {
        return CirclePoint2;
    }
    if (or1 == CGAL::COUNTERCLOCKWISE && or2 == CGAL::COLLINEAR) {
        return CirclePoint1;
    }
	if (or1 == CGAL::COLLINEAR && or2 == CGAL::COLLINEAR) {
		return PointPoint;
	}
	throw std::invalid_argument("Unexpected case.");
}

void Pseudotriangulation::maybeAddCertificate(PointId pId, const std::shared_ptr<Tangent>& t1, const std::shared_ptr<Tangent>& t2, const State& state) {
	m_certificates.emplace_back(ConsecutiveCertificate(pId, t1, t2));
}

Pseudotriangulation::TangentCirculator Pseudotriangulation::tangentsCirculator(PointId pId) {
    auto& ts = m_pointIdToTangents[pId];
    TangentCirculator circ(&ts);
    return circ;
}

Pseudotriangulation::TangentCirculator Pseudotriangulation::tangentCirculator(PointId pId, const std::shared_ptr<Tangent>& t) {
	auto& ts = m_pointIdToTangents[pId];
	auto tIt = std::find(ts.begin(), ts.end(), t);
    assert(tIt != ts.end());
	TangentCirculator circ(&ts, tIt);
	return circ;
}

std::shared_ptr<Pseudotriangulation::Tangent> Pseudotriangulation::previousTangent(PointId pId, const std::shared_ptr<Tangent>& t) {
    return *(--tangentCirculator(pId, t));
}

std::shared_ptr<Pseudotriangulation::Tangent> Pseudotriangulation::nextTangent(PointId pId, const std::shared_ptr<Tangent>& t) {
	return *(++tangentCirculator(pId, t));
}

std::pair<std::shared_ptr<Pseudotriangulation::Tangent>, std::shared_ptr<Pseudotriangulation::Tangent>> Pseudotriangulation::neighbouringTangents(PointId pId, const std::shared_ptr<Tangent>& t) {
    auto circ = tangentCirculator(pId, t);
    auto prev = circ;
    --prev;
    auto next = circ;
    ++next;
    return {*prev, *next};
}

// Returns the other edge of the straight if t is an edge of a straight, otherwise return nullopt.
std::optional<std::shared_ptr<Pseudotriangulation::Tangent>> Pseudotriangulation::otherEdgeOfStraight(const std::shared_ptr<Tangent>& t) {
    if (!t->source->straightId.has_value() || !t->target->straightId.has_value()) return std::nullopt;
    auto pId = t->source->pointId;
    auto sourceTangentCirc = tangentCirculator(pId, t);
    auto prev = sourceTangentCirc;
    --prev;
    auto next = sourceTangentCirc;
    ++next;

    auto prevT = *prev;
    auto nextT = *next;

    std::cout << "[otherEdgeOfStraight] t: " << *t << " prev: " << *prevT << " next: " << *nextT << std::endl;

    for (const auto& otherT : {prevT, nextT}) {
        if (otherT->type == PointPoint && otherT->otherEndpoint(pId)->pointId == t->otherEndpoint(pId)->pointId &&
            otherT->endpoint(pId)->straightId == t->straight() && otherT->otherEndpoint(pId) != t->otherEndpoint(pId) &&
            t->endpoint(pId)->type != otherT->endpoint(pId)->type) {
            return otherT;
        }
    }
    return std::nullopt;
}

std::shared_ptr<Pseudotriangulation::TangentObject> Pseudotriangulation::circleTangentObject(PointId pointId) const {
	return *std::find_if(m_tangentObjects.begin(), m_tangentObjects.end(), [pointId](const std::shared_ptr<TangentObject>& tObj) {
		return tObj->type == Circle && tObj->pointId == pointId;
	});
}

void Pseudotriangulation::addTangent(const std::shared_ptr<Tangent>& t) {
    std::cout << "[fix " << m_k << "] Adding tangent " << *t << std::endl;
    m_tangents.push_back(t);
    m_certificates.push_front(ExistenceCertificate(t));
	if (t->edgeOfStraight) return;
	for (auto& endpoint : {t->source, t->target}) {
		if (endpoint->type != Circle && endpoint->type != IncidentStraights) {
			m_certificates.push_back(PointCertificate(endpoint->pointId, t));
		}
	}
}

void Pseudotriangulation::addTangentObject(const std::shared_ptr<TangentObject>& tObj) {
    std::cout << "[fix " << m_k << "] Adding tangent object " << *tObj << std::endl;
    m_tangentObjects.push_back(tObj);
    if (tObj->type == IncidentStraights) {
        m_certificates.push_front(IncidentStraightsOutsideCircleCertificate(tObj));
    }
}

void Pseudotriangulation::fix(Certificate& certificate, State& state, const Settings& settings, Pseudotriangulations& pts) {
    std::visit([&](auto& c) { this->fix(c, state, settings, pts); }, certificate);
}

void Pseudotriangulation::fix(PointCertificate& certificate, State& state, const Settings& settings, Pseudotriangulations& pts) {
    auto& t = certificate.t;
    auto& pId = certificate.pointId;
    std::cout << "[fix " << m_k << "] Fixing point certificate " << certificate << std::endl;
    auto obj = t->endpoint(pId);
    auto other = t->otherEndpoint(pId);

    assert(obj->type == CircleStraight1 || obj->type == CircleStraight2);

    auto thisOri = obj->type == CircleStraight1 ? CGAL::CLOCKWISE : CGAL::COUNTERCLOCKWISE;
    CGAL::Orientation otherOri = orientation(t, other->pointId, state);
    std::cout << "[fix " << m_k << "] thisOri: " << thisOri << "; otherOri: " << otherOri << std::endl;
    std::cout << "[fix " << m_k << "] Changing endpoint " << *t->endpoint(pId) << " to " << *circleTangentObject(pId) << std::endl;
    t->endpoint(pId) = circleTangentObject(pId);
    auto sourceOri = t->endpoint(pId) == t->source ? thisOri : otherOri;
    auto targetOri = t->endpoint(pId) == t->source ? otherOri : thisOri;
    auto newType = tangentType(sourceOri, targetOri, t->source->elbowPoint(state).has_value(), t->target->elbowPoint(state).has_value());
    std::cout << "[fix " << m_k << "] Changing tangent type from " << name(t->type) << " to " << name(newType) << std::endl;
    t->type = newType;

    m_certificates.erase(std::remove_if(m_certificates.begin(), m_certificates.end(), [&](const Certificate& c) {
        if (auto pcP = std::get_if<PointCertificate>(&c)) {
            const PointCertificate& pc = *pcP;
            return pc == certificate;
        }
        return false;
    }), m_certificates.end());
}

void Pseudotriangulation::fix(ExistenceCertificate& certificate, State& state, const Settings& settings, Pseudotriangulations& pts) {
    auto t = certificate.t;

    if (t->source->type == IncidentStraights || t->target->type == IncidentStraights) {
        std::cout << "[fix " << m_k << "] Edge hits obstacle with incident edge around it." << std::endl;

        auto& shorter = t;
        auto inter = t->source->type == IncidentStraights ? t->source : t->target;
        auto& shorterOther = inter == t->source ? t->target : t->source;
        assert(shorterOther->circleStraight());
        auto& s1 = *shorterOther->straightId;
        assert(*inter->straightId == s1 || *inter->otherStraightId == s1);
        auto& s2 = *inter->straightId == s1 ? *inter->otherStraightId : *inter->straightId;

        auto circ = tangentCirculator(inter->pointId, t);
        if (shorterOther->type == CircleStraight1) { // or equivalently: if (inter->straightId == shorterOther)
            // Go in ccw order around inter->pointId
            ++circ;
        } else {
            assert(shorterOther->type == CircleStraight2);
            // Go in cw order around inter->pointId
            --circ;
        }

        auto cand = *circ;

        std::shared_ptr<Tangent> longer = nullptr;
        while (longer == nullptr) {
            if (cand->endpoint(inter->pointId) == inter && cand->otherEndpoint(inter->pointId)->circleStraight() &&
                *cand->otherEndpoint(inter->pointId)->straightId == s2) {
                longer = cand;
            } else {
                basicAngleZero(inter->pointId, shorter, cand, state);

                circ = tangentCirculator(inter->pointId, t);
                if (shorterOther->type == CircleStraight1) { // or equivalently: if (inter->straightId == shorterOther)
                    // Go in ccw order around inter->pointId
                    ++circ;
                } else {
                    assert(shorterOther->type == CircleStraight2);
                    // Go in cw order around inter->pointId
                    --circ;
                }
                cand = *circ;
            }
        }

        assert(longer != nullptr);

        std::cout << "[fix " << m_k << "] Splitting straight; longer is " << *longer << " and shorter is " << *shorter << std::endl;
        auto maybeOtherEdge = otherEdgeOfStraight(longer);

        if (!maybeOtherEdge.has_value()) {
            throw std::runtime_error("Expected tangent to be part of a straight.");
        }
        removeTangentObject(shorterOther);
        removeTangentObject(inter);
        splitStraight(state, settings, longer, shorter, *maybeOtherEdge, inter->pointId, s2, pts);
        return;
    } else {
        std::cout << "[fix " << m_k << "] Two circles collided." << std::endl;
    }
}

void Pseudotriangulation::fix(IncidentStraightsOutsideCircleCertificate& certificate, State& state, const Settings& settings, Pseudotriangulations& pts) {
    std::cout << "[fix " << m_k << "] Incident straights tangent object has entered circle." << std::endl;

    auto& tObj = certificate.tObj;
    auto pId = tObj->pointId;

    auto circ = tangentsCirculator(pId);
    // We assume there is at least one tangent that is incident to tObj and to some object other than tObj
    // Furthermore, the tangents that are incident to tObj form a contiguous sequence.
    // This holds for a valid pseudotriangulation.

    auto curr = circ;
    while ((*curr)->incidentTo(tObj)) {
        ++curr;
    }
    while (!(*curr)->incidentTo(tObj)) {
        ++curr;
    }
    // Now start is the first tangent of the contiguous sequence of tangents that are incident to tObj.
    // It should be the edge of a straight.
    assert((*curr)->edgeOfStraight);
    auto s1 = (*curr)->straight();
    auto tObj1 = std::make_shared<TangentObject>(pId, s1, false);
    addTangentObject(tObj1);
    (*curr)->endpoint(pId) = tObj1;
    ++curr;
    while (!(*curr)->edgeOfStraight) {
        (*curr)->endpoint(pId) = tObj1;
        ++curr;
    }
    assert((*curr)->endpoint(pId)->type == IncidentStraights);
    auto s2 = (*curr)->straight();
    auto tObj2 = std::make_shared<TangentObject>(pId, s2, true);
    addTangentObject(tObj2);
    (*curr)->endpoint(pId) = tObj2;

	auto connectee = (*curr)->otherEndpoint(pId);
	auto newTangent = std::make_shared<Tangent>(PointPoint, connectee, tObj1, false);
	if (connectee->elbowPoint(state)) {
		auto sOri = sourceOrientation(newTangent, state);
		auto tOri = targetOrientation(newTangent, state);
		newTangent->type = tangentType(sOri, tOri, false, false);
		newTangent->source = circleTangentObject(connectee->pointId);
	}
    addTangent(newTangent);
    m_pointIdToTangents[pId].insert(curr.current_iterator(), newTangent);
	auto cCirc = tangentCirculator(connectee->pointId, *curr);
	auto cPrev = *cCirc;
	++cCirc;
	auto cNext = *cCirc;
	m_pointIdToTangents[connectee->pointId].insert(cCirc.current_iterator(), newTangent);
	maybeAddCertificate(connectee->pointId, cPrev, newTangent, state);
	maybeAddCertificate(connectee->pointId, newTangent, cNext, state);


	auto next = *curr;
    maybeAddCertificate(pId, newTangent, next, state);
    --curr;
    --curr;
	auto prev = *curr;
    maybeAddCertificate(pId, prev, newTangent, state);

	m_certificates.erase(std::remove_if(m_certificates.begin(), m_certificates.end(), [&](const Certificate& c) {
		if (auto ccP = std::get_if<ConsecutiveCertificate>(&c)) {
			return ccP->t1 == prev && ccP->t2 == next || ccP->t1 == cPrev && ccP->t2 == cNext;
		}
		return false;
	}), m_certificates.end());

    removeTangentObject(tObj);
}

void Pseudotriangulation::fix(InnerElbowCertificate& certificate, State& state, const Settings& settings, Pseudotriangulations& pts) {
	std::cout << "[fix " << m_k << "] Inner elbow collapses." << std::endl;

	auto pId = certificate.pointId;
	auto t1 = certificate.t1;
	auto t2 = certificate.t2;

	collapseElbow(pId, t1, t2, state, pts);
}

// Refactor to not use ConsecutiveCertificate
void Pseudotriangulation::addAndRemove(PointId pId0, const std::shared_ptr<Tangent>& t1, const std::shared_ptr<Tangent>& t2, const State& state, const std::shared_ptr<Tangent>& oldTangent, const std::shared_ptr<Tangent>& newTangent) {
    // Define some standard aliases
    auto obj0 = t1->endpoint(pId0);
    auto obj1 = t1->otherEndpoint(pId0);
    auto obj2 = t2->otherEndpoint(pId0);
    auto pId1 = obj1->pointId;
    auto pId2 = obj2->pointId;
    auto& ts1 = m_pointIdToTangents[pId1];
    auto& ts2 = m_pointIdToTangents[pId2];
    bool t1R = t1->target->pointId == pId0;
    bool t2R = t2->target->pointId == pId0;
    bool angleZero = orientation(t1, t1R, state) == orientation(t2, t2R, state) ||
                     t1->endpoint(pId0)->straightId.has_value() && t1->endpoint(pId0) == t2->endpoint(pId0); // this part is probably not necessary anymore

    auto [prev0, next0] = neighbouringTangents(pId0, oldTangent);

    auto t1It = std::find(ts1.begin(), ts1.end(), t1);
    if (t2 == oldTangent && (angleZero ? orientation(t1, !t1R, state) == CGAL::CLOCKWISE : orientation(t1, t1R, state) == CGAL::COUNTERCLOCKWISE)) {
        ++t1It;
    }
    auto newIt1 = ts1.insert(t1It, newTangent);
    TangentCirculator newCirc1(&ts1, newIt1);

    auto t2It = std::find(ts2.begin(), ts2.end(), t2);
    if (t1 == oldTangent && (angleZero ? orientation(t2, !t2R, state) == CGAL::CLOCKWISE : orientation(t2, t2R, state) == CGAL::COUNTERCLOCKWISE)) {
        ++t2It;
    }
    auto newIt2 = ts2.insert(t2It, newTangent);
    TangentCirculator newCirc2(&ts2, newIt2);

    removeTangent(oldTangent);
    addTangent(newTangent);

    auto prev1 = newCirc1;
    --prev1;
    auto next1 = newCirc1;
    ++next1;

    maybeAddCertificate(pId1, *prev1, newTangent, state);
    maybeAddCertificate(pId1, newTangent, *next1, state);

    m_certificates.erase(std::remove_if(m_certificates.begin(), m_certificates.end(), [&](const Certificate& c) {
        if (auto ccP = std::get_if<ConsecutiveCertificate>(&c)) {
            return ccP->t1 == *prev1 && ccP->t2 == *next1;
        }
        return false;
    }), m_certificates.end());

    auto prev2 = newCirc2;
    --prev2;
    auto next2 = newCirc2;
    ++next2;

    maybeAddCertificate(pId2, *prev2, newTangent, state);
    maybeAddCertificate(pId2, newTangent, *next2, state);
    maybeAddCertificate(pId0, prev0, next0, state);

    m_certificates.erase(std::remove_if(m_certificates.begin(), m_certificates.end(), [&](const Certificate& c) {
        if (auto ccP = std::get_if<ConsecutiveCertificate>(&c)) {
            return ccP->t1 == *prev2 && ccP->t2 == *next2;
        }
        return false;
    }), m_certificates.end());
}

void Pseudotriangulation::snapTangentToPoint(State& state, PointId pId0, const std::shared_ptr<Tangent>& t1, const std::shared_ptr<Tangent>& t2) {
    auto fixTangentType = [&state, pId0, this](const std::shared_ptr<Tangent>& t) {
        if (t->otherEndpoint(pId0)->point()) {
            t->type = PointPoint;
        } else {
            CGAL::Orientation ori;
            if (!t->source->point()) {
                ori = sourceOrientation(t, state);
                auto tT = t->target;
                t->target = t->source;
                t->source = tT;
            } else {
                ori = targetOrientation(t, state);
            }
            assert(ori != CGAL::COLLINEAR);
            if (ori == CGAL::CLOCKWISE) {
                t->type = PointCircle1;
            } else {
                t->type = PointCircle2;
            }
        }
    };

    t1->endpoint(pId0) = t2->endpoint(pId0);
    fixTangentType(t1);
    m_certificates.push_back(PointCertificate(pId0, t1));
}

void Pseudotriangulation::handleIntersectingIncidentStraights(ConsecutiveCertificate& certificate, State& state) {
    const auto t1 = certificate.t1;
    const auto t2 = certificate.t2;
    auto pId0 = certificate.pointId;

    m_certificates.erase(std::remove_if(m_certificates.begin(), m_certificates.end(), [&certificate](const Certificate& c) {
        if (auto ccP = std::get_if<ConsecutiveCertificate>(&c)) {
            return *ccP == certificate;
        }
        return false;
    }), m_certificates.end());
    PointId thePId;
    if (t1->endpoint(pId0) != t2->endpoint(pId0) && t1->endpoint(pId0)->circleStraight() && t2->endpoint(pId0)->circleStraight()) {
        // Problem at pId0
        thePId = pId0;
        auto e1 = t1->endpoint(pId0);
        auto e2 = t2->endpoint(pId0);
        auto s1 = *e1->straightId;
        auto s2 = *e2->straightId;
        // Make sure that the first straight is first in ccw order on pId0
        auto inter = std::make_shared<TangentObject>(pId0, s1, s2);
        addTangentObject(inter);

        for (const auto& t : m_pointIdToTangents[pId0]) {
            if (t->endpoint(pId0) == e1 || t->endpoint(pId0) == e2)
                t->endpoint(pId0) = inter;
        }
        removeTangentObject(e1);
        removeTangentObject(e2);
    } else {
        // Problem at endpoint other than pId0
        auto otherId = t1->otherEndpoint(pId0)->pointId;
        thePId = otherId;
        auto e1 = t1->endpoint(otherId);
        auto e2 = t2->endpoint(otherId);
        auto s1 = *e1->straightId;
        auto s2 = *e2->straightId;
        // Make sure that the first straight is first in ccw order on otherId
        auto inter = std::make_shared<TangentObject>(otherId, s2, s1);
        addTangentObject(inter);

        t1->endpoint(otherId) = inter;
        t2->endpoint(otherId) = inter;

        for (const auto& t : m_pointIdToTangents[otherId]) {
            if (t->endpoint(otherId) == e1 || t->endpoint(otherId) == e2)
                t->endpoint(otherId) = inter;
        }

        // Remove old tangent objects
        removeTangentObject(e1);
        removeTangentObject(e2);
    }

    // If after the update the two tangents are identical, then remove one.
    if (t1->endpoint(thePId) == t2->endpoint(thePId) &&
        (t1->otherEndpoint(thePId) == t2->otherEndpoint(thePId) ||
         t1->otherEndpoint(thePId)->elbowPoint(state) && t2->otherEndpoint(thePId) == circleTangentObject(t1->otherEndpoint(thePId)->pointId) ||
         t2->otherEndpoint(thePId)->elbowPoint(state) && t1->otherEndpoint(thePId) == circleTangentObject(t2->otherEndpoint(thePId)->pointId))) {
        if (!t1->edgeOfStraight) {
            auto [prev, next] = neighbouringTangents(thePId, t1);
            removeTangent(t1);
            maybeAddCertificate(thePId, prev, next, state);
            return;
        }
        if (!t2->edgeOfStraight) {
            auto [prev, next] = neighbouringTangents(thePId, t2);
            removeTangent(t2);
            maybeAddCertificate(thePId, prev, next, state);
            return;
        }
    }
}

std::pair<std::shared_ptr<Pseudotriangulation::Tangent>, std::shared_ptr<Pseudotriangulation::Tangent>>
Pseudotriangulation::elbowTangents(PointId pId, const State& state) {
	const auto& ts = m_pointIdToTangents[pId];

	std::shared_ptr<Tangent> elbow1;
	std::shared_ptr<Tangent> elbow2;
	for (const auto& t : ts) {
		auto& tObj = t->endpoint(pId);
		if (tObj->elbowPoint(state)) {
			if (tObj->type == CircleStraight1) {
				elbow1 = t;
			} else {
				assert(tObj->type == CircleStraight2);
				elbow2 = t;
			}
		}
	}
	assert(elbow1 != nullptr && elbow2 != nullptr);
	return {elbow1, elbow2};
}

void Pseudotriangulation::insertTangentAndAddCertificates(PointId pId, TangentCirculator pos, const std::shared_ptr<Tangent>& newTangent, const State& state) {
	auto& ts = m_pointIdToTangents[pId];
	auto next = *pos;
	auto prevCirc = pos;
	--prevCirc;
	auto prev = *prevCirc;
	auto it = ts.insert(pos.current_iterator(), newTangent);
	m_certificates.erase(std::remove_if(m_certificates.begin(), m_certificates.end(), [&](const Certificate& c) {
		 if (auto ccP = std::get_if<ConsecutiveCertificate>(&c)) {
			 return ccP->t1 == prev && ccP->t2 == next;
		 }
		 return false;
	}), m_certificates.end());
	maybeAddCertificate(pId, prev, newTangent, state);
	maybeAddCertificate(pId, newTangent, next, state);
}

void Pseudotriangulation::removeTangentAndAddCertificates(std::shared_ptr<Tangent> t) {
	std::cout << "[fix " << m_k << "] Removing tangent " << *t << std::endl;

	auto it = std::remove(m_tangents.begin(), m_tangents.end(), t);
	if (it == m_tangents.end()) {
		std::cerr << "[fix " << m_k << "] Could not find tangent to delete" << std::endl;
	}
	m_tangents.erase(it, m_tangents.end());
	auto itC = std::remove_if(m_certificates.begin(), m_certificates.end(), [&t](const Certificate& c) {
		return usesTangent(c, t);
	});
	m_certificates.erase(itC, m_certificates.end());

	auto handleDeletion = [&](PointId pId) {
		auto& ts = m_pointIdToTangents[pId];
		auto circ = tangentCirculator(pId, t);
		auto prev = circ;
		--prev;
		auto next = circ;
		++next;
		ts.erase(circ.current_iterator());
		m_certificates.push_back(ConsecutiveCertificate(pId, *prev, *next));
	};
	handleDeletion(t->source->pointId);
	handleDeletion(t->target->pointId);
}

void Pseudotriangulation::splitStraight(State& state, const Settings& settings, const std::shared_ptr<Tangent>& longer, const std::shared_ptr<Tangent>& shorter, const std::shared_ptr<Tangent>& otherEdge, PointId pId0, StraightId oldStraight, Pseudotriangulations& pts) {
    //      shorter
    // obj0 ------- obstaclePId      nonObstaclePId
    //   |----------------------------------|
    //                  longer

    // Make orbit
    auto& [edge, orbitAfterIt] = oldStraight;
    auto obstaclePId = shorter->otherEndpoint(pId0)->pointId;
    auto nonObstaclePId = longer->otherEndpoint(pId0)->pointId;
    auto currentObstacleOrbits = state.pointIdToElbows[obstaclePId];
    Orbit newOrbit;
    newOrbit.pointId = obstaclePId;
    
	std::optional<std::pair<std::shared_ptr<Tangent>, std::shared_ptr<Tangent>>> oldElbowTangents = std::nullopt;
	std::optional<std::pair<CGAL::Orientation, CGAL::Orientation>> oldElbowOrientations = std::nullopt;
	std::optional<ElbowId> oldElbow = currentObstacleOrbits.empty() ? std::nullopt : std::optional(currentObstacleOrbits.back()); // stored here for later
    std::optional<int> oldElbowCat = std::nullopt;
    if (oldElbow) {
        oldElbowCat = state.pointIdToCat.at(oldElbow->first.first);
        auto& specialPt = pts.c[*oldElbowCat];
        oldElbowTangents = specialPt.elbowTangents(obstaclePId, state);
		oldElbowOrientations = std::pair(specialPt.orientation(oldElbowTangents->first, obstaclePId, state), specialPt.orientation(oldElbowTangents->second, obstaclePId, state));

		newOrbit.innerRadius = oldElbow->second->outerRadius;
		newOrbit.outerRadius = newOrbit.innerRadius + settings.edgeWidth;
    } else {
		newOrbit.innerRadius = settings.kelpRadius;
		newOrbit.outerRadius = settings.kelpRadius + settings.edgeWidth;
    }

    // Store source and target pointIds of straight before updating state
    auto [sId, tId] = state.straightEndpoints(oldStraight);
    newOrbit.dir = longer->endpoint(sId)->effectiveCircleStraightType(oldStraight) == CircleStraight2 ? CGAL::COUNTERCLOCKWISE : CGAL::CLOCKWISE;

    // Add orbit to edge
    auto newOrbitIt = state.edgeTopology[edge].orbits.insert(orbitAfterIt, newOrbit);
    StraightId newStraightId(edge, newOrbitIt);

    // Add elbow id to point
    state.pointIdToElbows[obstaclePId].emplace_back(edge, newOrbitIt);

    std::shared_ptr<Tangent> l1, l2, o1, o2 = nullptr;

    // Create new tangent objects
    std::vector<std::tuple<PointId, std::list<std::shared_ptr<Tangent>>::const_iterator, std::shared_ptr<Tangent>>> positionedNewTangents;
    for (const auto& straightId : {oldStraight, newStraightId}) {
        for (const auto& one : {false, true}) {
            auto [sourceId, targetId] = state.straightEndpoints(straightId);
            auto otherId = sourceId == obstaclePId ? targetId : sourceId;

            auto leoType = longer->endpoint(otherId)->effectiveCircleStraightType(oldStraight);
            auto connectee = one ? (leoType == CircleStraight2 ? longer->endpoint(otherId) : otherEdge->endpoint(otherId))
                                 : (leoType == CircleStraight1 ? longer->endpoint(otherId) : otherEdge->endpoint(otherId));
//            std::cout << (straightId == oldStraight ? "old straight" : "new straight") << " one: " << one << " leo type: " << leoType << " connectee type: " << connectee->effectiveCircleStraightType(oldStraight) << std::endl;
            if (connectee->deleted) {
                continue;
            }
//            if (one && connectee->effectiveCircleStraightType(oldStraight) != CircleStraight2 || !one && connectee->effectiveCircleStraightType(oldStraight) == CircleStraight2) {
//                continue;
//            }
            auto tObj = std::make_shared<TangentObject>(obstaclePId, straightId, one);
            addTangentObject(tObj);

            auto straightEdge = std::make_shared<Tangent>(PointPoint, tObj, connectee, true);
            addTangent(straightEdge);

            if (connectee == longer->endpoint(otherId) && one) {
                l1 = straightEdge;
            } else if (connectee == longer->endpoint(otherId) && !one) {
                l2 = straightEdge;
            } else if (connectee == otherEdge->endpoint(otherId) && one) {
                o1 = straightEdge;
            } else {
                assert(connectee == otherEdge->endpoint(otherId) && !one);
                o2 = straightEdge;
            }

            auto otherOtherId = longer->otherEndpoint(otherId)->pointId;
            // Add tangents to endpoint objects of oldStraight
            auto& ts = m_pointIdToTangents[otherId];
//            std::cout << "[debugging] Looking for tangent with " << *connectee << " at point " << otherId << " and other point being " << otherOtherId << std::endl;
            auto it = std::find_if(ts.begin(), ts.end(), [connectee, otherId, otherOtherId, this](const std::shared_ptr<Tangent>& t) {
//                std::cout << "[debugging] Checking tangent: " << *t << std::endl;
                return t->endpoint(otherId) == connectee && t->otherEndpoint(otherId)->pointId == otherOtherId && t->edgeOfStraight;
            });
            if (it == ts.end()) {
                std::cerr << "Could not find tangent in tangent list!" << std::endl;
            }
            positionedNewTangents.emplace_back(otherId, it, straightEdge);
        }
    }

    for (const auto& [id, it, straightEdge] : positionedNewTangents) {
        m_pointIdToTangents[id].insert(it, straightEdge);
    }

    // Add tangents to obstacle object.
    auto& obstacleTs = m_pointIdToTangents[obstaclePId];
    auto it = std::find(obstacleTs.begin(), obstacleTs.end(), shorter);
    std::list<std::shared_ptr<Tangent>>::iterator l1It;
    if (l1 != nullptr) {
        l1It = obstacleTs.insert(it, l1);
    }
    auto o2It = obstacleTs.insert(it, o2);
    auto o1It = obstacleTs.insert(it, o1);
    std::list<std::shared_ptr<Tangent>>::iterator l2It;
    if (l2 != nullptr) {
        l2It = obstacleTs.insert(it, l2);
    }

    // Update tangent objects
    if (longer->endpoint(sId)->straightId == oldStraight) {
        longer->endpoint(sId)->straightId = newStraightId;
    } else {
        assert(longer->endpoint(sId)->otherStraightId == oldStraight);
        longer->endpoint(sId)->otherStraightId = newStraightId;
    }
    if (otherEdge->endpoint(sId)->straightId == oldStraight) {
        otherEdge->endpoint(sId)->straightId = newStraightId;
    } else {
        assert(otherEdge->endpoint(sId)->otherStraightId == oldStraight);
        otherEdge->endpoint(sId)->otherStraightId = newStraightId;
    }

    // Remove tangents
    removeTangent(otherEdge);
    removeTangent(longer);

    auto [prevT, nextT] = neighbouringTangents(pId0, shorter);

    // Remove last tangent
    removeTangent(shorter);

    // Update certificates
    if (l1 != nullptr) {
        TangentCirculator l1Circ(&obstacleTs, l1It);
        auto l1CircPrev = l1Circ;
        --l1CircPrev;
        if (*l1CircPrev == shorter) {
            --l1CircPrev;
        }
        maybeAddCertificate(obstaclePId, *l1CircPrev, l1, state);
        maybeAddCertificate(obstaclePId, l1, o2, state);
    } else {
        TangentCirculator o2Circ(&obstacleTs, o2It);
        auto o2CircPrev = o2Circ;
        --o2CircPrev;
        if (*o2CircPrev == shorter) {
            --o2CircPrev; // todo check
        }
        maybeAddCertificate(obstaclePId, *o2CircPrev, o2, state);
    }
    maybeAddCertificate(obstaclePId, o2, o1, state);
    if (l2 != nullptr) {
        maybeAddCertificate(obstaclePId, o1, l2, state);
        TangentCirculator l2Circ(&obstacleTs, l2It);
        auto l2CircNext = l2Circ;
        ++l2CircNext;
        if (*l2CircNext == shorter) {
            ++l2CircNext;
        }
        maybeAddCertificate(obstaclePId, l2, *l2CircNext, state);
    } else {
        TangentCirculator o1Circ(&obstacleTs, o1It);
        auto o1CircNext = o1Circ;
        ++o1CircNext;
        if (*o1CircNext == shorter) {
            ++o1CircNext; // todo check
        }
        maybeAddCertificate(obstaclePId, o1, *o1CircNext, state);
    }
    maybeAddCertificate(pId0, prevT, nextT, state);

    auto somePId1 = o2->otherEndpoint(obstaclePId)->pointId;
    auto o2Circ = tangentCirculator(somePId1, o2);
    --o2Circ;
    maybeAddCertificate(somePId1, *o2Circ, o2, state);
    if (l1 != nullptr) {
        auto l1Circ2 = tangentCirculator(somePId1, l1);
        ++l1Circ2;
        maybeAddCertificate(somePId1, l1, *l1Circ2, state);
    }
    auto somePId2 = o1->otherEndpoint(obstaclePId)->pointId;
    if (l2 != nullptr) {
        auto l2Circ2 = tangentCirculator(somePId2, l2);
        --l2Circ2;
        maybeAddCertificate(somePId2, *l2Circ2, l2, state);
    }
    auto o1Circ = tangentCirculator(somePId2, o1);
    ++o1Circ;
    maybeAddCertificate(somePId2, o1, *o1Circ, state);

	// if point already had an orbit, create two new tangents near outer CircleStraights of what was previously the outer elbow.
	if (!oldElbow) return;
    pts.c[*oldElbowCat].postSplitStraight(obstaclePId, *oldElbowTangents, *oldElbowOrientations, state);
}

void
Pseudotriangulation::postSplitStraight(PointId obstaclePId,
                                       std::pair<std::shared_ptr<Tangent>, std::shared_ptr<Tangent>> oldElbowTangents,
                                       std::pair<CGAL::Orientation, CGAL::Orientation> oldElbowOrientations,
                                       State& state) {
    auto createNewTangents = [this, obstaclePId, &state](const std::shared_ptr<Tangent>& ebt, CGAL::Orientation ori) {
        auto other = ebt->otherEndpoint(obstaclePId);
        std::shared_ptr<Tangent> newTangent;
        if (!other->elbowPoint(state)) {
            auto type = ori == CGAL::CLOCKWISE ? CirclePoint2 : CirclePoint1;
            newTangent = std::make_shared<Tangent>(type, circleTangentObject(obstaclePId), other, false);
        } else {
            auto sourceOri = ori;
            auto targetOri = orientation(ebt, other->pointId, state);
            auto type = tangentType(sourceOri, targetOri, false, false);
            newTangent = std::make_shared<Tangent>(type, circleTangentObject(obstaclePId), circleTangentObject(other->pointId), false);
        }
        addTangent(newTangent);
        auto circ = tangentCirculator(other->pointId, ebt);
        if (ori == CGAL::COUNTERCLOCKWISE) {
            ++circ;
        }
        insertTangentAndAddCertificates(other->pointId, circ, newTangent, state);
        auto obstacleCirc1 = tangentCirculator(obstaclePId, ebt);
        if (ori == CGAL::CLOCKWISE) {
            ++obstacleCirc1;
        }
        insertTangentAndAddCertificates(obstaclePId, obstacleCirc1, newTangent, state);
    };

    std::cout << "[fix " << m_k << "] Post fix straight" << std::endl;
    auto [ebt1, ebt2] = std::move(oldElbowTangents);
    auto [ori1, ori2] = oldElbowOrientations;
    auto exists1 = std::find(m_pointIdToTangents[obstaclePId].begin(), m_pointIdToTangents[obstaclePId].end(), ebt1) != m_pointIdToTangents[obstaclePId].end();
    auto exists2 = std::find(m_pointIdToTangents[obstaclePId].begin(), m_pointIdToTangents[obstaclePId].end(), ebt2) != m_pointIdToTangents[obstaclePId].end();
    if (exists1)
        createNewTangents(ebt1, ori1);
    if (exists2)
        createNewTangents(ebt2, ori2);

    if (exists1 && exists2) {
        m_certificates.push_back(InnerElbowCertificate(obstaclePId, ebt2, ebt1));
    }
}

void Pseudotriangulation::collapseElbow(PointId pId0, std::shared_ptr<Tangent> t1, std::shared_ptr<Tangent> t2, State& state, Pseudotriangulations& pts) {
    auto obj0 = t1->endpoint(pId0);
    auto obj1 = t1->otherEndpoint(pId0);
    auto obj2 = t2->otherEndpoint(pId0);
    auto pId1 = obj1->pointId;
    auto pId2 = obj2->pointId;
    auto& ts0 = m_pointIdToTangents[pId0];
    auto& ts1 = m_pointIdToTangents[pId1];
    auto& ts2 = m_pointIdToTangents[pId2];

    auto elbowObj1 = t1->endpoint(pId0);
    auto elbowObj2 = t2->endpoint(pId0);

    auto s1 = elbowObj1->straightId;
    auto s2 = elbowObj2->straightId;
    auto edge = s1->first;

    auto orbit = *t1->endpoint(pId0)->elbowOrbit(state);

    // Other edge of straight s1
    auto other1 = otherEdgeOfStraight(t1);
    // Other edge of straight s2
    auto other2 = otherEdgeOfStraight(t2);

    auto makeOther = [&](bool one) {
        auto beforeLastIt = state.pointIdToElbows[pId0].end();
        --beforeLastIt;
        --beforeLastIt;
        auto beforeLast = *beforeLastIt;
        auto [s3, s4] = state.incidentStraights(beforeLast);
        auto [s3s, s3t] = state.straightEndpoints(s3);
        auto [s4s, s4t] = state.straightEndpoints(s4);
        StraightId interS;
        if (s3s == (one ? pId1 : pId2)) {
            interS = s3;
        } else {
            assert(s4t == (one ? pId1 : pId2));
            interS = s4;
        }

        auto inter = std::make_shared<TangentObject>((one ? pId1 : pId2), *s1, interS);
        // Create (or get) object at pId0
        auto& elbowObj = one ? elbowObj1 : elbowObj2;
        assert(elbowObj->circleStraight());
        auto elbowObjNew = std::make_shared<TangentObject>(pId0, *s1, elbowObj->type == CircleStraight1 ? false : true);
        if (one) {
            other1 = std::make_shared<Tangent>(PointPoint, inter, elbowObjNew, true);
        } else {
            other2 = std::make_shared<Tangent>(PointPoint, inter, elbowObjNew, true);
        }
        addTangentObject(elbowObjNew);
        addTangentObject(inter);
    };

    // If other1 or other2 does not exist, create a new tangent
    assert(other1.has_value() || other2.has_value());

    bool madeOther1 = false;
    if (!other1.has_value()) {
        makeOther(true);
        madeOther1 = true;
    }
    bool madeOther2 = false;
    if (!other2.has_value()) {
        makeOther(false);
        madeOther2 = true;
    }

    assert(other1.has_value() && other2.has_value());

    auto elbowObj3 = (*other1)->endpoint(pId0);
    auto elbowObj4 = (*other2)->endpoint(pId0);

    auto csObj1 = t1->otherEndpoint(pId0);
    auto csObj2 = (*other1)->otherEndpoint(pId0);
    auto csObj3 = t2->otherEndpoint(pId0);
    auto csObj4 = (*other2)->otherEndpoint(pId0);

    bool issue12 = *csObj1->straightId->second == orbit;
    std::list<Orbit>::const_iterator oldOrbitIt;
    ElbowId oldElbowId;
    if (issue12) {
        auto& correct = t2->otherEndpoint(pId0)->straightId->second;
        auto oldStraight = t1->otherEndpoint(pId0)->straightId;
        oldOrbitIt = oldStraight->second;
        oldElbowId = *oldStraight;
        t1->otherEndpoint(pId0)->straightId->second = correct;

        if (csObj2->straightId == oldStraight) {
            csObj2->straightId->second = correct;
        } else {
            assert(csObj2->type == IncidentStraights && csObj2->otherStraightId == oldStraight);
            csObj2->otherStraightId->second = correct;
        }
    } else {
        auto& correct = t1->otherEndpoint(pId0)->straightId->second;
        auto oldStraight = t2->otherEndpoint(pId0)->straightId;
        oldOrbitIt = oldStraight->second;
        oldElbowId = *oldStraight;
        t2->otherEndpoint(pId0)->straightId->second = correct;

        if (csObj4->straightId == oldStraight) {
            csObj4->straightId->second = correct;
        } else {
            assert(csObj4->type == IncidentStraights && csObj4->otherStraightId == oldStraight);
            csObj4->otherStraightId->second = correct;
        }
    }

    bool removingInnerOrbit = false;
    // Remove elbow/orbit
    auto oldElbowIt = std::find(state.pointIdToElbows[pId0].begin(), state.pointIdToElbows[pId0].end(), oldElbowId);
    auto modOrbitIt = oldElbowIt;
    auto oldWidth = oldOrbitIt->outerRadius - oldOrbitIt->innerRadius;
    while (++modOrbitIt != state.pointIdToElbows[pId0].end()) {
        modOrbitIt->second->innerRadius -= oldWidth;
        modOrbitIt->second->outerRadius -= oldWidth;
        removingInnerOrbit = true;
    }
    state.pointIdToElbows[pId0].erase(oldElbowIt);
    state.edgeTopology[edge].orbits.erase(oldOrbitIt);

	// create new tangent
	auto straightEdge = std::make_shared<Tangent>(PointPoint, t1->otherEndpoint(pId0), t2->otherEndpoint(pId0), true);
	auto otherStraightEdge = std::make_shared<Tangent>(PointPoint, (*other1)->otherEndpoint(pId0), (*other2)->otherEndpoint(pId0), true);

    // Remove tangent objects at pId0
    m_tangentObjects.erase(std::remove_if(m_tangentObjects.begin(), m_tangentObjects.end(), [&](const std::shared_ptr<TangentObject>& obj) {
        return obj == elbowObj1 || obj == elbowObj2 || obj == elbowObj3 || obj == elbowObj4;
    }), m_tangentObjects.end());

    auto it0 = std::find(ts0.begin(), ts0.end(), t1);
    auto it1 = std::find(ts1.begin(), ts1.end(), t1);
    auto it2 = std::find(ts2.begin(), ts2.end(), t2);

    std::shared_ptr<Tangent> newTangent;
    if (!madeOther1 && !madeOther2) {
        newTangent = std::make_shared<Tangent>(csObj2->type == CircleStraight1 ? PointCircle2 : PointCircle1, csObj2, circleTangentObject(pId0));
    } else if (madeOther1) {
        // By construction this is the straight we need
        auto inter = (*other1)->endpoint(pId1);
        auto s = *inter->otherStraightId;
        assert((*other1)->endpoint(pId0)->circleStraight());
        auto one = (*other1)->endpoint(pId0)->type == CircleStraight2;
        auto newObj = std::make_shared<TangentObject>(pId0, s, one);
        newTangent = std::make_shared<Tangent>(PointPoint, inter, newObj, true);
        addTangentObject(newObj);
    } else {
        assert(madeOther2);

        // By construction this is the straight we need
        auto inter = (*other2)->endpoint(pId2);
        auto s = *inter->otherStraightId;
        assert((*other2)->endpoint(pId0)->circleStraight());
        auto one = (*other2)->endpoint(pId0)->type == CircleStraight2;
        auto newObj = std::make_shared<TangentObject>(pId0, s, one);
        newTangent = std::make_shared<Tangent>(PointPoint, inter, newObj, true);
        addTangentObject(newObj);
    }

    addTangent(straightEdge);
    addTangent(otherStraightEdge);
    addTangent(newTangent);

    TangentCirculator start1, end1, start2, end2;

    auto [t1Prev, t1Next] = neighbouringTangents(pId0, t1);
    auto [t2Prev, t2Next] = neighbouringTangents(pId0, t2);
    auto itNew0 = ts0.insert(it0, newTangent);
    if (!madeOther1 && (
        t1Prev == other1 && (*other1)->otherEndpoint(pId0)->type == CircleStraight2 ||
        t1Next == other1 && (*other1)->otherEndpoint(pId0)->type == CircleStraight1
        ) || madeOther1 && (
        t2Prev == other2 && (*other2)->otherEndpoint(pId0)->type == CircleStraight2 ||
        t2Next == other2 && (*other2)->otherEndpoint(pId0)->type == CircleStraight1
        )) {
        if (newTangent->otherEndpoint(pId0)->pointId == pId2) {
            auto it = ts2.insert(it2, newTangent);
            end2 = TangentCirculator(&ts2, it);
            ts2.insert(it2, otherStraightEdge);
        } else {
            auto it = ts2.insert(it2, otherStraightEdge);
            end2 = TangentCirculator(&ts2, it);
        }
        end1 = TangentCirculator(&ts1, ts1.insert(it1, straightEdge));
        start1 = TangentCirculator(&ts1, ts1.insert(it1, otherStraightEdge));
        start2 = TangentCirculator(&ts2, ts2.insert(it2, straightEdge));
        if (newTangent->otherEndpoint(pId0)->pointId == pId1) {
            start1 = TangentCirculator(&ts1, ts1.insert(it1, newTangent));
        }
    } else {
        if (newTangent->otherEndpoint(pId0)->pointId == pId1) {
            end1 = TangentCirculator(&ts1, ts1.insert(it1, newTangent));
            ts1.insert(it1, otherStraightEdge);
        } else {
            end1 = TangentCirculator(&ts1, ts1.insert(it1, otherStraightEdge));
        }
        start1 = TangentCirculator(&ts1, ts1.insert(it1, straightEdge));
        end2 = TangentCirculator(&ts2, ts2.insert(it2, straightEdge));
        start2 = TangentCirculator(&ts2, ts2.insert(it2, otherStraightEdge));
        if (newTangent->otherEndpoint(pId0)->pointId == pId2) {
            start2 = TangentCirculator(&ts2, ts2.insert(it2, newTangent));
        }
    }

    auto createCertificates = [this, &state](PointId pId, TangentCirculator start, TangentCirculator end) {
        auto beforeStart = start;
        --beforeStart;
        auto afterEnd = end;
        ++afterEnd;
        auto& current = beforeStart;
        TangentCirculator next;
        do {
            next = current;
            ++next;
            maybeAddCertificate(pId, *current, *next, state);
            ++current;
        } while (current != afterEnd);
    };

    createCertificates(pId1, start1, end1);
    createCertificates(pId2, start2, end2);

    // remove t1 and t2
    removeTangent(t1);
    removeTangent(t2);
    if (!madeOther1)
        removeTangent(*other1);
    if (!madeOther2)
        removeTangent(*other2);

    for (auto& t : m_pointIdToTangents[pId0]) {
        auto tSID = t->endpoint(pId0)->straightId;
        if (tSID && (*tSID == s1 || tSID == s2)) {
            assert(t->endpoint(pId0) == elbowObj3 || t->endpoint(pId0) == elbowObj4);
            auto oriPId0 = t->endpoint(pId0)->type == CircleStraight1 ? CGAL::CLOCKWISE : CGAL::COUNTERCLOCKWISE;
            auto oriOther = orientation(t, t->otherEndpoint(pId0)->pointId, state);
            auto pId0Source = t->endpoint(pId0) == t->source;
            auto oriSource = pId0Source ? oriPId0 : oriOther;
            auto oriTarget = pId0Source ? oriOther : oriPId0;
            m_certificates.erase(std::remove_if(m_certificates.begin(), m_certificates.end(), [t](const Certificate& c) {
                if (auto pcP = std::get_if<PointCertificate>(&c)) {
                    return pcP->t == t;
                }
                return false;
            }), m_certificates.end());

            t->endpoint(pId0) = circleTangentObject(pId0);

            t->type = tangentType(oriSource, oriTarget, t->source->elbowPoint(state).has_value(), t->target->elbowPoint(state).has_value());
        }
    }

    auto [prevNew0, nextNew0] = neighbouringTangents(pId0, newTangent);
    maybeAddCertificate(pId0, prevNew0, newTangent, state);
    maybeAddCertificate(pId0, newTangent, nextNew0, state);

    if (state.pointIdToElbows[pId0].empty()) {
        std::cout << "[fix " << m_k << "] Collapsed the only elbow on point " << pId0 << std::endl;
        return;
    }
    auto nowOuterElbow = state.pointIdToElbows[pId0].back();
    int k = state.pointIdToCat[nowOuterElbow.first.first];
    pts.c[k].postCollapseElbow(pId0, state, removingInnerOrbit, nowOuterElbow);
    if (removingInnerOrbit) {
        std::cout << "[fix " << m_k << "] Removing inner orbit!" << std::endl;
        // If we removed an inner elbow, similarly remove tangents.
        // By construction these tangents will connect from csObj1 or csObj3 to pId0.
        bool elbow1 = false;
        bool elbow2 = false;
        std::shared_ptr<Tangent> tr1;
        std::shared_ptr<Tangent> tr2;
        auto [prevS, nextS] = neighbouringTangents(straightEdge->source->pointId, straightEdge);
        auto [prevT, nextT] = neighbouringTangents(straightEdge->target->pointId, straightEdge);

        for (auto& t : {prevS, nextS, prevT, nextT}) {
            if (t->edgeOfStraight) continue;
            if (t->source->pointId != pId0 && t->target->pointId != pId0) continue;
            auto toe = t->otherEndpoint(pId0);

            if (toe == csObj1 || csObj1->elbowPoint(state) && toe == circleTangentObject(csObj1->pointId)) {
                tr1 = t;
                if (csObj1->elbowPoint(state))
                    elbow1 = true;
            }
            if (toe == csObj3 || csObj3->elbowPoint(state) && toe == circleTangentObject(csObj3->pointId)) {
                tr2 = t;
                if (csObj3->elbowPoint(state))
                    elbow2 = true;
            }
        }

        auto handle = [this, &state, pId0](const std::shared_ptr<Tangent>& tr1, const std::shared_ptr<Tangent>& tr2) {
            auto [prev, next] = neighbouringTangents(tr2->otherEndpoint(pId0)->pointId, tr2);
            auto tr1oePId = tr1->otherEndpoint(pId0)->pointId;
            auto tr2oePId = tr2->otherEndpoint(pId0)->pointId;
            std::shared_ptr<Tangent> replaced;
            std::shared_ptr<Tangent> replacement;
            if (prev->otherEndpoint(tr2oePId) == circleTangentObject(pId0) &&
                prev->endpoint(tr2oePId) == circleTangentObject(tr2oePId)) {
                replaced = prev;
                replacement = std::make_shared<Tangent>(*replaced);
                replacement->endpoint(pId0) = circleTangentObject(tr1->otherEndpoint(pId0)->pointId);
            }
            if (next->otherEndpoint(tr2oePId) == circleTangentObject(pId0) &&
                next->endpoint(tr2oePId) == circleTangentObject(tr2oePId)) {
                replaced = next;
                replacement = std::make_shared<Tangent>(*replaced);
                replacement->endpoint(pId0) = circleTangentObject(tr1oePId);
            }
            addTangent(replacement);
            auto tr2oePIdIt = std::find(m_pointIdToTangents[tr2oePId].begin(), m_pointIdToTangents[tr2oePId].end(), replaced);
            insertTangentAndAddCertificates(tr2oePId, TangentCirculator(&m_pointIdToTangents[tr2oePId], tr2oePIdIt), replacement, state);
            auto tr1oePIdIt = std::find(m_pointIdToTangents[tr1oePId].begin(), m_pointIdToTangents[tr1oePId].end(), tr1);
            insertTangentAndAddCertificates(tr1oePId, TangentCirculator(&m_pointIdToTangents[tr1oePId], tr1oePIdIt), replacement, state);
            removeTangentAndAddCertificates(replaced);
        };

        if (elbow1 && !elbow2) {
            handle(tr1, tr2);
        } else if (!elbow1 && elbow2) {
            handle(tr2, tr1);
        }

        removeTangentAndAddCertificates(tr1);
        removeTangentAndAddCertificates(tr2);
    }
}

void Pseudotriangulation::postCollapseElbow(PointId pId0, State& state, bool removingInnerOrbit, ElbowId nowOuterElbow) {
    std::cout << "[fix " << m_k << "] Post collapse elbow." << std::endl;

    // If after collapsing the elbow, there are still elbows, then check all tangents that have
    // the now outer elbow as an endpoint; these should instead get the circle as endpoint (except edges of straights).
    for (const auto& t : m_pointIdToTangents[pId0]) {
        if (t->edgeOfStraight) continue;
        auto maybeElbow = t->endpoint(pId0)->elbowPoint(state);
        if (maybeElbow.has_value() && *maybeElbow == *nowOuterElbow.second) {
            auto sOri = sourceOrientation(t, state);
            auto tOri = targetOrientation(t, state);
            std::cout << "[debugging] before t: " << *t << " sOri: " << sOri << " tOri: " << tOri << std::endl;
            t->endpoint(pId0) = circleTangentObject(pId0);
            t->type = tangentType(sOri, tOri, false, false);
            std::cout << "[debugging] after t: " << *t << std::endl;
            m_certificates.erase(std::remove_if(m_certificates.begin(), m_certificates.end(), [&](const Certificate& c) {
                if (auto pcp = std::get_if<PointCertificate>(&c)) {
                    return pcp->t == t;
                }
                return false;
            }), m_certificates.end());
        }
    }

	// If we removed an outer elbow and the elbow was not the last elbow then remove 'duplicate' tangents at edges of the now outer elbow.
    if (!removingInnerOrbit) {
        std::cout << "[fix " << m_k << "] Removing outer orbit!" << std::endl;
        auto [ebt1, ebt2] = elbowTangents(pId0, state);
        auto nearlyEqual = [pId0, this, &state](const std::shared_ptr<Tangent> &ebt, const std::shared_ptr<Tangent> &t) {
            return t->endpoint(pId0) == circleTangentObject(pId0) &&
                    (ebt->otherEndpoint(pId0)->elbowPoint(state) ?
                    t->otherEndpoint(pId0) == circleTangentObject(ebt->otherEndpoint(pId0)->pointId) :
                    t->otherEndpoint(pId0) == ebt->otherEndpoint(pId0));
        };
        auto handleDuplicates = [this, nearlyEqual, pId0](const std::shared_ptr<Tangent> &ebt) {
            auto circ = tangentCirculator(pId0, ebt);
            auto prev = circ;
            --prev;
            if (nearlyEqual(ebt, *prev)) {
                removeTangentAndAddCertificates(*prev);
            }
            auto next = circ;
            ++next;
            if (nearlyEqual(ebt, *next)) {
                removeTangentAndAddCertificates(*next);
            }
        };
        handleDuplicates(ebt1);
        handleDuplicates(ebt2);

        m_certificates.erase(std::remove_if(m_certificates.begin(), m_certificates.end(), [&](const Certificate& c) {
            if (auto iecP = std::get_if<InnerElbowCertificate>(&c)) {
                return iecP->t1 == ebt2 && iecP->t2 == ebt1;
            }
            return false;
        }), m_certificates.end());

        // also loop through and remove exact duplicates
        auto circ = tangentsCirculator(pId0);
        auto curr = circ;
        std::vector<std::shared_ptr<Tangent>> toRemove;
        do {
            auto next = curr;
            ++next;
            auto t1 = *curr;
            auto t2 = *next;
            if (identical(t1, t2)) {
                toRemove.push_back(t1);
            }
        } while (++curr != circ);
        for (auto& tr : toRemove) {
            removeTangentAndAddCertificates(tr);
        }
    }
}

void Pseudotriangulation::basicAngleZero(PointId pId0, const std::shared_ptr<Tangent>& shorter, const std::shared_ptr<Tangent>& longer, const State& state) {
    auto& t1 = shorter;
    auto& t2 = longer;
    auto obj0 = t1->endpoint(pId0);
    auto obj1 = t1->otherEndpoint(pId0);
    auto obj2 = t2->otherEndpoint(pId0);

    auto orientationNewOnObj1 = opposite(orientation(t1, obj1->pointId, state));
    auto orientationNewOnObj2 = orientation(t2, obj2->pointId, state);
    std::cout << "[fix " << m_k << "] Orientation 1: " << orientationNewOnObj1 << "; orientation 2: " << orientationNewOnObj2 << std::endl;

    std::shared_ptr<TangentObject>& nObj1 = obj1;
    std::shared_ptr<TangentObject>& nObj2 = obj2;

    if (obj1->elbowPoint(state)) {
        nObj1 = circleTangentObject(obj1->pointId);
        std::cout << "[fix " << m_k << "] Obj1 is elbow point" << std::endl;
    }
    if (obj2->elbowPoint(state)) {
        nObj2 = circleTangentObject(obj2->pointId);
        std::cout << "[fix " << m_k << "] Obj2 is elbow point" << std::endl;
    }

    auto newTangent = std::make_shared<Tangent>(tangentType(orientationNewOnObj1, orientationNewOnObj2, false, false), nObj1, nObj2);
    addAndRemove(pId0, t1, t2, state, t2, newTangent);
}

void Pseudotriangulation::fix(ConsecutiveCertificate& certificate, State& state, const Settings& settings, Pseudotriangulations& pts) {
	std::cout << "[fix " << m_k << "] Fixing certificate " << certificate << std::endl;

	const auto t1 = certificate.t1;
	const auto t2 = certificate.t2;
	auto pId0 = certificate.pointId;
	auto obj0 = t1->endpoint(pId0);
	auto obj1 = t1->otherEndpoint(pId0);
	auto obj2 = t2->otherEndpoint(pId0);
	auto pId1 = obj1->pointId;
	auto pId2 = obj2->pointId;
	bool t1R = t1->target->pointId == pId0;
	bool t2R = t2->target->pointId == pId0;
    bool angleZero = orientation(t1, t1R, state) == orientation(t2, t2R, state) ||
                     t1->endpoint(pId0)->straightId.has_value() && t1->endpoint(pId0) == t2->endpoint(pId0); // this part is probably not necessary anymore

    if (t2->endpoint(pId0)->circleStraight() && !t2->endpoint(pId0)->elbowPoint(state) && !t1->endpoint(pId0)->circleStraight()) {
        std::cout << "[fix " << m_k << "] Snapping tangent point to circle-straight intersection" << std::endl;
        snapTangentToPoint(state, pId0, t1, t2);
        return;
    }
    if (t1->endpoint(pId0)->circleStraight() && !t1->endpoint(pId0)->elbowPoint(state) && !t2->endpoint(pId0)->circleStraight()) {
        std::cout << "[fix " << m_k << "] Snapping tangent point to circle-straight intersection" << std::endl;
        snapTangentToPoint(state, pId0, t2, t1);
        return;
    }

    if (angleZero &&
            (!t1->endpoint(pId0)->elbowPoint(state) && !t2->endpoint(pId0)->elbowPoint(state) ||
            t1->otherEndpoint(pId0)->pointId == t2->otherEndpoint(pId0)->pointId) &&
             (t1->endpoint(pId0) != t2->endpoint(pId0)  ||
             t1->endpoint(pId0) == t2->endpoint(pId0) && t1->otherEndpoint(pId0)->circleStraight() &&
             t2->otherEndpoint(pId0)->circleStraight() &&
             t1->otherEndpoint(pId0)->pointId == t2->otherEndpoint(pId0)->pointId)
    ) {
        std::cout << "[fix " << m_k << "] Two edges with common endpoint start intersecting" << std::endl;
        handleIntersectingIncidentStraights(certificate, state);
        return;
    }

	std::cout << "angleZero: " << angleZero << std::endl;
	auto& longer = certificate.t1SubsetOft2 ? t2 : t1;
	// Is oldTangent an edge of a straight?
	if (angleZero && longer->type == PointPoint) {
        auto& shorter = longer == t2 ? t1 : t2;
		auto maybeOtherEdge = otherEdgeOfStraight(longer);
		if (maybeOtherEdge.has_value()) {
            std::cout << "[fix " << m_k << "] Hit edge of straight" << std::endl;
			auto& otherEdge = *maybeOtherEdge;
			// Yes, subT is an edge of a straight.
            auto oldStraight = *(longer->endpoint(pId0)->straightId);
            splitStraight(state, settings, longer, shorter, otherEdge, pId0, oldStraight, pts);
			return;
		}
	}
    if (angleZero && certificate.t1SubsetOft2) {
        std::cout << "[fix " << m_k << "] Corner angle became zero, type 1" << std::endl;

		//      t1
		//   |-------|
		// obj0 -- obj1 -- obj2
		//   |--------------|
		//          t2

		// remove t2 and add new tangent between obj1 and obj2

		//      t1
		//   |-------|
		// obj0 -- obj1 -- obj2
		//           |-------|
		//              new

		// new tangent should have orientation opposite that of t1 on obj1
		// and the same orientation as t2 on obj2
        basicAngleZero(pId0, t1, t2, state);
	} else if (angleZero) {
        std::cout << "[fix " << m_k << "] Corner angle became zero, type 2" << std::endl;
        basicAngleZero(pId0, t2, t1, state);
		//      t2
		//   |-------|
		// obj0 -- obj2 -- obj1
		//   |--------------|
		//          t1

		// remove t1 and add new tangent between obj1 and obj2

		//      t2
		//   |-------|
		// obj0 -- obj2 -- obj1
		//           |-------|
		//              new

		// new tangent should have orientation opposite that of t2 on obj2
		// and the same orientation as t1 on obj1
	} else if (t1->endpoint(pId0)->elbowPoint(state) && t2->endpoint(pId0)->elbowPoint(state)) {
        std::cout << "[fix " << m_k << "] Elbow collapsed" << std::endl;
		collapseElbow(pId0, t1, t2, state, pts);
	} else {
        std::cout << "[fix " << m_k << "] Non-corner angle became smaller than pi radians" << std::endl;
		//      t1
		//   |-------|
		// obj1 -- obj0 -- obj2
		//           |-------|
		//              t2

		// remove t1 or t2; we remove t2 if possible.
		// add new tangent between obj1 and obj2

		//      t1
		//   |-------|
		// obj1 -- obj0 -- obj2
		//   |---------------|
		//          new

		// new tangent should have orientation same as that of t1 on obj1 and t2 on obj2
		auto orientationNewOnObj1 = orientation(t1, !t1R, state);
		auto orientationNewOnObj2 = orientation(t2, !t2R, state);

		std::shared_ptr<TangentObject>& nObj1 = obj1;
		std::shared_ptr<TangentObject>& nObj2 = obj2;

		if (obj1->elbowPoint(state)) {
			nObj1 = circleTangentObject(obj1->pointId);
			std::cout << "[fix " << m_k << "] Obj1 is elbow point" << std::endl;
		}
		if (obj2->elbowPoint(state)) {
			nObj2 = circleTangentObject(obj2->pointId);
			std::cout << "[fix " << m_k << "] Obj2 is elbow point" << std::endl;
		}

		auto newTangent = std::make_shared<Tangent>(tangentType(orientationNewOnObj1, orientationNewOnObj2, false, false), nObj1, nObj2);

		if (t2->edgeOfStraight) {
			addAndRemove(pId0, t1, t2, state, t1, newTangent);
		} else {
			addAndRemove(pId0, t1, t2, state, t2, newTangent);
		}
	}
}

std::pair<Pseudotriangulations, PseudotriangulationGeometries>
PseudotriangulationGeometries::initialize(InputInstance& input, State& state, const StateGeometry& stateGeometry) {
    Pseudotriangulations pts;
    PseudotriangulationGeometries ptgs;
    for (int k = 0; k < input.numCategories(); ++k) {
        auto [pt, ptg] = PseudotriangulationGeometry::initialize(input, state, stateGeometry, k);
        pts.c.push_back(pt);
        ptgs.c.push_back(ptg);
    }
    return {pts, ptgs};
}

PseudotriangulationGeometries::PseudotriangulationGeometries(const Pseudotriangulations& pts, const State& state, const StateGeometry& stateGeometry, const InputInstance& input) {
    for (const auto& pt : pts.c) {
        c.emplace_back(pt, state, stateGeometry, input);
    }
}

TangentType reverse(TangentType type) {
    switch (type) {
        case Outer1: return Outer2;
        case Outer2: return Outer1;
        case Inner1: return Inner1;
        case Inner2: return Inner2;
        case PointCircle1: return CirclePoint2;
        case PointCircle2: return CirclePoint1;
        case CirclePoint1: return PointCircle2;
        case CirclePoint2: return PointCircle1;
        case PointPoint: return PointPoint;
    }
    throw std::runtime_error("Unknown tangent type");
}
}