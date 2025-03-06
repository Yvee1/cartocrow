#include "pseudotriangulation.h"
#include "cartocrow/circle_segment_helpers/cs_polyline_helpers.h"
#include "cartocrow/circle_segment_helpers/poly_line_gon_intersection.h"

#include <future>

namespace cartocrow::kinetic_kelp {
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

    if (pl1S == pl2S || pl1T == pl2T || pl1S == pl2T) return false;

    for (auto eit1 = pl1.edges_begin(); eit1 != pl1.edges_end(); ++eit1) {
        for (auto eit2 = pl2.edges_begin(); eit2 != pl2.edges_end(); ++eit2) {
            if (CGAL::do_intersect(*eit1, *eit2)) {
                return true;
            }
        }
    }

    return false;
}

std::pair<Pseudotriangulation, PseudotriangulationGeometry> PseudotriangulationGeometry::pseudotriangulationTangents(const State& state, const StateGeometry& stateGeometry) {
    Pseudotriangulation pt;
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
                        Tangent straightTangent(PointPoint, obj1, obj2);
                        finalTangents.emplace_back(straightTangent, RationalTangent(Segment<Exact>(p1, p2)));
                    }
                    continue;
                }
            }
			if (obj1->elbowPoint(state) || obj2->elbowPoint(state)) continue;
            tangents(std::pair(obj1, obj1G), std::pair(obj2, obj2G), std::back_inserter(allTangents));
        }
    }

    auto task = [&allTangents, &state, &stateGeometry](int iStart, int iEnd) {
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
            if (pt.edgeOfStraight(t).has_value()) continue;
            pt.m_certificates.emplace_back(Pseudotriangulation::PointCertificate(pId, t));
        }
    }

    return {pt, ptg};
}

PseudotriangulationGeometry::TangentObjectGeometry
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

    if (tangentObject.straightId.has_value()) {
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
	return geometry(tangent, sObj, tObj);
}

PseudotriangulationGeometry::PseudotriangulationGeometry(const Pseudotriangulation& pt, const State& state, const StateGeometry& stateGeometry, const InputInstance& input) {
    for (const auto& tObj : pt.m_tangentObjects) {
        try {
            m_tangentObject[*tObj] = geometry(*tObj, state, stateGeometry, input);
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
            std::cerr << "Tangent does not exist!" << std::endl;
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
    auto tG = ptg.m_tangents.at(*t);
    auto& point = input[pointId].point;
    return valid(tG, point);
}

bool
Pseudotriangulation::PointCertificate::valid(Pseudotriangulation& pt, const State& state, const PseudotriangulationGeometry& ptg, const InputInstance& input) {
    return valid(ptg, input);
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

    // Check whether t1 and t2 are part of consecutive straights.
    bool special = false;
//    if (t1->otherEndpoint(pointId)->circleStraight() && t2->otherEndpoint(pointId)->circleStraight()) {
//        auto straightId1 = *t1->otherEndpoint(pointId)->straightId;
//        auto straightId2 = *t2->otherEndpoint(pointId)->straightId;
//        auto [s1, t1] = state.straightEndpoints(straightId1);
//        auto [s2, t2] = state.straightEndpoints(straightId2);
//        if (straightId1.first == straightId2.first && t1 == s2 || t2 == s1) {
//            special = true;
//        }
//    }

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
	        pt.orientation(t2, t2R, state) == CGAL::CLOCKWISE && pt.orientation(t1, t1R, state) == CGAL::COUNTERCLOCKWISE ||
            special) {
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
	auto t2G = PseudotriangulationGeometry::geometry(*t2, state, stateGeometry, input);
    auto& point = input[pointId].point;
	if (!t1G.has_value()) {
		throw std::runtime_error("Tangent does not exist!");
	}
	if (!t2G.has_value()) {
		throw std::runtime_error("Tangent does not exist!");
	}
	return valid(pt, state, *t1G, *t2G, point);
}

bool Pseudotriangulation::ConsecutiveCertificate::valid(Pseudotriangulation& pt, const State& state, const PseudotriangulationGeometry& ptg, const InputInstance& input) {
	auto t1G = ptg.m_tangents.at(*t1);
	auto t2G = ptg.m_tangents.at(*t2);
    auto& point = input[pointId].point;

	return valid(pt, state, t1G, t2G, point);
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
    }
    throw std::invalid_argument("Unimplemented handling of a tangent object type.");
}

std::ostream& operator<<(std::ostream& os, const Pseudotriangulation::TangentObject& t) {
    os << "TangentObject(" << name(t.type) << ", pId: " << t.pointId << ", straight: " << (t.straightId ? std::to_string(t.straightId->first.first) + " " + std::to_string(t.straightId->first.second) : "no") << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Pseudotriangulation::Tangent& t) {
	os << name(t.type) << "," << *t.source << " -> " << *t.target;
	return os;
}

std::ostream& operator<<(std::ostream& os, const Pseudotriangulation::ConsecutiveCertificate& c)
{
	os << "pId: " << c.pointId << " t1(" << *c.t1 << ") t2(" << *c.t2 << ")";
	return os;
}

bool Pseudotriangulation::usesTangent(const Pseudotriangulation::Certificate& certificate, const std::shared_ptr<Pseudotriangulation::Tangent>& t) {
    return std::visit([&t](const auto& c) {
        return c.usesTangent(t);
    }, certificate);
}

void Pseudotriangulation::removeTangent(std::shared_ptr<Tangent> t) {
	std::cout << "Removing tangent " << *t << std::endl;

	auto it = std::remove(m_tangents.begin(), m_tangents.end(), t);
	if (it == m_tangents.end()) {
		std::cerr << "Could not find tangent to delete" << std::endl;
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
//	if (t1->otherEndpoint(pId)->pointId == t2->otherEndpoint(pId)->pointId) return;
//	if (!t1->endpoint(pId)->circleStraight() && t2->endpoint(pId)->circleStraight() && orientation(t1, pId, state) == CGAL::COUNTERCLOCKWISE) {
//		assert(t2->endpoint(pId)->type == Pseudotriangulation::CircleStraight1);
//		return;
//	}
//	if (t1->endpoint(pId)->circleStraight() && !t2->endpoint(pId)->circleStraight() && orientation(t2, pId, state) == CGAL::CLOCKWISE) {
//		assert(t1->endpoint(pId)->type == Pseudotriangulation::CircleStraight2);
//		return;
//	}

    std::cout << "Creating consecutive certificate between tangents t1: " << *t1 << " and t2: " << *t2 << std::endl;
	m_certificates.emplace_back(ConsecutiveCertificate(pId, t1, t2));
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
std::optional<std::shared_ptr<Pseudotriangulation::Tangent>> Pseudotriangulation::edgeOfStraight(const std::shared_ptr<Tangent>& t) {
    if (!t->source->circleStraight() || !t->target->circleStraight()) return std::nullopt;
    auto pId = t->source->pointId;
    auto sourceTangentCirc = tangentCirculator(pId, t);
    auto prev = sourceTangentCirc;
    --prev;
    auto next = sourceTangentCirc;
    ++next;

    auto prevT = *prev;
    auto nextT = *next;

    for (const auto& otherT : {prevT, nextT}) {
        if (otherT->type == PointPoint && otherT->otherEndpoint(pId)->pointId == t->otherEndpoint(pId)->pointId &&
            otherT->endpoint(pId)->straightId == t->endpoint(pId)->straightId && otherT->otherEndpoint(pId) != t->otherEndpoint(pId) &&
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

void Pseudotriangulation::fix(Certificate& certificate, State& state, const Settings& settings) {
    std::visit([&](auto& c) { this->fix(c, state, settings); }, certificate);
}

void Pseudotriangulation::fix(PointCertificate& certificate, State& state, const Settings& settings) {
    auto& t = certificate.t;
    auto& pId = certificate.pointId;
    auto obj = t->endpoint(pId);
    auto other = t->otherEndpoint(pId);

    assert(obj->type == CircleStraight1 || obj->type == CircleStraight2);

    auto thisOri = obj->type == CircleStraight1 ? CGAL::CLOCKWISE : CGAL::COUNTERCLOCKWISE;
    CGAL::Orientation otherOri = orientation(t, other->pointId, state);
    t->endpoint(pId) = circleTangentObject(pId);
    auto sourceOri = t->endpoint(pId) == t->source ? thisOri : otherOri;
    auto targetOri = t->endpoint(pId) == t->source ? otherOri : thisOri;
    t->type = tangentType(sourceOri, targetOri, t->source->elbowPoint(state).has_value(), t->target->elbowPoint(state).has_value());

    m_certificates.erase(std::remove_if(m_certificates.begin(), m_certificates.end(), [&](const Certificate& c) {
        if (auto pcP = std::get_if<PointCertificate>(&c)) {
            const PointCertificate& pc = *pcP;
            return pc == certificate;
        }
        return false;
    }), m_certificates.end());
}

void Pseudotriangulation::fix(ConsecutiveCertificate& certificate, State& state, const Settings& settings) {
	std::cout << "Fixing certificate " << certificate << std::endl;

	const auto t1 = certificate.t1;
	const auto t2 = certificate.t2;
	auto pId0 = certificate.pointId;
	auto obj0 = t1->endpoint(pId0);
	auto obj1 = t1->otherEndpoint(pId0);
	auto obj2 = t2->otherEndpoint(pId0);
	auto pId1 = obj1->pointId;
	auto pId2 = obj2->pointId;
	auto& ts0 = m_pointIdToTangents[pId0];
	auto& ts1 = m_pointIdToTangents[pId1];
	auto& ts2 = m_pointIdToTangents[pId2];
	bool t1R = t1->target->pointId == pId0;
	bool t2R = t2->target->pointId == pId0;

    auto fixTangentType = [&state, pId0, this](std::shared_ptr<Tangent> t) {
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

    std::cout << *t1->endpoint(pId0) << " is elbow point: " << t1->endpoint(pId0)->elbowPoint(state).has_value() << std::endl;
    std::cout << *t2->endpoint(pId0) << " is elbow point: " << t2->endpoint(pId0)->elbowPoint(state).has_value() << std::endl;
    // "Snap" tangent point to circle-straight intersection
    if (t2->endpoint(pId0)->circleStraight() && !t2->endpoint(pId0)->elbowPoint(state) && !t1->endpoint(pId0)->circleStraight()) {
        t1->endpoint(pId0) = t2->endpoint(pId0);
        fixTangentType(t1);
        m_certificates.push_back(PointCertificate(pId0, t1));
        return;
    }
    if (t1->endpoint(pId0)->circleStraight() && !t1->endpoint(pId0)->elbowPoint(state) && !t2->endpoint(pId0)->circleStraight()) {
        t2->endpoint(pId0) = t1->endpoint(pId0);
        fixTangentType(t2);
        m_certificates.push_back(PointCertificate(pId0, t2));
        return;
    }

    bool angleZero = orientation(t1, t1R, state) == orientation(t2, t2R, state) || t1->endpoint(pId0)->circleStraight() && t1->endpoint(pId0) == t2->endpoint(pId0);

    if (edgeOfStraight(t1) && edgeOfStraight(t2) &&
       !t1->endpoint(pId0)->elbowPoint(state) && !t2->endpoint(pId0)->elbowPoint(state) &&
            (t1->endpoint(pId0) != t2->endpoint(pId0) ||
            t1->endpoint(pId0) == t2->endpoint(pId0) && t1->otherEndpoint(pId0)->circleStraight() &&
            t2->otherEndpoint(pId0)->circleStraight())) {
        std::cout << "two edges with common endpoint start intersecting" << std::endl;
        m_certificates.erase(std::remove_if(m_certificates.begin(), m_certificates.end(), [&certificate](const Certificate& c) {
            if (auto ccP = std::get_if<ConsecutiveCertificate>(&c)) {
                return *ccP == certificate;
            }
            return false;
        }), m_certificates.end());
        return;
    }

    auto handle = [&](const std::shared_ptr<Tangent>& oldTangent, const std::shared_ptr<Tangent>& newTangent) {
		std::cout << "New tangent: " << *newTangent << std::endl;
        for (auto& ntEndpoint : {newTangent->source, newTangent->target}) {
            if (ntEndpoint->point()) {
                m_certificates.push_back(PointCertificate(ntEndpoint->pointId, newTangent));
            }
        }

        auto [prev0, next0] = neighbouringTangents(pId0, oldTangent);

        auto t1It = std::find(ts1.begin(), ts1.end(), t1);
		if (t2 == oldTangent && (angleZero ? orientation(t1, !t1R, state) == CGAL::CLOCKWISE : orientation(t1, t1R, state) == CGAL::COUNTERCLOCKWISE)) {
			++t1It;
		}
		auto newIt1 = ts1.insert(t1It, newTangent);
//        std::cout << "Inserting new tangent on pId " << pId1 << " before " << **t1It << std::endl;
        TangentCirculator newCirc1(&ts1, newIt1);

		auto t2It = std::find(ts2.begin(), ts2.end(), t2);
		if (t1 == oldTangent && (angleZero ? orientation(t2, !t2R, state) == CGAL::CLOCKWISE : orientation(t2, t2R, state) == CGAL::COUNTERCLOCKWISE)) {
			++t2It;
		}
//        std::cout << "Inserting new tangent on pId " << pId2 << " before " << **t2It << std::endl;
		auto newIt2 = ts2.insert(t2It, newTangent);
        TangentCirculator newCirc2(&ts2, newIt2);

		removeTangent(oldTangent);
		m_tangents.push_back(newTangent);

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
	};

//	std::cout << "Before fixing" << std::endl;
//	for (const auto& c : m_certificates) {
//		if (c.pointId == pId0 || c.pointId == pId1 || c.pointId == pId2) {
//			std::cout << "Certificate " << c << std::endl;
//		}
//	}

	std::cout << "angleZero: " << angleZero << std::endl;
	auto& longer = certificate.t1SubsetOft2 ? t2 : t1;
	// Is oldTangent an edge of a straight?
	if (angleZero && longer->type == PointPoint) {
        auto& shorter = longer == t2 ? t1 : t2;
		auto maybeOtherEdge = edgeOfStraight(longer);
		if (maybeOtherEdge.has_value()) {
//			std::cout << "Hit edge of straight!" << std::endl;
			auto& otherEdge = *maybeOtherEdge;
			// Yes, subT is an edge of a straight.
			// We want to split maybeOtherEdge into two tangents.
			// Update cerificates etc.
			// And update state: add orbit to edge.

			// Make orbit
			auto oldStraight = *(longer->endpoint(pId0)->straightId);
			auto& [edge, orbitAfterIt] = oldStraight;
			auto obstaclePId = t1 == shorter ? pId1 : pId2;
            auto nonObstaclePId = t1 == shorter ? pId2 : pId1;
			auto currentObstacleOrbits = state.pointIdToElbows[obstaclePId];
			Orbit newOrbit;
			newOrbit.pointId = obstaclePId;
			if (currentObstacleOrbits.empty()) {
				newOrbit.innerRadius = settings.kelpRadius;
				newOrbit.outerRadius = settings.kelpRadius + settings.edgeWidth;
			} else {
				auto lastOrbit = currentObstacleOrbits.back();
				newOrbit.innerRadius = lastOrbit.second->outerRadius;
				newOrbit.outerRadius = newOrbit.innerRadius + settings.edgeWidth;
			}

			// Store source and target pointIds of straight before updating state
			auto [sId, tId] = state.straightEndpoints(oldStraight);

			newOrbit.dir = longer->endpoint(sId)->type == CircleStraight2 ? CGAL::COUNTERCLOCKWISE : CGAL::CLOCKWISE;

			// Add orbit to edge
			auto newOrbitIt = state.edgeTopology[edge].orbits.insert(orbitAfterIt, newOrbit);
			StraightId newStraightId(edge, newOrbitIt);

			// Add elbow id to point
			state.pointIdToElbows[obstaclePId].emplace_back(edge, newOrbitIt);

            std::shared_ptr<Tangent> l1, l2, o1, o2;

//            std::vector
			// Create new tangent objects
            std::vector<std::tuple<PointId, std::list<std::shared_ptr<Tangent>>::const_iterator, std::shared_ptr<Tangent>>> positionedNewTangents;
			for (const auto& straightId : {oldStraight, newStraightId}) {
				for (const auto& one : {false, true}) {
					auto tObj = std::make_shared<TangentObject>(obstaclePId, straightId, one);
					m_tangentObjects.push_back(tObj);

					auto [sourceId, targetId] = state.straightEndpoints(straightId);
					auto otherId = sourceId == obstaclePId ? targetId : sourceId;

					auto connectee = one ? (longer->endpoint(otherId)->type == CircleStraight2 ? longer->endpoint(otherId) : otherEdge->endpoint(otherId))
					    				 : (longer->endpoint(otherId)->type == CircleStraight1 ? longer->endpoint(otherId) : otherEdge->endpoint(otherId));
					auto straightEdge = std::make_shared<Tangent>(PointPoint, tObj, connectee);
					m_tangents.push_back(straightEdge);

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
                    std::cout << "Looking for tangent that has one endpoint " << *connectee << " and the other one with pointId " << otherOtherId << " and that is an edge of straight" << std::endl;
                    auto it = std::find_if(ts.begin(), ts.end(), [connectee, otherId, otherOtherId, this](const std::shared_ptr<Tangent>& t) {
                        std::cout << "Candidate tangent: " << *t << " is it: " << (t->endpoint(otherId) == connectee && t->otherEndpoint(otherId)->pointId == otherOtherId && edgeOfStraight(t)) << std::endl;
                        return t->endpoint(otherId) == connectee && t->otherEndpoint(otherId)->pointId == otherOtherId && edgeOfStraight(t);
                    });
                    if (it == ts.end()) {
                        std::cerr << "Could not find tangent in tangent list!" << std::endl;
                    }
//                    ts.insert(it, straightEdge);
                    positionedNewTangents.emplace_back(otherId, it, straightEdge);
				}
			}

            for (const auto& [id, it, straightEdge] : positionedNewTangents) {
                m_pointIdToTangents[id].insert(it, straightEdge);
            }

            // Add tangents to obstacle object.
            auto& obstacleTs = m_pointIdToTangents[obstaclePId];
            auto it = std::find(obstacleTs.begin(), obstacleTs.end(), shorter);
            auto l1It = obstacleTs.insert(it, l1);
            obstacleTs.insert(it, o2);
            obstacleTs.insert(it, o1);
            auto l2It = obstacleTs.insert(it, l2);

            // Update tangent objects
            longer->endpoint(sId)->straightId = newStraightId;
            otherEdge->endpoint(sId)->straightId = newStraightId;

			// Remove tangents
			std::cout << *otherEdge << " " << *t1 << " " << *t2 << std::endl;
			removeTangent(otherEdge);
			removeTangent(longer);

            auto [prevT, nextT] = neighbouringTangents(pId0, shorter);

            // Remove last tangent
            removeTangent(shorter);

            // Update certificates
            TangentCirculator l1Circ(&obstacleTs, l1It);
            auto l1CircPrev = l1Circ;
            --l1CircPrev;
            if (*l1CircPrev == shorter) {
                --l1CircPrev;
            }
            maybeAddCertificate(obstaclePId, *l1CircPrev, l1, state);
            maybeAddCertificate(obstaclePId, l1, o2, state);
            maybeAddCertificate(obstaclePId, o2, o1, state);
            maybeAddCertificate(obstaclePId, o1, l2, state);
            TangentCirculator l2Circ(&obstacleTs, l2It);
            auto l2CircNext = l2Circ;
            ++l2CircNext;
            if (*l2CircNext == shorter) {
                ++l2CircNext;
            }
            maybeAddCertificate(obstaclePId, l2, *l2CircNext, state);

            maybeAddCertificate(pId0, prevT, nextT, state);

            auto somePId1 = o2->otherEndpoint(obstaclePId)->pointId;
            auto o2Circ = tangentCirculator(somePId1, o2);
            --o2Circ;
            maybeAddCertificate(somePId1, *o2Circ, o2, state);
            auto l1Circ2 = tangentCirculator(somePId1, l1);
            ++l1Circ2;
            maybeAddCertificate(somePId1, l1, *l1Circ2, state);

            auto somePId2 = l2->otherEndpoint(obstaclePId)->pointId;
            auto l2Circ2 = tangentCirculator(somePId2, l2);
            --l2Circ2;
            maybeAddCertificate(somePId2, *l2Circ2, l2, state);
            auto o1Circ = tangentCirculator(somePId2, o1);
            ++o1Circ;
            maybeAddCertificate(somePId2, o1, *o1Circ, state);

			return;
		}
	}
    if (angleZero && certificate.t1SubsetOft2) {
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
		auto orientationNewOnObj1 = opposite(orientation(t1, !t1R, state));
		auto orientationNewOnObj2 = orientation(t2, !t2R, state);

		std::shared_ptr<TangentObject>& nObj1 = obj1;
		std::shared_ptr<TangentObject>& nObj2 = obj2;

		if (obj1->elbowPoint(state)) {
			nObj1 = circleTangentObject(obj1->pointId);
		}
		if (obj2->elbowPoint(state)) {
			nObj2 = circleTangentObject(obj2->pointId);
		}

		auto newTangent = orientationNewOnObj2 == CGAL::COLLINEAR && orientationNewOnObj1 != CGAL::COLLINEAR ?
		                  std::make_shared<Tangent>(tangentType(orientationNewOnObj2, orientationNewOnObj1, false, false), nObj2, nObj1) :
		                  std::make_shared<Tangent>(tangentType(orientationNewOnObj1, orientationNewOnObj2, false, false), nObj1, nObj2);
		handle(t2, newTangent);
	} else if (angleZero) {
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
		auto orientationNewOnObj2 = opposite(orientation(t2, !t2R, state));
		auto orientationNewOnObj1 = orientation(t1, !t1R, state);

		std::shared_ptr<TangentObject>& nObj1 = obj1;
		std::shared_ptr<TangentObject>& nObj2 = obj2;

		if (obj1->elbowPoint(state)) {
			nObj1 = circleTangentObject(obj1->pointId);
		}
		if (obj2->elbowPoint(state)) {
			nObj2 = circleTangentObject(obj2->pointId);
		}
		auto newTangent = orientationNewOnObj2 == CGAL::COLLINEAR && orientationNewOnObj1 != CGAL::COLLINEAR ?
		                  std::make_shared<Tangent>(tangentType(orientationNewOnObj2, orientationNewOnObj1, false, false), nObj2, nObj1) :
		                  std::make_shared<Tangent>(tangentType(orientationNewOnObj1, orientationNewOnObj2, false, false), nObj1, nObj2);
		handle(t1, newTangent);
	} else if (t1->endpoint(pId0)->elbowPoint(state) && t2->endpoint(pId0)->elbowPoint(state)) {
		auto elbowObj1 = t1->endpoint(pId0);
		auto elbowObj2 = t2->endpoint(pId0);

		auto s1 = elbowObj1->straightId;
		auto s2 = elbowObj2->straightId;
		auto edge = s1->first;

		auto orbit = *t1->endpoint(pId0)->elbowPoint(state);

		// Other edge of straight s1
		auto other1 = *edgeOfStraight(t1);
		// Other edge of straight s2
		auto other2 = *edgeOfStraight(t2);

		auto elbowObj3 = other1->endpoint(pId0);
		auto elbowObj4 = other2->endpoint(pId0);

		auto csObj1 = t1->otherEndpoint(pId0);
		auto csObj2 = other1->otherEndpoint(pId0);
		auto csObj3 = t2->otherEndpoint(pId0);
		auto csObj4 = other2->otherEndpoint(pId0);

		bool issue12 = *csObj1->straightId->second == orbit;
		std::list<Orbit>::const_iterator oldOrbitIt;
		if (issue12) {
			auto& correct = t2->otherEndpoint(pId0)->straightId->second;
			oldOrbitIt = t1->otherEndpoint(pId0)->straightId->second;
			t1->otherEndpoint(pId0)->straightId->second = correct;
			other1->otherEndpoint(pId0)->straightId->second = correct;
		} else {
			auto& correct = t1->otherEndpoint(pId0)->straightId->second;
			oldOrbitIt = t2->otherEndpoint(pId0)->straightId->second;
			t2->otherEndpoint(pId0)->straightId->second = correct;
			other2->otherEndpoint(pId0)->straightId->second = correct;
		}

		// Remove elbow/orbit
		state.pointIdToElbows[pId0].pop_back();
		state.edgeTopology[edge].orbits.erase(oldOrbitIt);

		// create new tangent
		auto straightEdge = std::make_shared<Tangent>(PointPoint, t1->otherEndpoint(pId0), t2->otherEndpoint(pId0));
		auto otherStraightEdge = std::make_shared<Tangent>(PointPoint, other1->otherEndpoint(pId0), other2->otherEndpoint(pId0));

		// Remove tangent objects at pId0
		m_tangentObjects.erase(std::remove_if(m_tangentObjects.begin(), m_tangentObjects.end(), [&](const std::shared_ptr<TangentObject>& obj) {
			return obj == elbowObj1 || obj == elbowObj2 || obj == elbowObj3 || obj == elbowObj4;
		}), m_tangentObjects.end());

		auto it0 = std::find(ts0.begin(), ts0.end(), t1);
		auto it1 = std::find(ts1.begin(), ts1.end(), t1);
		auto it2 = std::find(ts2.begin(), ts2.end(), t2);

		auto newTangent = std::make_shared<Tangent>(csObj2->type == CircleStraight1 ? PointCircle2 : PointCircle1, csObj2, circleTangentObject(pId0));
		m_tangents.push_back(straightEdge);
		m_tangents.push_back(otherStraightEdge);
		m_tangents.push_back(newTangent);

		TangentCirculator start1, end1, start2, end2;

		auto [t1Prev, t1Next] = neighbouringTangents(pId0, t1);
		auto itNew0 = ts0.insert(it0, newTangent);
		if (t1Prev == other1 && other1->otherEndpoint(pId0)->type == CircleStraight2 ||
		    t1Next == other1 && other1->otherEndpoint(pId0)->type == CircleStraight1) {
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
		removeTangent(other1);
		removeTangent(other2);

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
	} else {
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

        // But then also not immediately clear what we want.

		auto newTangent = orientationNewOnObj2 == CGAL::COLLINEAR && orientationNewOnObj1 != CGAL::COLLINEAR ?
		                  std::make_shared<Tangent>(tangentType(orientationNewOnObj2, orientationNewOnObj1, obj2->elbowPoint(state).has_value(), obj1->elbowPoint(state).has_value()), obj2, obj1) :
		                  std::make_shared<Tangent>(tangentType(orientationNewOnObj1, orientationNewOnObj2, obj1->elbowPoint(state).has_value(), obj2->elbowPoint(state).has_value()), obj1, obj2);

		if (edgeOfStraight(t2)) {
			handle(t1, newTangent);
		} else {
			handle(t2, newTangent);
		}
	}

//	std::cout << "After fixing" << std::endl;
//	for (auto c : m_certificates) {
//		if (c.pointId == pId0 || c.pointId == pId1 || c.pointId == pId2) {
//			std::cout << "Certificate: " << c << std::endl;
//		}
//	}
}
}