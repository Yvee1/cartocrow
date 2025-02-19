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
            auto orbit = stateGeometry.elbow(last).orbit();
            auto r = orbit.outerRadius;
            circleGeometry = RationalRadiusCircle(circle.center, r);
        }
        ptg.m_tangentObject[*(pt.m_tangentObjects.back())] = circleGeometry;
        auto& incidentEdges = state.pointIdToEdges[pId];
        for (const auto& edge : incidentEdges) {
            auto& edgeG = stateGeometry.edgeGeometry.at(edge);
            auto straightIndex = edge.first == pId ? 0 : edgeG.straights.size() - 1;
            auto straight = edgeG.straights[straightIndex];
            auto straightId = std::pair(edge, straightIndex);
            std::vector<OneRootPoint> ipts;
            intersectionPoints(straight.csPolygon(), circleToCSPolygon(circleGeometry.circle()), std::back_inserter(ipts));
            for (const auto& ipt : ipts) {
                Point<Exact> approx = pretendExact(approximateOneRootPoint(ipt));

                bool one = liesOnHalf(ipt, straight, true);
                assert(one || liesOnHalf(ipt, straight, false));
                pt.m_tangentObjects.push_back(std::make_shared<TangentObject>(pId, straightId, one));
                assert(!ptg.m_tangentObject.contains(*(pt.m_tangentObjects.back())));
                ptg.m_tangentObject[*(pt.m_tangentObjects.back())] = approx;
            }
        }

        auto& orbitElbows = state.pointIdToElbows[pId];
        for (const auto& elbowId : orbitElbows) {
            auto elbow = stateGeometry.elbow(elbowId);
            for (const auto& [straight, straightId] : {std::pair(*(elbow.prev), StraightId{elbowId.first, elbowId.second}), std::pair(*(elbow.next), StraightId{elbowId.first, elbowId.second + 1})}) {
                std::vector<OneRootPoint> ipts;
                intersectionPoints(straight.csPolygon(), circleToCSPolygon(circleGeometry.circle()), std::back_inserter(ipts));
                for (const auto &ipt: ipts) {
                    Point<Exact> approx = pretendExact(approximateOneRootPoint(ipt));

                    bool one = liesOnHalf(ipt, straight, true);
                    assert(one || liesOnHalf(ipt, straight, false));
                    pt.m_tangentObjects.push_back(std::make_shared<TangentObject>(pId, straightId, one));
                    assert(!ptg.m_tangentObject.contains(*(pt.m_tangentObjects.back())));
                    ptg.m_tangentObject[*(pt.m_tangentObjects.back())] = approx;
                }
            }
        }
        // todo circle-circle intersections
    }

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
                if (sId1 == sId2) continue;
            }
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
                    auto& orbit = stateGeometry.elbow(last).orbit();
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

    std::vector<std::pair<Tangent, RationalTangent>> finalTangents;

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

    for (const auto& [t, tg] : finalTangents) {
        pt.m_tangents.push_back(std::make_shared<Tangent>(t));
        ptg.m_tangents[*(pt.m_tangents.back())] = tg;
    }

    return {pt, ptg};
}

//Point<Exact> PseudotriangulationGeometry::geometry(const Vertex& vertex, const InputInstance& input, const Settings& settings) {
//}

PseudotriangulationGeometry::TangentObjectGeometry
PseudotriangulationGeometry::geometry(const TangentObject& tangentObject, const State& state, const StateGeometry& stateGeometry, const InputInstance& input) {
    auto pId = tangentObject.pointId;
    auto& elbows = state.pointIdToElbows[pId];
    RationalRadiusCircle circleGeometry;
    if (elbows.empty()) {
        circleGeometry = stateGeometry.vertexGeometry.at(pId);
    } else {
        auto r = stateGeometry.elbow(elbows.back()).orbit().outerRadius;
        circleGeometry = RationalRadiusCircle(input[tangentObject.pointId].point, r);
    }

    if (tangentObject.type == Pseudotriangulation::TangentObjectType::Circle) {
        return circleGeometry;
    }

    if (tangentObject.straightId.has_value()) {
        auto straightId = *(tangentObject.straightId);
        const auto& straight = stateGeometry.straight(straightId);
        std::vector<OneRootPoint> ipts;
        intersectionPoints(straight.csPolygon(), circleToCSPolygon(circleGeometry.circle()), std::back_inserter(ipts));
        for (const auto& ipt: ipts) {
            bool one = liesOnHalf(ipt, straight, true);
            if (one && tangentObject.type == Pseudotriangulation::CircleStraight1 ||
                !one && tangentObject.type == Pseudotriangulation::CircleStraight2) {
                Point<Exact> approx = pretendExact(approximateOneRootPoint(ipt));
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

    throw std::runtime_error("Unexpected tangent type.");
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
}