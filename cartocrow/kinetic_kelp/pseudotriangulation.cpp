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
            auto& edgeG = stateGeometry.edgeGeometry.at(edge);
            auto orbitsIt = edge.first == pId ? orbits.begin() : (--orbits.end());
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

	// Sort tangents around each point
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
			pt.maybeAddCertificate(pId, t1, t2);
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
Pseudotriangulation::TangentEndpointCertificate::valid(const State& state, const StateGeometry& stateGeometry, const InputInstance& input, const Settings& settings) {
	auto t1G = PseudotriangulationGeometry::geometry(*t1, state, stateGeometry, input);
	auto t2G = PseudotriangulationGeometry::geometry(*t2, state, stateGeometry, input);
	bool t1R = t1->target->pointId == pointId;
	bool t2R = t2->target->pointId == pointId;
	auto v1 = t1G->target() - t1G->source();
	auto v2 = t2G->target() - t2G->source();
	if (t1R) {
		v1 = -v1;
	}
	if (t2R) {
		v2 = -v2;
	}
	bool valid;
	if (t1->endpoint(t1R)->circleTangent() && t2->endpoint(t2R)->circleTangent() && t1->orientation(t1R) != t2->orientation(t2R)) {
		valid = CGAL::determinant(v1, v2) < 0;
	} else {
		valid = CGAL::determinant(v1, v2) > 0;
	}

	t1SubsetOft2 = v1.squared_length() < v2.squared_length();

	return valid;
}

bool
Pseudotriangulation::TangentEndpointCertificate::valid(const State& state, const InputInstance& input, const Settings& settings) {
	// todo: selectively pass input
	// todo: selectively compute geometry and/or (for now) add option for passing state geometry
	StateGeometry stateGeometry(state, input, settings);
	// todo: approximate
	valid(state, stateGeometry, input, settings);
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
	case PointPoint:
		return "PointPoint";
	}
	throw std::invalid_argument("Unimplemented handling of a tangent type.");
}

std::ostream& operator<<(std::ostream& os, const Pseudotriangulation::Tangent& t) {
	os << name(t.type) << "," << t.source->pointId << " -> " << t.target->pointId;
	return os;
}

std::ostream& operator<<(std::ostream& os, const Pseudotriangulation::TangentEndpointCertificate& c)
{
	os << "pId: " << c.pointId << " t1(" << *c.t1 << ") t2(" << *c.t2 << ")";
	return os;
}

void Pseudotriangulation::removeTangent(std::shared_ptr<Tangent> t) {
	std::cout << "Removing tangent " << *t << std::endl;

	auto it = std::remove(m_tangents.begin(), m_tangents.end(), t);
	if (it == m_tangents.end()) {
		std::cerr << "Could not find tangent to delete" << std::endl;
	}
	m_tangents.erase(it, m_tangents.end());
	auto itC = std::remove_if(m_tangentEndpointCertificates.begin(), m_tangentEndpointCertificates.end(), [&t](const TangentEndpointCertificate& tP) {
		return *tP.t1 == *t || *tP.t2 == *t;
	});
	m_tangentEndpointCertificates.erase(itC, m_tangentEndpointCertificates.end());

	auto& tsS = m_pointIdToTangents[t->source->pointId];
	tsS.erase(std::remove(tsS.begin(), tsS.end(), t), tsS.end());
	auto& tsT = m_pointIdToTangents[t->target->pointId];
	tsT.erase(std::remove(tsT.begin(), tsT.end(), t), tsT.end());
}

TangentType tangentType(CGAL::Orientation or1, CGAL::Orientation or2) {
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
	if (or1 == CGAL::COLLINEAR && or2 == CGAL::COLLINEAR) {
		return PointPoint;
	}
	throw std::invalid_argument("Unexpected case.");
}

void Pseudotriangulation::maybeAddCertificate(PointId pId, const std::shared_ptr<Tangent>& t1, const std::shared_ptr<Tangent>& t2) {
	if (t1->otherEndpoint(pId)->pointId == t2->otherEndpoint(pId)->pointId) return;
	if (!t1->endpoint(pId)->circleStraight() && t2->endpoint(pId)->circleStraight() && t1->orientation(pId) == CGAL::COUNTERCLOCKWISE) {
		assert(t2->endpoint(pId)->type == Pseudotriangulation::CircleStraight1);
		return;
	}
	if (t1->endpoint(pId)->circleStraight() && !t2->endpoint(pId)->circleStraight() && t2->orientation(pId) == CGAL::CLOCKWISE) {
		assert(t1->endpoint(pId)->type == Pseudotriangulation::CircleStraight2);
		return;
	}
	m_tangentEndpointCertificates.emplace_back(pId, t1, t2);
}

Pseudotriangulation::TangentCirculator Pseudotriangulation::tangentCirculator(PointId pId, const std::shared_ptr<Tangent>& t) {
	auto& ts = m_pointIdToTangents[pId];
	auto tIt = std::find(ts.begin(), ts.end(), t);
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

void Pseudotriangulation::fix(TangentEndpointCertificate& certificate, State& state, const Settings& settings) {
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

    bool angleZero = t1->orientation(t1R) == t2->orientation(t2R) || t1->endpoint(pId0)->circleStraight() && t1->endpoint(pId0) == t2->endpoint(pId0);

    auto handle = [&](const std::shared_ptr<Tangent>& oldTangent, const std::shared_ptr<Tangent>& newTangent) {
		std::cout << "New tangent: " << *newTangent << std::endl;

        auto [prev0, next0] = neighbouringTangents(pId0, oldTangent);

        auto t1It = std::find(ts1.begin(), ts1.end(), t1);
		if (t2 == oldTangent && (angleZero ? t1->orientation(!t1R) == CGAL::CLOCKWISE : t1->orientation(t1R) == CGAL::COUNTERCLOCKWISE)) {
			++t1It;
		}
		auto newIt1 = ts1.insert(t1It, newTangent);
        TangentCirculator newCirc1(&ts1, newIt1);

		auto t2It = std::find(ts2.begin(), ts2.end(), t2);
		if (t1 == oldTangent && (angleZero ? t2->orientation(!t2R) == CGAL::CLOCKWISE : t2->orientation(t2R) == CGAL::COUNTERCLOCKWISE)) {
			++t2It;
		}
		auto newIt2 = ts2.insert(t2It, newTangent);
        TangentCirculator newCirc2(&ts2, newIt2);

		removeTangent(oldTangent);
		m_tangents.push_back(newTangent);

		auto prev1 = newCirc1;
		--prev1;
		auto next1 = newCirc1;
		++next1;

		maybeAddCertificate(pId1, *prev1, newTangent);
		maybeAddCertificate(pId1, newTangent, *next1);

		m_tangentEndpointCertificates.erase(std::remove_if(m_tangentEndpointCertificates.begin(), m_tangentEndpointCertificates.end(), [&](const TangentEndpointCertificate& c) {
			return c.t1 == *prev1 && c.t2 == *next1;
		}), m_tangentEndpointCertificates.end());

		auto prev2 = newCirc2;
		--prev2;
		auto next2 = newCirc2;
		++next2;

		maybeAddCertificate(pId2, *prev2, newTangent);
		maybeAddCertificate(pId2, newTangent, *next2);
		maybeAddCertificate(pId0, prev0, next0);

		m_tangentEndpointCertificates.erase(std::remove_if(m_tangentEndpointCertificates.begin(), m_tangentEndpointCertificates.end(), [&](const TangentEndpointCertificate& c) {
			return c.t1 == *prev2 && c.t2 == *next2;
		}), m_tangentEndpointCertificates.end());
	};

	std::cout << "Before fixing" << std::endl;
	for (const auto& c : m_tangentEndpointCertificates) {
		if (c.pointId == pId0 || c.pointId == pId1 || c.pointId == pId2) {
			std::cout << "Certificate " << c << std::endl;
		}
	}

	std::cout << "angleZero: " << angleZero << std::endl;
	auto longer = certificate.t1SubsetOft2 ? t2 : t1;
	// Is oldTangent an edge of a straight?
	if (angleZero && longer->type == PointPoint) {
		auto shorter = longer == t1 ? t2 : t1;
		auto longerIt = tangentCirculator(pId0, longer);
		auto prev0It = longerIt;
		--prev0It;
		auto next0It = longerIt;
		++next0It;
		auto prev0 = *prev0It;
		auto next0 = *next0It;
		auto maybeOtherEdge = prev0 == shorter ? next0 : prev0;
		std::cout << "!" << std::endl;
		if (maybeOtherEdge->type == PointPoint &&
		    longer->otherEndpoint(pId0) != maybeOtherEdge->otherEndpoint(pId0) &&
		    longer->otherEndpoint(pId0)->pointId == maybeOtherEdge->otherEndpoint(pId0)->pointId &&
		    longer->endpoint(pId0) != maybeOtherEdge->endpoint(pId0)) {
			std::cout << "Hit edge of straight!" << std::endl;
			auto& otherEdge = maybeOtherEdge;
			// Yes, subT is an edge of a straight.
			// We want to split maybeOtherEdge into two tangents.
			// Update cerificates etc.
			// And update state: add orbit to edge.

			// Make orbit
			auto oldStraight = *(longer->endpoint(pId0)->straightId);
			auto& [edge, orbitAfterIt] = oldStraight;
			auto obstaclePId = t1 == shorter ? pId1 : pId2;
			auto currentObstacleOrbits = state.pointIdToElbows[obstaclePId];
			Orbit newOrbit;
			newOrbit.pointId = obstaclePId;
			if (currentObstacleOrbits.empty()) {
				newOrbit.innerRadius = settings.vertexRadius;
				newOrbit.outerRadius = settings.vertexRadius + settings.edgeWidth;
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

			// Update tangent objects
			longer->endpoint(sId)->straightId = newStraightId;
			otherEdge->endpoint(sId)->straightId = newStraightId;
			// Create new tangent objects
			for (const auto& straightId : {oldStraight, newStraightId}) {
				for (const auto& one : {false, true}) {
					auto tObj = std::make_shared<TangentObject>(obstaclePId, straightId, one);
					m_tangentObjects.push_back(tObj);
					// Connect

					auto [sourceId, targetId] = state.straightEndpoints(straightId);
					auto otherId = sourceId == obstaclePId ? targetId : sourceId;

					auto connectee = one ? (longer->endpoint(otherId)->type == CircleStraight2 ? longer->endpoint(otherId) : otherEdge->endpoint(otherId))
					    				 : (longer->endpoint(otherId)->type == CircleStraight1 ? longer->endpoint(otherId) : otherEdge->endpoint(otherId));
					auto straightEdge = std::make_shared<Tangent>(PointPoint, tObj, connectee);
					std::cout << "Adding tangent " << *straightEdge << std::endl;
					m_tangents.push_back(std::move(straightEdge));
					// Todo add tangents to m_pointIdToTangents in appropriate position
				}
			}

			// Remove tangents
			std::cout << *otherEdge << " " << *t1 << " " << *t2 << std::endl;
			removeTangent(otherEdge);
			removeTangent(t1);
			removeTangent(t2);

			// Create tangents for the edges of the two new straights

			// Update certificates...

			return;
		}
	} else if (angleZero && certificate.t1SubsetOft2) {
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
		auto orientationNewOnObj1 = opposite(t1->orientation(!t1R));
		auto orientationNewOnObj2 = t2->orientation(!t2R);

		auto newTangent = orientationNewOnObj2 == CGAL::COLLINEAR && orientationNewOnObj1 != CGAL::COLLINEAR ?
		                  std::make_shared<Tangent>(tangentType(orientationNewOnObj2, orientationNewOnObj1), obj2, obj1) :
		                  std::make_shared<Tangent>(tangentType(orientationNewOnObj1, orientationNewOnObj2), obj1, obj2);
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
		auto orientationNewOnObj2 = opposite(t2->orientation(!t2R));
		auto orientationNewOnObj1 = t1->orientation(!t1R);

		auto newTangent = orientationNewOnObj2 == CGAL::COLLINEAR && orientationNewOnObj1 != CGAL::COLLINEAR ?
		                  std::make_shared<Tangent>(tangentType(orientationNewOnObj2, orientationNewOnObj1), obj2, obj1) :
		                  std::make_shared<Tangent>(tangentType(orientationNewOnObj1, orientationNewOnObj2), obj1, obj2);
		handle(t1, newTangent);
	} else {
		//      t1
		//   |-------|
		// obj1 -- obj0 -- obj2
		//           |-------|
		//              t2

		// remove t1 or t2; we remove t2.
		// add new tangent between obj1 and obj2

		//      t1
		//   |-------|
		// obj1 -- obj0 -- obj2
		//   |---------------|
		//          new

		// new tangent should have orientation same as that of t1 on obj1 and t2 on obj2
		auto orientationNewOnObj1 = t1->orientation(!t1R);
		auto orientationNewOnObj2 = t2->orientation(!t2R);

		auto newTangent = orientationNewOnObj2 == CGAL::COLLINEAR && orientationNewOnObj1 != CGAL::COLLINEAR ?
		                  std::make_shared<Tangent>(tangentType(orientationNewOnObj2, orientationNewOnObj1), obj2, obj1) :
		                  std::make_shared<Tangent>(tangentType(orientationNewOnObj1, orientationNewOnObj2), obj1, obj2);
		handle(t2, newTangent);
	}

	std::cout << "After fixing" << std::endl;
	for (auto c : m_tangentEndpointCertificates) {
		if (c.pointId == pId0 || c.pointId == pId1 || c.pointId == pId2) {
			std::cout << "Certificate: " << c << std::endl;
		}
	}
}
}