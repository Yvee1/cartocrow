#include "route_edges.h"

#include "cartocrow/circle_segment_helpers/cs_render_helpers.h"
#include "cartocrow/circle_segment_helpers/cs_polyline_helpers.h"
#include "cartocrow/circle_segment_helpers/poly_line_gon_intersection.h"

#include "cartocrow/renderer/ipe_renderer.h"

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/pending/disjoint_sets.hpp>

using namespace cartocrow::renderer;

namespace cartocrow::kinetic_kelp {
CGAL::Orientation opposite(CGAL::Orientation orientation) {
    if (orientation == CGAL::CLOCKWISE) {
        return CGAL::COUNTERCLOCKWISE;
    } else if (orientation == CGAL::COUNTERCLOCKWISE) {
        return CGAL::CLOCKWISE;
    } else {
        return CGAL::COLLINEAR;
    }
}

void RoutingGraph::makeRoutingObjects() {
    for (int i = 0; i < m_input.catPoints().size(); ++i) {
        auto p = m_input[i].point;
        m_objects.push_back(std::make_shared<Object>(RationalRadiusCircle(p, m_settings.kelpRadius + m_settings.edgeWidth / 2), i));
        m_objects.push_back(std::make_shared<Object>(p, i));
    }
}

std::shared_ptr<RoutingGraph::Object> RoutingGraph::pointObject(PointId pointId) {
	return m_objects[2 * pointId + 1];
}

std::shared_ptr<RoutingGraph::Object> RoutingGraph::circleObject(PointId pointId) {
	return m_objects[2 * pointId];
}

bool RoutingGraph::free(const RationalTangent& t, const Object& object) {
	if (auto rrcP = std::get_if<RationalRadiusCircle>(&object.geom)) {
		auto rrc = *rrcP;
		auto pl = polylineToCSPolyline(t.polyline());
		auto pgn = circleToCSPolygon(rrc.circle());
		auto plBbox = CGAL::bbox_2(pl.curves_begin(), pl.curves_end());
		auto pgnBbox = CGAL::bbox_2(pgn.curves_begin(), pgn.curves_end());
		plBbox.dilate(100);
		pgnBbox.dilate(100);
		bool overlap = do_overlap(plBbox, pgnBbox);
		if (overlap && !intersection(pl, pgn, true).empty()) {
			return false;
		}
	}
	return true;
}

bool RoutingGraph::free(const CSCurve& curve, const Object& object) {
	if (auto rrcP = std::get_if<RationalRadiusCircle>(&object.geom)) {
		auto rrc = *rrcP;
		std::vector<CSXMCurve> xmCurves;
		curveToXMonotoneCurves(curve, std::back_inserter(xmCurves));
		auto pl = CSPolyline(xmCurves.begin(), xmCurves.end());
		auto pgn = circleToCSPolygon(rrc.circle());
		auto plBbox = CGAL::bbox_2(pl.curves_begin(), pl.curves_end());
		auto pgnBbox = CGAL::bbox_2(pgn.curves_begin(), pgn.curves_end());
		plBbox.dilate(100);
		pgnBbox.dilate(100);
		bool overlap = do_overlap(plBbox, pgnBbox);
		if (overlap && !intersection(pl, pgn, true).empty()) {
			return false;
		}
	}
	return true;
}

bool RoutingGraph::free(const Tangent& t) {
    bool free = true;
    for (const auto& obj : m_objects) {
        if (obj->pointId == t.source->pointId || obj->pointId == t.target->pointId) continue;
        if (!RoutingGraph::free(t.geom, *obj)) {
			free = false;
			break;
		}
    }
    return free;
}

void RoutingGraph::addTangentToGraph(const Tangent& t) {
	auto& g = *m_g;
    int vertexEndpoints = 0;
    GraphV sourceVertex;
    auto& ptv = m_pointToVertex;
    if (std::holds_alternative<RationalRadiusCircle>(t.source->geom)) {
        auto p = t.geom.source();
        if (ptv.contains(p)) {
            sourceVertex = ptv[p];
        } else {
            sourceVertex = boost::add_vertex(g);
            g[sourceVertex].point = p;
            g[sourceVertex].object = t.source;
            ptv[p] = sourceVertex;
			m_circleVertices[t.source->pointId].push_back(sourceVertex);
        }
    } else {
		auto pt = std::get<Point<Exact>>(t.source->geom);
		sourceVertex = ptv[pt];
		++vertexEndpoints;
	}
    GraphV targetVertex;
    if (std::holds_alternative<RationalRadiusCircle>(t.target->geom)) {
        auto p = t.geom.target();
        if (ptv.contains(p)) {
            targetVertex = ptv[p];
        } else {
            targetVertex = boost::add_vertex(g);
            g[targetVertex].point = p;
            g[targetVertex].object = t.target;
            ptv[p] = targetVertex;
			m_circleVertices[t.target->pointId].push_back(targetVertex);
        }
    } else {
		auto pt = std::get<Point<Exact>>(t.target->geom);
		targetVertex = ptv[pt];
        ++vertexEndpoints;
    }
    auto [e, added] = boost::add_edge(sourceVertex, targetVertex, g);
	if (!added) {
		std::cout << "Edge already in graph!" << std::endl;
	}
    g[e].geom = t.geom;
    g[e].length = CGAL::sqrt(CGAL::squared_distance(approximate(t.geom.source()), approximate(t.geom.target())));
    if (vertexEndpoints == 0) {
        g[e].weight = g[e].length;
    } else if (vertexEndpoints == 1) {
        g[e].weight = 1000000 + g[e].length;
    } else {
        g[e].weight = 2000000 + g[e].length;
    }
}

void RoutingGraph::createCircleEdges(PointId pointId) {
	auto& g = *m_g;

	auto [eit, eEnd] = boost::edges(g);
	Graph::edge_iterator next;
	for (next = eit; eit != eEnd; eit = next) {
		++next;
		auto sv = g[eit->m_source];
		auto tv = g[eit->m_target];
		if (sv.object->pointId == pointId && std::holds_alternative<RationalRadiusCircle>(sv.object->geom) &&
		    tv.object->pointId == pointId && std::holds_alternative<RationalRadiusCircle>(tv.object->geom)) {
			boost::remove_edge(*eit, g);
		}
	}

	std::vector<GraphV>& vs = m_circleVertices[pointId];
	auto& c = m_input[pointId].point;
	std::sort(vs.begin(), vs.end(), [&c, &g](const auto& v1, const auto& v2) {
		auto p1 = g[v1].point;
		auto p2 = g[v2].point;
		return (p1-c).direction() < (p2-c).direction();
	});

	auto obj = circleObject(pointId);
	for (int j = 0; j < vs.size(); ++j) {
		auto v1 = vs[j];
		auto v2 = vs[(j + 1) % m_circleVertices[pointId].size()];
		auto p1 = g[v1].point;
		auto p2 = g[v2].point;
		OneRootPoint p1A(p1.x(), p1.y());
		OneRootPoint p2A(p2.x(), p2.y());
		auto c = std::get<RationalRadiusCircle>(obj->geom);
		CSCurve curve(c.center, c.radius, CGAL::COUNTERCLOCKWISE, p1A, p2A);
		// check intersection with objects
		bool free = true;
		for (const auto& objP : m_objects) {
			if (objP->pointId != pointId && !RoutingGraph::free(curve, *objP)) {
				free = false;
				break;
			}
		}
		if (free) {
			auto e = boost::add_edge(v1, v2, g);
			g[e.first].length = approximateLength(curve);
			g[e.first].weight = g[e.first].length;
			g[e.first].geom = curve;
		}
	}
}

RoutingGraph::RoutingGraph(InputInstance input, Settings settings) : m_input(std::move(input)), m_settings(std::move(settings)) {
	m_g = std::make_unique<Graph>();
	auto& g = *m_g;
	m_circleVertices.resize(m_input.size());

    makeRoutingObjects();
    std::vector<Tangent> ts;
    freeTangents(std::back_inserter(ts));

    // Add vertices corresponding to points in the input
    for (int pointId = 0; pointId < m_input.size(); ++pointId) {
        auto v = boost::add_vertex(g);
        g[v].point = m_input[pointId].point;
        g[v].object = pointObject(pointId);
        m_pointToVertex[m_input[pointId].point] = v;
    }

    // Create edges for tangents and vertices on circles
    for (const auto& freeTangent : ts) {
        addTangentToGraph(freeTangent);
    }

    // Sort vertices on circles
    for (int pointId = 0; pointId < m_circleVertices.size(); ++pointId) {
		createCircleEdges(pointId);
    }
}

RoutingGraph::Path RoutingGraph::computePath(GraphV u, GraphV v) const {
	auto& g = *m_g;

	typedef std::unordered_map<GraphV,GraphV> VertexPredecessorMap;
	typedef boost::associative_property_map<VertexPredecessorMap> VertexPredecessorPropertyMap;
    VertexPredecessorMap predecessors;
	VertexPredecessorPropertyMap predecessorsPMap(predecessors);

	typedef std::unordered_map<GraphV,int> VertexIndexMap;
	typedef boost::associative_property_map<VertexIndexMap> VertexIndexPropertyMap;
	VertexIndexMap vertexToIndex;
	VertexIndexPropertyMap vertexToIndexPMap(vertexToIndex);
	int index = 0;
	for (GraphV vd : g.vertex_set())
		vertexToIndex[vd] = index++;
    boost::dijkstra_shortest_paths(g, u, boost::weight_map(boost::get(&Edge::weight, g))
            .predecessor_map(predecessorsPMap).vertex_index_map(vertexToIndexPMap));

    std::vector<GraphV> path;
    GraphV current = v;
    double weight = 0;
    do {
        path.push_back(current);
        auto pre = predecessors[current];
        if (pre == current) break;
        weight += g[boost::edge(current, pre, g).first].weight;
        current = pre;
    } while (current != u);
    path.push_back(current);
    std::reverse(path.begin(), path.end());
    return {path, weight};
}

bool RoutingGraph::nonProperlyIntersectedTangent(const RationalTangent& t, const CSPolygon& obstacleEdge) {
	CSPolygonSet differenceSet;
	CSPolygonSet intersectionSet;
	CSPolygon tangentEdge = rationalTangentToCSPolygon(t, m_settings);

	auto b1 = CGAL::bbox_2(tangentEdge.curves_begin(), tangentEdge.curves_end());
	auto b2 = CGAL::bbox_2(obstacleEdge.curves_begin(), obstacleEdge.curves_end());
	b1.dilate(100);
	b2.dilate(100);
	if (!do_overlap(b1, b2)) return false;

	auto dir = t.target() - t.source();
	auto norm = dir.perpendicular(CGAL::POSITIVE);
	norm = norm / CGAL::sqrt(CGAL::to_double(norm.squared_length()));
	auto p1 = t.source() + norm * m_settings.edgeWidth / 2;
	auto p2 = t.source() - norm * m_settings.edgeWidth / 2;
	auto p3 = t.target() + norm * m_settings.edgeWidth / 2;
	auto p4 = t.target() - norm * m_settings.edgeWidth / 2;

	std::vector<CSXMCurve> sourceSegCurves({{p1, p2}});
	std::vector<CSXMCurve> targetSegCurves({{p3, p4}});
	CSPolyline sourceSeg(sourceSegCurves.begin(), sourceSegCurves.end());
	CSPolyline targetSeg(targetSegCurves.begin(), targetSegCurves.end());

	return !intersection(sourceSeg, obstacleEdge, true).empty() && !intersection(targetSeg, obstacleEdge, true).empty();
}

void RoutingGraph::removeNonProperlyIntersectedTangents(GraphV u, GraphV v, const CSPolygon& obstacleEdge) {
	auto& g = *m_g;
    auto [eStart, eEnd] = boost::edges(g);
    std::vector<RoutingGraph::GraphE> nonProperlyIntersected;
    for (auto eit = eStart; eit != eEnd; ++eit) {
		auto e = *eit;
		if (e.m_source == u || e.m_source == v || e.m_target == u || e.m_target == v) continue;
		if (auto rtP = std::get_if<RationalTangent>(&(g[e].geom))) {
        	if (nonProperlyIntersectedTangent(*rtP, obstacleEdge)) {
				nonProperlyIntersected.push_back(*eit);
			}
		}
    }
    for (auto ed : nonProperlyIntersected) {
        boost::remove_edge(ed, g);
    }
}

std::pair<std::shared_ptr<State>, std::shared_ptr<StateGeometry>> routeEdges(const InputInstance& input, const Settings& settings, GeometryRenderer& renderer) {
    // Initial graph is now done, we start routing
    RoutingGraph graph(input, settings);

	auto& g = *graph.m_g;

    // Helper function for drawing edges
    auto drawEdge = [&renderer, &g](RoutingGraph::GraphE e) {
        auto geom = g[e].geom;
        if (auto rtP = std::get_if<RationalTangent>(&geom)) {
            renderer.draw(rtP->polyline());
        }
        if (auto csCurveP = std::get_if<CSCurve>(&geom)) {
            renderer.draw(renderPath(*csCurveP));
        }
    };

    std::vector<int> msts(input.numCategories());

    // Set up disjoint sets data structure
    std::vector<std::vector<int>> ranks;
    std::vector<std::vector<int>> parents;
    std::vector<boost::disjoint_sets<int*, int*>> dSets;
    ranks.reserve(input.numCategories());
    parents.reserve(input.numCategories());
    dSets.reserve(input.numCategories());
    for (int k = 0; k < input.numCategories(); ++k) {
        auto& rank = ranks.emplace_back(input.category(k).size());
        auto& parent = parents.emplace_back(input.category(k).size());
        auto& dSetsK = dSets.emplace_back(&rank[0], &parent[0]);

        for (int kPtIndex = 0; kPtIndex < input.category(k).size(); ++kPtIndex) {
            dSetsK.make_set(kPtIndex);
        }
    }

    auto state = std::make_shared<State>();
    auto stateGeometry = std::make_shared<StateGeometry>();
    for (const auto& cp : input.catPoints()) {
        state->pointIdToCat.push_back(cp.category);
    }
    state->msts = std::vector<MST>(input.numCategories());
    state->pointIdToEdges = std::vector<std::list<MSTEdge>>(input.size());
    state->pointIdToElbows = std::vector<std::list<ElbowId>>(input.size());

    for (int i = 0; i < input.size(); ++i) {
        stateGeometry->vertexGeometry[i] = RationalRadiusCircle(input[i].point, settings.kelpRadius);
    }

    // Iteratively find the cheapest edge
    // Can probably be sped up (for average case) by using a heap.
    bool found;
    do {
        found = false;
        RoutingGraph::Path cheapestPath;
        double cheapestCost = std::numeric_limits<double>::infinity();
        int cheapestK = -1;
        int cheapestSet1 = -1;
        int cheapestSet2 = -1;
        for (int k = 0; k < input.numCategories(); ++k) {
            auto& dSetsK = dSets[k];
            const auto &pts = input.category(k);
            if (msts[k] >= pts.size() - 1) continue;

            for (int idIndex1 = 0; idIndex1 < pts.size(); ++idIndex1) {
                PointId id1 = pts[idIndex1];
                auto set1 = dSetsK.find_set(idIndex1);
                for (int idIndex2 = idIndex1; idIndex2 < pts.size(); ++idIndex2) {
                    auto set2 = dSetsK.find_set(idIndex2);
                    if (set1 == set2) continue;
                    PointId id2 = pts[idIndex2];
                    auto path = graph.computePath(graph.m_pointToVertex[input[id1].point], graph.m_pointToVertex[input[id2].point]);
                    if (path.path.front() == path.path.back()) {
                        continue;
                    }
                    if (path.weight < cheapestCost) {
                        found = true;
                        cheapestCost = path.weight;
                        cheapestPath = path;
                        cheapestK = k;
                        cheapestSet1 = set1;
                        cheapestSet2 = set2;
                    }
                }
            }
        }
        if (found) {
            const auto& path = cheapestPath.path;
            for (int i = 0; i < path.size() - 1; ++i) {
                renderer.setStroke(Color{100, 100, 255}, 3.0);
                auto [ed, exists] = boost::edge(path[i], path[i + 1], g);
                if (exists) {
                    drawEdge(ed);
                } else {
                    renderer.draw(Segment<Exact>(g[path[i]].point, g[path[i + 1]].point));
                }
            }

            auto topoOld = extractTopology(cheapestPath, g, settings);
            auto& mstE = state->msts[cheapestK].emplace_back(topoOld.source, topoOld.target);
            state->edgeTopology[mstE] = std::move(topoOld);
			auto& topo = state->edgeTopology[mstE];
            state->pointIdToEdges[mstE.first].push_back(mstE);
            state->pointIdToEdges[mstE.second].push_back(mstE);
            stateGeometry->edgeGeometry[mstE] = EdgeGeometry(topo, input, settings);
            auto& geometry = stateGeometry->edgeGeometry[mstE];

            for (auto orbitIt = topo.orbits.begin(); orbitIt != topo.orbits.end(); ++orbitIt) {
                const auto& orbit = *orbitIt;
				auto pId = orbit.pointId;
                state->pointIdToElbows[pId].emplace_back(mstE, orbitIt);
				// Remove all edges and vertices on the orbit
				for (const auto& v : graph.m_circleVertices[pId]) {
					graph.m_pointToVertex.erase(g[v].point);
					boost::clear_vertex(v, g);
					boost::remove_vertex(v, g);
				}
				graph.m_circleVertices[pId].clear();
				graph.m_circleVertices[pId].resize(0);

				// Update object to new radius
				auto changedObject = graph.circleObject(pId);
				changedObject->geom = RationalRadiusCircle(input[pId].point, orbit.outerRadius + settings.edgeWidth / 2);

				auto [eit, eEnd] = boost::edges(g);
				RoutingGraph::Graph::edge_iterator next;
				for (next = eit; eit != eEnd; eit = next) {
					++next;
					if (g[eit->m_source].object->pointId == changedObject->pointId || g[eit->m_target].object->pointId == changedObject->pointId) continue;
					if (auto* rtp = std::get_if<RationalTangent>(&(g[*eit].geom))) {
						if (!RoutingGraph::free(*rtp, *changedObject)) {
							// Remove edge *eit
							boost::remove_edge(*eit, g);
						}
					}
				}

				// Add new tangents
				std::vector<RoutingGraph::Tangent> newTangents;
				for (const auto& obj : graph.m_objects) {
					if (obj == changedObject) continue;
					RoutingGraph::tangents(changedObject, obj, std::back_inserter(newTangents));
				}
				for (const auto& t : newTangents) {
					if (graph.free(t)) {
						graph.addTangentToGraph(t);
					}
				}
			}

			for (int pId = 0; pId < input.size(); ++pId) {
				graph.createCircleEdges(pId);
			}

            graph.removeNonProperlyIntersectedTangents(path.front(), path.back(), geometry.csPolygon());

            msts[cheapestK] += 1;
            dSets[cheapestK].link(cheapestSet1, cheapestSet2);
        }
    } while (found);

    auto [eStart, eEnd] = boost::edges(g);
    for (auto eit = eStart; eit != eEnd; ++eit) {
        renderer.setStroke(Color{0, 0, 0}, 1.0);
        drawEdge(*eit);
    }

    return {state, stateGeometry};
}

EdgeTopology extractTopology(const RoutingGraph::Path& path, RoutingGraph::Graph& g, const Settings& settings) {
    std::list<Orbit> orbits;
    int lastOrbit = -1;
    auto p = path.path;

    for (int i = 0; i < p.size() - 1; ++i) {
        auto [ed, exists] = boost::edge(p[i], p[i + 1], g);
        if (!exists) continue;
        if (auto curveP = std::get_if<CSCurve>(&g[ed].geom)) {
            auto obj = *(g[p[i]].object);
            auto v = obj.pointId;
            if (lastOrbit != v && curveP->is_circular()) {
                lastOrbit = v;
                auto reversed = curveP->source() != pretendOneRootPoint(g[p[i]].point);
                auto orient = reversed ? opposite(curveP->orientation()) : curveP->orientation();
                auto r = std::get<RationalRadiusCircle>(g[p[i]].object->geom).radius;
                orbits.emplace_back(v, r - settings.edgeWidth / 2, r + settings.edgeWidth / 2, orient);
            }
        }
        boost::remove_edge(ed, g);
    }

    return EdgeTopology(g[p.front()].object->pointId, g[p.back()].object->pointId, orbits);
}

CSPolygon rationalTangentToCSPolygon(const RationalTangent& rt, const Settings& settings) {
    InputInstance input({{0, rt.source()}, {0, rt.target()}});
    EdgeTopology topo(0, 1, {});
    EdgeGeometry geom(topo, input, settings);
    return geom.csPolygon();
}
}