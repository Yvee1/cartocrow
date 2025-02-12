#include "route_edges.h"

#include "cartocrow/circle_segment_helpers/cs_render_helpers.h"
#include "cartocrow/circle_segment_helpers/cs_polyline_helpers.h"
#include "cartocrow/circle_segment_helpers/poly_line_gon_intersection.h"

#include "cartocrow/renderer/ipe_renderer.h"

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/pending/disjoint_sets.hpp>

#include <CGAL/Boolean_set_operations_2.h>

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

std::pair<State, std::shared_ptr<StateGeometry>> routeEdges(const InputInstance& input, const Settings& settings, GeometryRenderer& renderer) {
    // Make routing objects
	std::vector<std::shared_ptr<RoutingObject>> objects;
	for (int i = 0; i < input.catPoints().size(); ++i) {
        auto p = input[i].point;
		objects.push_back(std::make_shared<RoutingObject>(RationalRadiusCircle(p, settings.vertexRadius + settings.edgeWidth / 2), i));
		objects.push_back(std::make_shared<RoutingObject>(p, i));
	}

    // Make all tangents between objects
	std::vector<RoutingTangent> allTangents;

	for (const auto& o1 : objects) {
		for (const auto& o2 : objects) {
            if (o1 == o2) continue;
            tangents(o1, o2, std::back_inserter(allTangents));
		}
		ArrCSTraits traits;
	}

    // Determine which tangents are free, that is, intersect no object.
    auto free = [&objects](const RoutingTangent& t) {
        bool free = true;
        for (const auto& obj : objects) {
            if (obj->vertex == t.source->vertex || obj->vertex == t.target->vertex) continue;
            if (auto rrcP = std::get_if<RationalRadiusCircle>(&obj->geom)) {
                auto rrc = *rrcP;
                auto pl = polylineToCSPolyline(t.geom.polyline());
                auto pgn = circleToCSPolygon(rrc.circle());
                if (!intersection(pl, pgn, true).empty()) {
                    free = false;
                    break;
                }
            }
        }
        return free;
    };

    std::vector<RoutingTangent> freeTangents;
    for (const auto& t : allTangents) {
        if (free(t)) {
            freeTangents.push_back(t);
        }
    }

    // Build routing graph

    // Add vertices corresponding to points in the input
    std::map<Point<Exact>, RoutingGraph::vertex_descriptor> points;
    RoutingGraph g;
    for (int i = 0; i < input.size(); ++i) {
        auto v = boost::add_vertex(g);
        g[v].point = input[i].point;
        g[v].object = objects[2 * i + 1];
        points[input[i].point] = v;
    }

    // Create edges for tangents and vertices on circles
    std::vector<std::vector<RoutingGraph::vertex_descriptor>> circleVertices(input.size());
    for (const auto& freeTangent : freeTangents) {
        int vertexEndpoints = 0;
        RoutingGraph::vertex_descriptor sourceVertex;
        if (auto cp = std::get_if<RationalRadiusCircle>(&(freeTangent.source->geom))) {
            auto p = freeTangent.geom.source();
            if (points.contains(p)) {
                sourceVertex = points[p];
            } else {
                sourceVertex = boost::add_vertex(g);
                g[sourceVertex].point = p;
                g[sourceVertex].object = freeTangent.source;
                points[p] = sourceVertex;
            }
            circleVertices[freeTangent.source->vertex].push_back(sourceVertex);
        } else {
            sourceVertex = freeTangent.source->vertex;
            ++vertexEndpoints;
        }
        RoutingGraph::vertex_descriptor targetVertex;
        if (auto cp = std::get_if<RationalRadiusCircle>(&(freeTangent.target->geom))) {
            auto p = freeTangent.geom.target();
            if (points.contains(p)) continue;
            if (points.contains(p)) {
                targetVertex = points[p];
            } else {
                targetVertex = boost::add_vertex(g);
                g[targetVertex].point = p;
                g[targetVertex].object = freeTangent.target;
                points[p] = targetVertex;
            }
            circleVertices[freeTangent.target->vertex].push_back(targetVertex);
        } else {
            targetVertex = freeTangent.target->vertex;
            ++vertexEndpoints;
        }
        auto e = boost::add_edge(sourceVertex, targetVertex, g);
        g[e.first].geom = freeTangent.geom;
        g[e.first].length = CGAL::sqrt(CGAL::squared_distance(approximate(freeTangent.geom.source()), approximate(freeTangent.geom.target())));
        if (vertexEndpoints == 0) {
            g[e.first].weight = g[e.first].length;
        } else if (vertexEndpoints == 1) {
            g[e.first].weight = 1000000 + g[e.first].length;
        } else {
            g[e.first].weight = 2000000 + g[e.first].length;
        }
    }

    // Sort vertices on circles
    for (int i = 0; i < circleVertices.size(); ++i) {
        auto& vs = circleVertices[i];
        auto& c = input[i].point;
        std::sort(vs.begin(), vs.end(), [&c, &g](const auto& v1, const auto& v2) {
            auto p1 = g[v1].point;
            auto p2 = g[v2].point;
            return (p1-c).direction() < (p2-c).direction();
        });
    }

    // Create edges on circles
    for (int i = 0; i < circleVertices.size(); ++i) {
        for (int j = 0; j < circleVertices[i].size(); ++j) {
            auto v1 = circleVertices[i][j];
            auto v2 = circleVertices[i][(j + 1) % circleVertices[i].size()];
            auto p1 = g[v1].point;
            auto p2 = g[v2].point;
            OneRootPoint p1A(p1.x(), p1.y());
            OneRootPoint p2A(p2.x(), p2.y());
            auto c = std::get<RationalRadiusCircle>(objects[2 * i]->geom);
            CSCurve curve(c.center, c.radius, CGAL::COUNTERCLOCKWISE, p1A, p2A);
            auto e = boost::add_edge(v1, v2, g);
            g[e.first].length = approximateLength(curve);
            g[e.first].weight = g[e.first].length;
            g[e.first].geom = curve;
        }
    }

    // Initial graph is now done, we start routing

    // Helper function for drawing edges
    auto drawEdge = [&renderer, &g](RoutingGraph::edge_descriptor e) {
        auto geom = g[e].geom;
        if (auto rtP = std::get_if<RationalTangent>(&geom)) {
            renderer.draw(rtP->polyline());
        }
        if (auto csCurveP = std::get_if<CSCurve>(&geom)) {
            renderer.draw(renderPath(*csCurveP));
        }
    };

    // Helper function computing paths
    auto computePath = [&g](RoutingGraph::vertex_descriptor u, RoutingGraph::vertex_descriptor v) -> RoutingPath {
        std::vector<RoutingGraph::vertex_descriptor> predecessors(num_vertices(g));
        boost::dijkstra_shortest_paths(g, u, boost::weight_map(boost::get(&RoutingEdge::weight, g))
                .predecessor_map(&predecessors[0]));

        std::vector<RoutingGraph::vertex_descriptor> path;
        RoutingGraph::vertex_descriptor current = v;
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

    State state;
    auto stateGeometry = std::make_shared<StateGeometry>();
    state.msts = std::vector<MST>(input.numCategories());

    for (int i = 0; i < input.size(); ++i) {
        stateGeometry->vertexGeometry[i] = Circle<Exact>(input[i].point, settings.vertexRadius * settings.vertexRadius);
    }

    // Iteratively find the cheapest edge
    // Can probably be sped up (for average case) by using a heap.
    bool found;
    do {
        found = false;
        RoutingPath cheapestPath;
        double cheapestCost = std::numeric_limits<double>::infinity();
        int cheapestK = -1;
        int cheapestSet1 = -1;
        int cheapestSet2 = -1;
        for (int k = 0; k < input.numCategories(); ++k) {
            auto& dSetsK = dSets[k];
            const auto &pts = input.category(k);
            if (msts[k] >= pts.size() - 1) continue;

            for (int idIndex1 = 0; idIndex1 < pts.size(); ++idIndex1) {
                auto id1 = pts[idIndex1];
                auto set1 = dSetsK.find_set(idIndex1);
                for (int idIndex2 = idIndex1; idIndex2 < pts.size(); ++idIndex2) {
                    auto set2 = dSetsK.find_set(idIndex2);
                    if (set1 == set2) continue;
                    auto id2 = pts[idIndex2];
                    auto path = computePath(id1, id2);
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
                    renderer.draw(Segment<Exact>(input[path[i]].point, input[path[i + 1]].point));
                }
            }

            auto topo = extractTopology(cheapestPath, g, settings);
            auto& mstE = state.msts[cheapestK].emplace_back(topo.source, topo.target);
            state.edgeTopology[mstE] = topo;
            stateGeometry->edgeGeometry[mstE] = EdgeGeometry(topo, input, settings);
            auto& geometry = stateGeometry->edgeGeometry[mstE];

            for (const auto& elbow : geometry.elbows) {
                auto r = elbow.orbit().outerRadius + settings.edgeWidth / 2;
                auto vId = elbow.orbit().vertexId;
                RationalRadiusCircle circle(input[vId].point, r);
                auto newObject = std::make_shared<RoutingObject>(circle, vId);
                objects.push_back(newObject);
                std::vector<RoutingTangent> newTangents;
                for (const auto& object : objects) {
                    if (object->vertex == elbow.orbit().vertexId && std::holds_alternative<RationalRadiusCircle>(object->geom)) {
//                        tangents(newObject, object, std::back_insert(newTangents));
                    }
                }
                for (const auto& t : newTangents) {
                    auto rev = t.target == newObject;
                    auto ptOnNewObject = rev ? t.geom.target() : t.geom.source();
                    // check ptOnNewObject lies on elbow, essentially.

                    auto v = boost::add_vertex(g);
                }
            }

            auto [eStart, eEnd] = boost::edges(g);
            std::vector<RoutingGraph::edge_descriptor> nonProperlyIntersected;
            for (auto eit = eStart; eit != eEnd; ++eit) {
                if (eit->m_source == path.front() || eit->m_source == path.back() || eit->m_target == path.front() || eit->m_target == path.back()) continue;
                if (auto rtP = std::get_if<RationalTangent>(&(g[*eit].geom))) {
                    CSPolygonSet differenceSet;
                    CSPolygonSet intersectionSet;
                    CSPolygon obstacleEdge = geometry.csPolygon();
                    CSPolygon tangentEdge = rationalTangentToCSPolygon(*rtP, settings);

                    intersectionSet.join(tangentEdge);
                    intersectionSet.intersection(obstacleEdge);
                    std::vector<CSPolygonWithHoles> inters;
                    intersectionSet.polygons_with_holes(std::back_inserter(inters));

                    if (inters.empty()) continue;

                    differenceSet.join(tangentEdge);
                    differenceSet.difference(obstacleEdge);

                    std::vector<CSPolygonWithHoles> pgnWHs;
                    differenceSet.polygons_with_holes(std::back_inserter(pgnWHs));
                    int numberOfLargeDifferences = 0;
                    for (const auto& pgn : pgnWHs) {
                        if (area(pgn) > M_PI * CGAL::square(settings.edgeWidth / 2) + M_EPSILON) {
                            ++numberOfLargeDifferences;
                        }
                    }
                    if (numberOfLargeDifferences <= inters.size()) {
                        nonProperlyIntersected.push_back(*eit);
                    }
                }
            }
            for (auto ed : nonProperlyIntersected) {
                boost::remove_edge(ed, g);
            }

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

EdgeTopology extractTopology(const RoutingPath& path, RoutingGraph& g, const Settings& settings) {
    std::vector<Orbit> orbits;
    int lastOrbit = -1;
    auto p = path.path;

    for (int i = 0; i < p.size() - 1; ++i) {
        auto [ed, exists] = boost::edge(p[i], p[i + 1], g);
        if (!exists) continue;
        if (auto curveP = std::get_if<CSCurve>(&g[ed].geom)) {
            auto obj = *(g[p[i]].object);
            auto v = obj.vertex;
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

    return EdgeTopology(p.front(), p.back(), orbits);
}

CSPolygon rationalTangentToCSPolygon(const RationalTangent& rt, const Settings& settings) {
    InputInstance input({{0, rt.source()}, {0, rt.target()}});
    EdgeTopology topo(0, 1, {});
    EdgeGeometry geom(topo, input, settings);
    return geom.csPolygon();
}
}