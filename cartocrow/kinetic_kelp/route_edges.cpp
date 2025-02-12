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
        m_objects.push_back(std::make_shared<Object>(RationalRadiusCircle(p, m_settings.vertexRadius + m_settings.edgeWidth / 2), i));
        m_objects.push_back(std::make_shared<Object>(p, i));
    }
}

bool RoutingGraph::free(const Tangent& t) {
    bool free = true;
    for (const auto& obj : m_objects) {
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
}

void RoutingGraph::addTangentToGraph(const Tangent& t) {
    m_circleVertices.resize(m_input.size());
    int vertexEndpoints = 0;
    GraphV sourceVertex;
    auto& ptv = m_pointToVertex;
    if (auto cp = std::get_if<RationalRadiusCircle>(&(t.source->geom))) {
        auto p = t.geom.source();
        if (ptv.contains(p)) {
            sourceVertex = ptv[p];
        } else {
            sourceVertex = boost::add_vertex(m_g);
            m_g[sourceVertex].point = p;
            m_g[sourceVertex].object = t.source;
            ptv[p] = sourceVertex;
        }
        m_circleVertices[t.source->vertex].push_back(sourceVertex);
    } else {
        sourceVertex = t.source->vertex;
        ++vertexEndpoints;
    }
    GraphV targetVertex;
    if (auto cp = std::get_if<RationalRadiusCircle>(&(t.target->geom))) {
        auto p = t.geom.target();
        if (ptv.contains(p)) return;
        if (ptv.contains(p)) {
            targetVertex = ptv[p];
        } else {
            targetVertex = boost::add_vertex(m_g);
            m_g[targetVertex].point = p;
            m_g[targetVertex].object = t.target;
            ptv[p] = targetVertex;
        }
        m_circleVertices[t.target->vertex].push_back(targetVertex);
    } else {
        targetVertex = t.target->vertex;
        ++vertexEndpoints;
    }
    auto e = boost::add_edge(sourceVertex, targetVertex, m_g);
    m_g[e.first].geom = t.geom;
    m_g[e.first].length = CGAL::sqrt(CGAL::squared_distance(approximate(t.geom.source()), approximate(t.geom.target())));
    if (vertexEndpoints == 0) {
        m_g[e.first].weight = m_g[e.first].length;
    } else if (vertexEndpoints == 1) {
        m_g[e.first].weight = 1000000 + m_g[e.first].length;
    } else {
        m_g[e.first].weight = 2000000 + m_g[e.first].length;
    }
}

RoutingGraph::RoutingGraph(InputInstance  input, Settings settings) : m_input(std::move(input)), m_settings(std::move(settings)) {
    makeRoutingObjects();
    std::vector<Tangent> ts;
    freeTangents(std::back_inserter(ts));

    // Add vertices corresponding to points in the input
    for (int i = 0; i < m_input.size(); ++i) {
        auto v = boost::add_vertex(m_g);
        m_g[v].point = m_input[i].point;
        m_g[v].object = m_objects[2 * i + 1];
        m_pointToVertex[m_input[i].point] = v;
    }

    // Create edges for tangents and vertices on circles
    for (const auto& freeTangent : ts) {
        addTangentToGraph(freeTangent);
    }

    // Sort vertices on circles
    for (int i = 0; i < m_circleVertices.size(); ++i) {
        auto& vs = m_circleVertices[i];
        auto& c = m_input[i].point;
        std::sort(vs.begin(), vs.end(), [&c, this](const auto& v1, const auto& v2) {
            auto p1 = m_g[v1].point;
            auto p2 = m_g[v2].point;
            return (p1-c).direction() < (p2-c).direction();
        });
    }

    // Create edges on circles
    for (int i = 0; i < m_circleVertices.size(); ++i) {
        for (int j = 0; j < m_circleVertices[i].size(); ++j) {
            auto v1 = m_circleVertices[i][j];
            auto v2 = m_circleVertices[i][(j + 1) % m_circleVertices[i].size()];
            auto p1 = m_g[v1].point;
            auto p2 = m_g[v2].point;
            OneRootPoint p1A(p1.x(), p1.y());
            OneRootPoint p2A(p2.x(), p2.y());
            auto c = std::get<RationalRadiusCircle>(m_objects[2 * i]->geom);
            CSCurve curve(c.center, c.radius, CGAL::COUNTERCLOCKWISE, p1A, p2A);
            auto e = boost::add_edge(v1, v2, m_g);
            m_g[e.first].length = approximateLength(curve);
            m_g[e.first].weight = m_g[e.first].length;
            m_g[e.first].geom = curve;
        }
    }
}

RoutingGraph::Path RoutingGraph::computePath(GraphV u, GraphV v) const {
    std::vector<GraphV> predecessors(num_vertices(m_g));
    boost::dijkstra_shortest_paths(m_g, u, boost::weight_map(boost::get(&Edge::weight, m_g))
            .predecessor_map(&predecessors[0]));

    std::vector<GraphV> path;
    GraphV current = v;
    double weight = 0;
    do {
        path.push_back(current);
        auto pre = predecessors[current];
        if (pre == current) break;
        weight += m_g[boost::edge(current, pre, m_g).first].weight;
        current = pre;
    } while (current != u);
    path.push_back(current);
    std::reverse(path.begin(), path.end());
    return {path, weight};
}

void RoutingGraph::removeNonProperlyIntersectedTangents(GraphV u, GraphV v, const CSPolygon& obstacleEdge) {
    auto [eStart, eEnd] = boost::edges(m_g);
    std::vector<RoutingGraph::GraphE> nonProperlyIntersected;
    for (auto eit = eStart; eit != eEnd; ++eit) {
        if (eit->m_source == u || eit->m_source == v || eit->m_target == u || eit->m_target == v) continue;
        if (auto rtP = std::get_if<RationalTangent>(&(m_g[*eit].geom))) {
            CSPolygonSet differenceSet;
            CSPolygonSet intersectionSet;
            CSPolygon tangentEdge = rationalTangentToCSPolygon(*rtP, m_settings);

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
                if (area(pgn) > M_PI * CGAL::square(m_settings.edgeWidth / 2) + M_EPSILON) {
                    ++numberOfLargeDifferences;
                }
            }
            if (numberOfLargeDifferences <= inters.size()) {
                nonProperlyIntersected.push_back(*eit);
            }
        }
    }
    for (auto ed : nonProperlyIntersected) {
        boost::remove_edge(ed, m_g);
    }
}

std::pair<State, std::shared_ptr<StateGeometry>> routeEdges(const InputInstance& input, const Settings& settings, GeometryRenderer& renderer) {
    // Initial graph is now done, we start routing
    RoutingGraph graph(input, settings);

    // Helper function for drawing edges
    auto drawEdge = [&renderer, &graph](RoutingGraph::GraphE e) {
        auto geom = graph.m_g[e].geom;
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
                auto id1 = pts[idIndex1];
                auto set1 = dSetsK.find_set(idIndex1);
                for (int idIndex2 = idIndex1; idIndex2 < pts.size(); ++idIndex2) {
                    auto set2 = dSetsK.find_set(idIndex2);
                    if (set1 == set2) continue;
                    auto id2 = pts[idIndex2];
                    auto path = graph.computePath(id1, id2);
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
                auto [ed, exists] = boost::edge(path[i], path[i + 1], graph.m_g);
                if (exists) {
                    drawEdge(ed);
                } else {
                    renderer.draw(Segment<Exact>(input[path[i]].point, input[path[i + 1]].point));
                }
            }

            auto topo = extractTopology(cheapestPath, graph.m_g, settings);
            auto& mstE = state.msts[cheapestK].emplace_back(topo.source, topo.target);
            state.edgeTopology[mstE] = topo;
            stateGeometry->edgeGeometry[mstE] = EdgeGeometry(topo, input, settings);
            auto& geometry = stateGeometry->edgeGeometry[mstE];

            // probably easier to go over the topology to create the circular arcs
            for (const auto& elbow : geometry.elbows) {
                auto r = elbow.orbit().outerRadius + settings.edgeWidth / 2;
                auto vId = elbow.orbit().vertexId;

                RationalRadiusCircle circle(input[vId].point, r);
//                RationalCircularArc arc(circle,)
                auto newObject = std::make_shared<RoutingGraph::Object>(circle, vId);
                graph.m_objects.push_back(newObject);
                std::vector<RoutingGraph::Tangent> newTangents;
                for (const auto& object : graph.m_objects) {
                    if (object->vertex == elbow.orbit().vertexId && std::holds_alternative<RationalRadiusCircle>(object->geom)) {
//                        tangents(newObject, object, std::back_insert(newTangents));
                    }
                }
                for (const auto& t : newTangents) {
                    auto rev = t.target == newObject;
                    auto ptOnNewObject = rev ? t.geom.target() : t.geom.source();
                    // check ptOnNewObject lies on elbow, essentially.

                    auto v = boost::add_vertex(graph.m_g);
                }
            }

            graph.removeNonProperlyIntersectedTangents(path.front(), path.back(), geometry.csPolygon());

            msts[cheapestK] += 1;
            dSets[cheapestK].link(cheapestSet1, cheapestSet2);
        }
    } while (found);

    auto [eStart, eEnd] = boost::edges(graph.m_g);
    for (auto eit = eStart; eit != eEnd; ++eit) {
        renderer.setStroke(Color{0, 0, 0}, 1.0);
        drawEdge(*eit);
    }

    return {state, stateGeometry};
}

EdgeTopology extractTopology(const RoutingGraph::Path& path, RoutingGraph::Graph& g, const Settings& settings) {
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