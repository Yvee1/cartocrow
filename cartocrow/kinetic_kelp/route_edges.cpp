#include "route_edges.h"

#include "cartocrow/circle_segment_helpers/cs_render_helpers.h"
#include "cartocrow/circle_segment_helpers/cs_polyline_helpers.h"
#include "cartocrow/circle_segment_helpers/poly_line_gon_intersection.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>

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

State routeEdges(const InputInstance& input, const Settings& settings, GeometryRenderer& renderer) {
	std::vector<std::shared_ptr<RoutingObject>> objects;
	for (int i = 0; i < input.catPoints().size(); ++i) {
        auto p = input[i].point;
		objects.push_back(std::make_shared<RoutingObject>(RationalRadiusCircle(p, settings.vertexRadius + settings.edgeWidth / 2), i));
		objects.push_back(std::make_shared<RoutingObject>(p, i));
	}

	std::vector<RoutingTangent> allTangents;

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

	for (const auto& o1 : objects) {
		for (const auto& o2 : objects) {
            if (o1 == o2) continue;
            tangents(o1, o2, std::back_inserter(allTangents));
		}
		ArrCSTraits traits;
	}
    std::vector<RoutingTangent> freeTangents;
    for (const auto& t : allTangents) {
        if (free(t)) {
            freeTangents.push_back(t);
        }
    }

    // build graph
    using RoutingGraph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, RoutingVertex, RoutingEdge>;

    std::map<Point<Exact>, RoutingGraph::vertex_descriptor> points;
    RoutingGraph g;
    for (int i = 0; i < input.size(); ++i) {
        auto v = boost::add_vertex(g);
        g[v].point = input[i].point;
        g[v].object = objects[2 * i + 1];
        points[input[i].point] = v;
    }
    std::vector<std::vector<RoutingGraph::vertex_descriptor>> circleVertices(input.size());
    for (const auto& freeTangent : freeTangents) {
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
        }
        auto e = boost::add_edge(sourceVertex, targetVertex, g);
        g[e.first].geom = freeTangent.geom;
    }

    for (int i = 0; i < circleVertices.size(); ++i) {
        auto& vs = circleVertices[i];
        auto& c = input[i].point;
        std::sort(vs.begin(), vs.end(), [&c, &g](const auto& v1, const auto& v2) {
            auto p1 = g[v1].point;
            auto p2 = g[v2].point;
            return (p1-c).direction() < (p2-c).direction();
        });
    }

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
            g[e.first].geom = curve;
        }
    }
    auto [eStart, eEnd] = boost::edges(g);
    for (auto eit = eStart; eit != eEnd; ++eit) {
        auto& e = g[*eit];
        if (auto rtP = std::get_if<RationalTangent>(&e.geom)) {
            e.weight = CGAL::sqrt(CGAL::squared_distance(approximate(rtP->source()), approximate(rtP->target())));
            renderer.draw(rtP->polyline());
        }
        if (auto csCurveP = std::get_if<CSCurve>(&e.geom)) {
            e.weight = approximateLength(*csCurveP);
            renderer.draw(renderPath(*csCurveP));
        }
    }

    auto drawEdge = [&renderer, &g](RoutingGraph::edge_descriptor e) {
        auto geom = g[e].geom;
        if (auto rtP = std::get_if<RationalTangent>(&geom)) {
            renderer.draw(rtP->polyline());
        }
        if (auto csCurveP = std::get_if<CSCurve>(&geom)) {
            renderer.draw(renderPath(*csCurveP));
        }
    };

    auto computePath = [&g, &objects, &free](RoutingGraph::vertex_descriptor u, RoutingGraph::vertex_descriptor v) -> RoutingPath {
        auto uObj = objects[2 * u + 1];
        auto vObj = objects[2 * v + 1];
        auto uPnt = std::get<Point<Exact>>(uObj->geom);
        auto vPnt = std::get<Point<Exact>>(vObj->geom);
        RationalTangent uvGeom(Segment<Exact>(uPnt, vPnt));
        RoutingTangent rt(uvGeom, uObj, vObj, PointPoint);
        if (free(rt)) {
            return {std::vector({u, v}), CGAL::sqrt(CGAL::squared_distance(approximate(uPnt), approximate(vPnt)))};
        }

        std::vector<RoutingGraph::vertex_descriptor> predecessors(num_vertices(g));
        boost::dijkstra_shortest_paths(g, u, boost::weight_map(boost::get(&RoutingEdge::weight, g))
                .predecessor_map(&predecessors[0]));

        std::vector<RoutingGraph::vertex_descriptor> path;
        RoutingGraph::vertex_descriptor current = v;
        double weight = 0;
        do {
            path.push_back(current);
            auto pre = predecessors[current];
            weight += g[boost::edge(current, pre, g).first].weight;
            current = pre;
        } while (current != u);
        path.push_back(current);
        std::reverse(path.begin(), path.end());
        return {path, weight};
    };

    struct MSTVertex {
        VertexId id;
    };
    using MSTGraph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, MSTVertex, RoutingPath>;
    std::vector<MSTGraph> gMSTs(input.numCategories());
    std::vector<std::vector<MSTGraph::edge_descriptor>> msts(input.numCategories());
    for (int k = 0; k < input.numCategories(); ++k) {
        MSTGraph& gMST = gMSTs[k];
        const auto& pts = input.category(k);
        for (auto id : pts) {
            auto v = boost::add_vertex(gMST);
            gMST[v].id = id;
        }
        for (RoutingGraph::vertex_descriptor i = 0; i < pts.size(); ++i) {
            for (RoutingGraph::vertex_descriptor j = i + 1; j < pts.size(); ++j) {
                auto e = boost::add_edge(gMST[i].id, gMST[j].id, gMST);
                gMST[e.first] = computePath(gMST[i].id, gMST[j].id);
            }
        }
        boost::kruskal_minimum_spanning_tree(gMST, std::back_inserter(msts[k]), boost::weight_map(boost::get(&RoutingPath::weight, gMST)));
    }

    State state;
    for (int k = 0; k < input.numCategories(); ++k) {
        auto& mst = msts[k];
        auto& gMST = gMSTs[k];

        auto& stateMst = state.msts.emplace_back();
        for (const auto &mstEdge: mst) {
            const auto& path = gMST[mstEdge].path;
            auto mstE = stateMst.emplace_back(path.front(), path.back());
            std::vector<Orbit> orbits;
            int lastOrbit = -1;

            for (int i = 0; i < path.size() - 1; ++i) {
                renderer.setStroke(Color{100, 100, 255}, 3.0);
                auto [ed, exists] = boost::edge(path[i], path[i + 1], g);
                if (exists) {
                    drawEdge(ed);
                } else {
                    renderer.draw(Segment<Exact>(input[path[i]].point, input[path[i + 1]].point));
                }
                if (!exists) continue;
                if (auto curveP = std::get_if<CSCurve>(&g[ed].geom)) {
                    if (curveP->is_circular()) {
                        auto obj = *(g[path[i]].object);
                        auto v = obj.vertex;
                        if (lastOrbit == v) continue;
                        lastOrbit = v;
                        auto reversed = curveP->source() != pretendOneRootPoint(g[path[i]].point);
                        auto orient = reversed ? opposite(curveP->orientation()) : curveP->orientation();
                        auto r = std::get<RationalRadiusCircle>(g[path[i]].object->geom).radius;
                        orbits.emplace_back(v, r - settings.edgeWidth / 2, r + settings.edgeWidth / 2, orient);
                    }
                }
            }

            state.edgeTopology[mstE] = EdgeTopology(mstE.first, mstE.second, orbits);
            for (const auto &v: path) {
                renderer.draw(g[v].point);
            }
        }
    }

    return state;
}
}