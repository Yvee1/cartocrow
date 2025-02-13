#ifndef CARTOCROW_ROUTE_EDGES_H
#define CARTOCROW_ROUTE_EDGES_H

#include <utility>

#include "input_instance.h"
#include "state.h"
#include "state_geometry.h"
#include "cartocrow/renderer/geometry_renderer.h"
#include "cartocrow/circle_segment_helpers/circle_tangents.h"

#include <boost/graph/adjacency_list.hpp>

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

	auto operator<=>(const RationalCircularArc& other) const = default;
};

bool circlePointLiesOnArc(const Point<Exact>& point, const RationalCircularArc& arc);

class RoutingGraph {
  public:
    class Object;

    class Tangent {
    public:
        Tangent(RationalTangent geom, std::shared_ptr<Object> source, std::shared_ptr<Object> target, TangentType type) :
                geom(std::move(geom)), source(std::move(source)), target(std::move(target)), type(type) {}
        RationalTangent geom;
        std::shared_ptr<Object> source;
        std::shared_ptr<Object> target;
        TangentType type;
    };

    class Object {
    public:
        Object(std::variant<Point<Exact>, RationalRadiusCircle, RationalCircularArc> geom, VertexId vertex) :
                geom(std::move(geom)), vertex(vertex) {};
        bool operator==(const Object& other) const = default;
        std::variant<Point<Exact>, RationalRadiusCircle, RationalCircularArc> geom;
        VertexId vertex;
    };

    class Vertex {
    public:
        Vertex() = default;
        Vertex(std::shared_ptr<Object> object, Point<Exact> point) : object(std::move(object)), point(std::move(point)) {};
        std::shared_ptr<Object> object;
        Point<Exact> point;
    };

    class Edge {
    public:
        Edge() = default;
        Edge(std::variant<RationalTangent, CSCurve> geom, double weight) : geom(std::move(geom)), weight(weight) {};
        std::variant<RationalTangent, CSCurve> geom;
        double weight;
        double length;
    };

    using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Vertex, Edge>;
    using GraphV = Graph::vertex_descriptor;
    using GraphE = Graph::edge_descriptor;

    struct Path {
        std::vector<long unsigned int> path;
        double weight;
    };

    EdgeTopology extractTopology(const Path& p, const Settings& settings);

	// TangentType is incorrect but not used? So remove? todo
    template <class OutputIterator>
    static void tangents(const std::shared_ptr<Object> one, const std::shared_ptr<Object>& other, OutputIterator out) {
        if (auto cp1 = std::get_if<RationalRadiusCircle>(&one->geom)) {
            auto c1 = *cp1;
            if (auto cp2 = std::get_if<RationalRadiusCircle>(&other->geom)) {
                auto c2 = *cp2;
                auto outer = rationalBitangents(c1, c2, false);
                auto inner = rationalBitangents(c1, c2, true);
                if (outer.has_value()) {
                    *out++ = Tangent(outer->first, one, other, Outer1);
                    *out++ = Tangent(outer->second, one, other, Outer2);
                }
                if (inner.has_value()) {
                    *out++ = Tangent(inner->first, one, other, Inner1);
                    *out++ = Tangent(inner->second, one, other, Inner2);
                }
            } else if (auto cap2 = std::get_if<RationalCircularArc>(&other->geom)) {
				auto ca2 = *cap2;
				auto outer = rationalBitangents(c1, ca2.circle, false);
				auto inner = rationalBitangents(c1, ca2.circle, true);
				if (outer.has_value()) {
					auto [t1, t2] = *outer;
					if (circlePointLiesOnArc(t1.target(), ca2)) {
						*out++ = Tangent(t1, one, other, Outer1);
					}
					if (circlePointLiesOnArc(t2.target(), ca2)) {
						*out++ = Tangent(t2, one, other, Outer2);
					}
				}
				if (inner.has_value()) {
					auto [t1, t2] = *inner;
					if (circlePointLiesOnArc(t1.target(), ca2)) {
						*out++ = Tangent(t1, one, other, Inner1);
					}
					if (circlePointLiesOnArc(t2.target(), ca2)) {
						*out++ = Tangent(t2, one, other, Inner2);
					}
				}
			} else {
                auto p2 = std::get<Point<Exact>>(other->geom);
                auto ts = rationalTangents(p2, c1);
                if (ts.has_value()) {
                    *out++ = Tangent(ts->first, other, one, PointCircle1);
                    *out++ = Tangent(ts->second, other, one, PointCircle2);
                }
            }
        } else if (auto cap1 = std::get_if<RationalCircularArc>(&one->geom)) {
			auto ca1 = *cap1;
			if (auto cap2 = std::get_if<RationalCircularArc>(&other->geom)) {
				auto ca2 = *cap2;
				auto outer = rationalBitangents(ca1.circle, ca2.circle, false);
				auto inner = rationalBitangents(ca1.circle, ca2.circle, true);
				if (outer.has_value()) {
					auto [t1, t2] = *outer;
					if (circlePointLiesOnArc(t1.source(), ca1) && circlePointLiesOnArc(t1.target(), ca2)) {
						*out++ = Tangent(t1, one, other, Outer1);
					}
					if (circlePointLiesOnArc(t2.source(), ca1) && circlePointLiesOnArc(t2.target(), ca2)) {
						*out++ = Tangent(t2, one, other, Outer2);
					}
				}
				if (inner.has_value()) {
					auto [t1, t2] = *inner;
					if (circlePointLiesOnArc(t1.source(), ca1) && circlePointLiesOnArc(t1.target(), ca2)) {
						*out++ = Tangent(t1, one, other, Inner1);
					}
					if (circlePointLiesOnArc(t2.source(), ca1) && circlePointLiesOnArc(t2.target(), ca2)) {
						*out++ = Tangent(t2, one, other, Inner2);
					}
				}
			} else if (auto p2p = std::get_if<Point<Exact>>(&other->geom)) {
				auto ts = rationalTangents(ca1.circle, *p2p);
				if (ts.has_value()) {
					auto [t1, t2] = *ts;
					if (circlePointLiesOnArc(t1.source(), ca1)) {
						*out++ = Tangent(t1, one, other, CirclePoint1);
					}
					if (circlePointLiesOnArc(t2.source(), ca1)) {
						*out++ = Tangent(t2, one, other, CirclePoint2);
					}
				}
			} else {
				tangents(other, one, out);
			}
		} else {
			auto p1 = std::get<Point<Exact>>(one->geom);
			if (auto p2p = std::get_if<Point<Exact>>(&other->geom)) {
				*out++ = Tangent(RationalTangent(Segment<Exact>(p1, *p2p)), one, other, PointPoint);
			} else {
				tangents(other, one, out);
			}
		}
    }

    bool free(const Tangent& t);
    RoutingGraph(InputInstance input, Settings settings);

    template <class OutputIterator>
    void freeTangents(OutputIterator out) {
        // Make all tangents between objects
        std::vector<Tangent> allTangents;

        for (const auto& o1 : m_objects) {
            for (const auto& o2 : m_objects) {
                if (o1 == o2) continue;
                tangents(o1, o2, std::back_inserter(allTangents));
            }
        }

        // Determine which tangents are free, that is, intersect no object.
        for (const auto& t : allTangents) {
            if (free(t)) {
                *out++ = t;
            }
        }
    }

    Path computePath(GraphV u, GraphV v) const;

    Graph m_g;
    std::vector<std::shared_ptr<Object>> m_objects;
    void removeNonProperlyIntersectedTangents(GraphV u, GraphV v, const CSPolygon& obstacleEdge);
	bool nonProperlyIntersectedTangent(const RationalTangent& t, const CSPolygon& obstacleEdge);
	void addTangentToGraph(const Tangent& t);
  private:
    Settings m_settings;
    InputInstance m_input;
    std::map<Point<Exact>, GraphV> m_pointToVertex;
    std::vector<std::vector<GraphV>> m_circleVertices;
	std::map<RationalCircularArc, std::vector<GraphV>> m_arcVertices;

    void makeRoutingObjects();
};

std::pair<State, std::shared_ptr<StateGeometry>> routeEdges(const InputInstance& input, const Settings& settings, renderer::GeometryRenderer& renderer);

CSPolygon rationalTangentToCSPolygon(const RationalTangent& rt, const Settings& settings);

EdgeTopology extractTopology(const RoutingGraph::Path& path, RoutingGraph::Graph& g, const Settings& settings);
}

#endif //CARTOCROW_ROUTE_EDGES_H
