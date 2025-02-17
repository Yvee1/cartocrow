#ifndef CARTOCROW_ROUTE_EDGES_H
#define CARTOCROW_ROUTE_EDGES_H

#include <utility>

#include "input_instance.h"
#include "state.h"
#include "state_geometry.h"
#include "cartocrow/renderer/geometry_renderer.h"
#include "cartocrow/circle_segment_helpers/circle_tangents.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/functional/hash.hpp>

namespace std {
template <>
struct hash<cartocrow::Number<cartocrow::Exact>>
{
	std::size_t operator()(const cartocrow::Number<cartocrow::Exact>& x) const
	{
		return hash<double>{}(CGAL::to_double(x));
	}
};

template <>
struct hash<cartocrow::Point<cartocrow::Exact>>
{
	std::size_t operator()(const cartocrow::Point<cartocrow::Exact>& p) const
	{
		// Compute individual hash values for first, second and third
		// http://stackoverflow.com/a/1646913/126995
		std::size_t res = 17;
		res = res * 31 + hash<cartocrow::Number<cartocrow::Exact>>{}(p.x());
		res = res * 31 + hash<cartocrow::Number<cartocrow::Exact>>{}(p.y());
		return res;
	}
};

template <>
struct hash<cartocrow::RationalRadiusCircle>
{
	std::size_t operator()(const cartocrow::RationalRadiusCircle& c) const
	{
		// Compute individual hash values for first, second and third
		// http://stackoverflow.com/a/1646913/126995
		std::size_t res = 17;
		res = res * 31 + hash<cartocrow::Point<cartocrow::Exact>>{}(c.center);
		res = res * 31 + hash<cartocrow::Number<cartocrow::Exact>>{}(c.radius);
		return res;
	}
};
}

namespace cartocrow::kinetic_kelp {
class RoutingGraph {
  public:
    class Object;

    class Tangent {
    public:
        Tangent(RationalTangent geom, std::shared_ptr<Object> source, std::shared_ptr<Object> target) :
                geom(std::move(geom)), source(std::move(source)), target(std::move(target)) {}
        RationalTangent geom;
        std::shared_ptr<Object> source;
        std::shared_ptr<Object> target;
    };

    class Object {
    public:
        Object(std::variant<Point<Exact>, RationalRadiusCircle> geom, PointId pointId) :
                geom(std::move(geom)), pointId(pointId) {};
        bool operator==(const Object& other) const = default;
        std::variant<Point<Exact>, RationalRadiusCircle> geom;
		PointId pointId;
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

    using Graph = boost::adjacency_list<boost::listS, boost::listS, boost::undirectedS, Vertex, Edge>;
    using GraphV = Graph::vertex_descriptor;
    using GraphE = Graph::edge_descriptor;

    struct Path {
        std::vector<GraphV> path;
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
                    *out++ = Tangent(outer->first, one, other);
                    *out++ = Tangent(outer->second, one, other);
                }
                if (inner.has_value()) {
                    *out++ = Tangent(inner->first, one, other);
                    *out++ = Tangent(inner->second, one, other);
                }
            } else {
                auto p2 = std::get<Point<Exact>>(other->geom);
                auto ts = rationalTangents(p2, c1);
                if (ts.has_value()) {
                    *out++ = Tangent(ts->first, other, one);
                    *out++ = Tangent(ts->second, other, one);
                }
            }
		} else {
			auto p1 = std::get<Point<Exact>>(one->geom);
			if (auto p2p = std::get_if<Point<Exact>>(&other->geom)) {
				*out++ = Tangent(RationalTangent(Segment<Exact>(p1, *p2p)), one, other);
			} else {
				tangents(other, one, out);
			}
		}
    }

	static bool free(const RationalTangent& t, const Object& object);
	static bool free(const CSCurve& curve, const Object& object);
    bool free(const Tangent& t);
    RoutingGraph(InputInstance input, Settings settings);

    template <class OutputIterator>
    void freeTangents(OutputIterator out) {
        // Make all tangents between objects
        std::vector<Tangent> allTangents;

        for (auto objId1 = 0; objId1 < m_objects.size(); ++objId1) {
			for (auto objId2 = objId1 + 1; objId2 < m_objects.size(); ++objId2) {
                tangents(m_objects[objId1], m_objects[objId2], std::back_inserter(allTangents));
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

    std::unique_ptr<Graph> m_g;
    void removeNonProperlyIntersectedTangents(GraphV u, GraphV v, const CSPolygon& obstacleEdge);
	bool nonProperlyIntersectedTangent(const RationalTangent& t, const CSPolygon& obstacleEdge);
	void addTangentToGraph(const Tangent& t);
	void createCircleEdges(PointId pointId);
	std::shared_ptr<Object> circleObject(PointId pointId);
	std::shared_ptr<Object> pointObject(PointId pointId);
	std::unordered_map<Point<Exact>, GraphV> m_pointToVertex;
	std::vector<std::vector<GraphV>> m_circleVertices;
	std::vector<std::shared_ptr<Object>> m_objects;
  private:
    Settings m_settings;
    InputInstance m_input;

    void makeRoutingObjects();
};

std::pair<State, std::shared_ptr<StateGeometry>> routeEdges(const InputInstance& input, const Settings& settings, renderer::GeometryRenderer& renderer);

CSPolygon rationalTangentToCSPolygon(const RationalTangent& rt, const Settings& settings);

EdgeTopology extractTopology(const RoutingGraph::Path& path, RoutingGraph::Graph& g, const Settings& settings);
}

#endif //CARTOCROW_ROUTE_EDGES_H
