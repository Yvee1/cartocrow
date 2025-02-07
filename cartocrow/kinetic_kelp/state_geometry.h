#ifndef CARTOCROW_STATE_GEOMETRY_H
#define CARTOCROW_STATE_GEOMETRY_H

#include "../core/cs_types.h"

#include "state.h"
#include "input_instance.h"
#include "cartocrow/core/circle_tangents.h"
#include "cartocrow/core/cs_polygon_helpers.h"

#include "kelp.h"

namespace cartocrow::kinetic_kelp {
class Elbow {
public:
    Elbow();
    CSPolygon csPolygon() const;
    CSCurve outerArc() const;
    CSCurve innerArc() const;
private:

};

class Straight {
    Straight();
    CSPolygon csPolygon() const;
private:

};

using Pin = std::variant<Orbit, RationalRadiusCircle>;

class EdgeGeometry {
public:
    EdgeGeometry() = default;
    EdgeGeometry(const EdgeTopology& edgeTopology, const InputInstance& input, const Settings& settings);

    CSPolygon csPolygon() const;
//    Elbow elbow(int i) const;
//    Connector connector(int i) const;
//    int size();
private:
//    EdgeTopology m_edgeTopology;
    CSPolygon m_csPolygon;
    std::vector<Pin> m_pins;
//    std::vector<Straight> m_straights;
};

struct StateGeometry {
    std::map<MSTEdge, EdgeGeometry> edgeGeometry;
    std::map<VertexId, Circle<Exact>> vertexGeometry;
};

StateGeometry stateToGeometry(const State& state, const InputInstance& input, const Settings& settings);

template <class OutputIterator>
void stateGeometrytoKelps(const StateGeometry& stateGeometry, const InputInstance& input, double smoothing, OutputIterator out) {
    std::vector<CSPolygonSet> roughKelps;
    while (roughKelps.size() < input.numCategories()) {
        roughKelps.emplace_back();
    }
    for (const auto& [mstEdge, geometry] : stateGeometry.edgeGeometry) {
        int k = input[mstEdge.first].category;
        CSPolygonSet& roughKelp = roughKelps[k];
        try {
            roughKelp.join(geometry.csPolygon());
        } catch (...) {
            std::cerr << "Problems with edge geometry of edge " << mstEdge.first << " -> " << mstEdge.second << std::endl;
        }
    }
    for (const auto& [vertex, circle] : stateGeometry.vertexGeometry) {
        int k = input[vertex].category;
        CSPolygonSet& roughKelp = roughKelps[k];
        roughKelp.join(circleToCSPolygon(circle));
    }
    for (const auto& roughKelp : roughKelps) {
        std::vector<CSPolygonWithHoles> polygons;
        roughKelp.polygons_with_holes(std::back_inserter(polygons));
        if (polygons.size() != 1) {
            throw std::runtime_error("MST geometry cannot consist of more than one part!");
        }
        *out++ = Kelp(polygons[0], smoothing);
    }
}
}

#endif //CARTOCROW_STATE_GEOMETRY_H
