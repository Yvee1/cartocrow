#ifndef CARTOCROW_STATE_GEOMETRY_H
#define CARTOCROW_STATE_GEOMETRY_H

#include <ranges>

#include "cartocrow/circle_segment_helpers/circle_tangents.h"
#include "cartocrow/circle_segment_helpers/cs_types.h"
#include "cartocrow/circle_segment_helpers/cs_polygon_helpers.h"
#include "cartocrow/circle_segment_helpers/cs_curve_helpers.h"

#include "state.h"
#include "settings.h"
#include "input_instance.h"
#include "kelp.h"

namespace cartocrow::kinetic_kelp {
using Pin = std::variant<Orbit, RationalRadiusCircle>;

struct Straight;

struct ElbowOrTerminal {
	virtual CSPolygon csPolygon() const = 0;
	Pin pin;
	Straight* prev;
	Straight* next;
};

struct Elbow : public ElbowOrTerminal {
	CSPolygon csPolygon() const override;
    CSCurve outerArc() const { return orbit().dir == CGAL::CLOCKWISE ? secondHalf : firstHalf; }
    CSCurve innerArc() const { return orbit().dir == CGAL::CLOCKWISE ? firstHalf : secondHalf; }
	const Orbit& orbit() const { return std::get<Orbit>(pin); }
	CSCurve firstHalf;
	CSCurve secondHalf;
};

struct Terminal : public ElbowOrTerminal {
	CSPolygon csPolygon() const override;
	RationalRadiusCircle circle() const { return std::get<RationalRadiusCircle>(pin); }
	CSCurve curve;
	bool start;
	Straight* straight() const { return start ? next : prev; }
};

struct Straight {
    CSPolygon csPolygon() const;
	RationalTangent firstHalf;
	RationalTangent secondHalf;
	ElbowOrTerminal* prev;
	ElbowOrTerminal* next;
};

template <class OutputIterator>
void rtXMCurves(const RationalTangent& tangent, OutputIterator out) {
	if (auto* uvs = std::get_if<Segment<Exact>>(&tangent.variant)) {
		*out++ = CSXMCurve(uvs->source(), uvs->target());
	} else if (auto* uvsp = std::get_if<std::pair<Segment<Exact>, Segment<Exact>>>(&tangent.variant)) {
		auto [uvs1, uvs2] = *uvsp;
		*out++ = CSXMCurve(uvs1.source(), uvs1.target());
		*out++ = CSXMCurve(uvs2.source(), uvs2.target());
	} else {
		throw std::runtime_error("Impossible: unexpected type in variant");
	}
}

class EdgeGeometry {
public:
    EdgeGeometry() = default;
    EdgeGeometry(const EdgeTopology& edgeTopology, const InputInstance& input, const Settings& settings);

    CSPolygon csPolygon() const {
		std::vector<CSXMCurve> xmCurves;
		auto in = std::back_inserter(xmCurves);
		curveToXMonotoneCurves(startTerminal.curve, in);
		for (const auto& elbow : elbows) {
			const Straight& straight = *(elbow.prev);
			rtXMCurves(straight.firstHalf, in);
			curveToXMonotoneCurves(elbow.firstHalf, in);
		}
		rtXMCurves(endTerminal.prev->firstHalf, in);
		curveToXMonotoneCurves(endTerminal.curve, in);
		for (const auto& elbow : std::ranges::reverse_view(elbows)) {
			const Straight& straight = *(elbow.next);
			rtXMCurves(straight.secondHalf, in);
			curveToXMonotoneCurves(elbow.secondHalf, in);
		}
		rtXMCurves(startTerminal.next->secondHalf, in);
		return {xmCurves.begin(), xmCurves.end()};
	}

  	std::vector<Elbow> elbows;
	std::vector<Straight> straights;
	Terminal startTerminal;
	Terminal endTerminal;
};

class StateGeometry {
public:
	std::map<MSTEdge, EdgeGeometry> edgeGeometry;
	std::map<PointId, RationalRadiusCircle> vertexGeometry;

	Elbow& elbow(ElbowId elbowId) {
		auto& [edge, i] = elbowId;
		return edgeGeometry[edge].elbows[i];
	}
	Straight& straight(StraightId straightId) {
		auto& [edge, i] = straightId;
		return edgeGeometry[edge].straights[i];
	}
	const Elbow& elbow(ElbowId elbowId) const {
        auto& [edge, i] = elbowId;
        return edgeGeometry.at(edge).elbows.at(i);
    }
	const Straight& straight(StraightId straightId) const {
        auto& [edge, i] = straightId;
        return edgeGeometry.at(edge).straights.at(i);
    }

    StateGeometry() = default;
    StateGeometry(const State& state, const InputInstance& input, const Settings& settings);
};

template <class OutputIterator>
void stateGeometrytoKelps(const StateGeometry& stateGeometry, const InputInstance& input, double smoothing, OutputIterator out) {
    std::vector<CSPolygonSet> roughKelps;
    while (roughKelps.size() < input.numCategories()) {
        roughKelps.emplace_back();
    }
    for (const auto& [mstEdge, geometry] : stateGeometry.edgeGeometry) {
        int k = input[mstEdge.first].category;
        CSPolygonSet& roughKelp = roughKelps[k];
        auto edgePoly = geometry.csPolygon();
        if (is_simple(edgePoly)) {
            roughKelp.join(edgePoly);
        } else {
            for (const auto& straight : geometry.straights) {
                roughKelp.join(straight.csPolygon());
            }
            for (const auto& elbow : geometry.elbows) {
                roughKelp.join(elbow.csPolygon());
            }
            roughKelp.join(geometry.startTerminal.csPolygon());
            roughKelp.join(geometry.endTerminal.csPolygon());
        }
    }
    for (const auto& [vertex, circle] : stateGeometry.vertexGeometry) {
        int k = input[vertex].category;
        CSPolygonSet& roughKelp = roughKelps[k];
        roughKelp.join(circleToCSPolygon(circle.circle()));
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
