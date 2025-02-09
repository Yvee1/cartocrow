#include "state_geometry.h"

#include "cartocrow/circle_segment_helpers/circle_tangents.h"
#include "cartocrow/circle_segment_helpers/cs_curve_helpers.h"

namespace cartocrow::kinetic_kelp {
EdgeGeometry::EdgeGeometry(const EdgeTopology& edge, const InputInstance& input, const Settings& settings) {
    auto u = edge.source;
    auto v = edge.target;

    RationalRadiusCircle uRCircle(input[u].point, settings.edgeWidth / 2);
    RationalRadiusCircle vRCircle(input[v].point, settings.edgeWidth / 2);

	// Initialize terminals
	startTerminal.pin = uRCircle;
	startTerminal.start = true;
	endTerminal.pin = vRCircle;
	endTerminal.start = false;

    auto pinToOrientedCircle = [&input](const Pin& pin, bool firstHalf) -> std::pair<RationalRadiusCircle, CGAL::Orientation> {
        if (auto* orbitP = std::get_if<Orbit>(&pin)) {
            auto orient = orbitP->dir;
            bool innerR = firstHalf ? orient == CGAL::CLOCKWISE : orient == CGAL::COUNTERCLOCKWISE;
            auto c = RationalRadiusCircle(input[orbitP->vertexId].point, innerR ? orbitP->innerRadius : orbitP->outerRadius);
            return std::pair(c, orient);
        } else if (auto* circleP = std::get_if<RationalRadiusCircle>(&pin)) {
            auto c = *circleP;
            auto orient = firstHalf ? CGAL::COUNTERCLOCKWISE : CGAL::CLOCKWISE;
            return std::pair(c, orient);
        } else {
            throw std::runtime_error("Impossible: unexpected type in variant.");
        }
    };

    auto createTangent = [&pinToOrientedCircle](const Pin& pin1, const Pin& pin2, bool firstHalf) -> RationalTangent {
        auto [c1, orient1] = pinToOrientedCircle(pin1, firstHalf);
        auto [c2, orient2] = pinToOrientedCircle(pin2, firstHalf);
        bool inner = orient1 != orient2;

        // todo: handle overlapping circles => no tangents
        auto tangents = rationalBitangents(c1, c2, inner);
        bool one = firstHalf ? orient1 == CGAL::COUNTERCLOCKWISE : orient1 == CGAL::CLOCKWISE;
        auto tangent = one ? tangents->first : tangents->second;
        return tangent;
    };

	// Initialize elbows
	std::vector<ElbowOrTerminal*> elbowOrTerminals;
	elbowOrTerminals.push_back(&startTerminal);
	elbows.reserve(edge.orbits.size());
	for (const auto& orbit : edge.orbits) {
		Elbow& elbow = elbows.emplace_back();
		elbow.pin = orbit;
		elbowOrTerminals.push_back(&elbow);
	}
	elbowOrTerminals.push_back(&endTerminal);

	// Compute straights
	straights.reserve(elbowOrTerminals.size() - 1);
	for (int i = 0; i < elbowOrTerminals.size() - 1; ++i) {
		ElbowOrTerminal* prev = elbowOrTerminals[i];
		ElbowOrTerminal* next = elbowOrTerminals[i+1];
		auto fh = createTangent(prev->pin, next->pin, true);
		auto sh = createTangent(next->pin, prev->pin, false);
		Straight& straight = straights.emplace_back();
		straight.firstHalf = fh;
		straight.secondHalf = sh;
		straight.prev = prev;
		prev->next = &straight;
		straight.next = prev;
		next->prev = &straight;
	}

    auto createArc = [](const RationalRadiusCircle& circle, CGAL::Orientation dir, const Point<Exact>& arcSource, const Point<Exact>& arcTarget) {
        OneRootPoint arcSourceORP(arcSource.x(), arcSource.y());
        OneRootPoint arcTargetORP(arcTarget.x(), arcTarget.y());
        CSCurve arc(circle.center, circle.radius, dir, arcSourceORP, arcTargetORP);
		return arc;
    };

	auto oppositeDir = [](CGAL::Orientation dir) {
	  if (dir == CGAL::CLOCKWISE) {
		  return CGAL::COUNTERCLOCKWISE;
	  } else if (dir == CGAL::COUNTERCLOCKWISE) {
		  return CGAL::CLOCKWISE;
	  } else {
		  return CGAL::COLLINEAR;
	  }
	};

	// Compute terminal curves
	auto& straightF = straights.front();
	Point<Exact> arcuSource = straightF.secondHalf.target();
	Point<Exact> arcuTarget = straightF.firstHalf.source();
	startTerminal.curve = createArc(uRCircle, CGAL::COUNTERCLOCKWISE, arcuSource, arcuTarget);

	auto& straightB = straights.back();
	Point<Exact> arcvSource = straightB.firstHalf.target();
	Point<Exact> arcvTarget = straightB.secondHalf.source();
	endTerminal.curve = createArc(vRCircle, CGAL::COUNTERCLOCKWISE, arcvSource, arcvTarget);

	// Compute elbow curves
	for (auto& elbow : elbows) {
		auto& orbit = elbow.orbit();

		Point<Exact> arcSourceFH = elbow.prev->firstHalf.target();
		Point<Exact> arcTargetFH = elbow.next->firstHalf.source();
		bool innerFH = orbit.dir == CGAL::CLOCKWISE;
		RationalRadiusCircle cFH(input[orbit.vertexId].point, innerFH ? orbit.innerRadius : orbit.outerRadius);
		elbow.firstHalf = createArc(cFH, orbit.dir, arcSourceFH, arcTargetFH);

		Point<Exact> arcSourceSH = elbow.next->secondHalf.target();
		Point<Exact> arcTargetSH = elbow.prev->secondHalf.source();
		bool innerSH = orbit.dir == CGAL::COUNTERCLOCKWISE;
		RationalRadiusCircle cSH(input[orbit.vertexId].point, innerSH ? orbit.innerRadius : orbit.outerRadius);
		elbow.secondHalf = createArc(cSH, oppositeDir(orbit.dir), arcSourceSH, arcTargetSH);
	}
}

StateGeometry stateToGeometry(const State& state, const InputInstance& input, const Settings& settings) {
    StateGeometry stateGeometry;
    for (auto [mstEdge, topology] : state.edgeTopology) {
        stateGeometry.edgeGeometry[mstEdge] = EdgeGeometry(topology, input, settings);
    }
    for (int i = 0; i < input.size(); ++i) {
        auto circle = Circle<Exact>(input[i].point, CGAL::square(settings.vertexRadius));
        stateGeometry.vertexGeometry[i] = circle;
    }
    return stateGeometry;
}

CSPolygon Elbow::csPolygon() const {
	auto p1 = next->firstHalf.source();
	auto p2 = next->secondHalf.target();
	auto p3 = prev->secondHalf.source();
	auto p4 = prev->firstHalf.target();

	std::vector<CSXMCurve> xmCurves;
	auto in = std::back_inserter(xmCurves);
	curveToXMonotoneCurves(firstHalf, in);
	*in++ = CSXMCurve(p1, p2);
	curveToXMonotoneCurves(secondHalf, in);
	*in++ = CSXMCurve(p3, p4);

	return {xmCurves.begin(), xmCurves.end()};
}

CSPolygon Straight::csPolygon() const {
	CSPolygon csPolygon;

	auto addTangent = [&csPolygon](const auto& tangent) {
		if (auto* uvs = std::get_if<Segment<Exact>>(&tangent.variant)) {
			CSXMCurve uv_xm(uvs->source(), uvs->target());
			csPolygon.push_back(uv_xm);
			return uvs->target();
		} else if (auto* uvsp = std::get_if<std::pair<Segment<Exact>, Segment<Exact>>>(&tangent.variant)) {
			auto [uvs1, uvs2] = *uvsp;
			csPolygon.push_back({uvs1.source(), uvs1.target()});
			csPolygon.push_back({uvs2.source(), uvs2.target()});
			return uvs2.target();
		} else {
			throw std::runtime_error("Impossible: unexpected type in variant");
		}
	};

	auto p1 = firstHalf.target();
	auto p2 = secondHalf.source();
	auto p3 = secondHalf.target();
	auto p4 = firstHalf.source();
	addTangent(firstHalf);
	csPolygon.push_back({p1, p2});
	addTangent(secondHalf);
	csPolygon.push_back({p3, p4});

	return csPolygon;
}

CSPolygon Terminal::csPolygon() const {
	std::vector<CSXMCurve> xmCurves;
	auto in = std::back_inserter(xmCurves);
	curveToXMonotoneCurves(curve, in);
	auto s = straight();
	if (start) {
		*in++ = {s->firstHalf.source(), s->secondHalf.target()};
	} else {
		*in++ = {s->secondHalf.source(), s->firstHalf.target()};
	}
	return {xmCurves.begin(), xmCurves.end()};
}
}