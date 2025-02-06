#include "state_geometry.h"

#include "cartocrow/core/circle_tangents.h"
#include "cartocrow/core/cs_curve_helpers.h"
#include "cartocrow/core/cs_polygon_helpers.h"

namespace cartocrow::kinetic_kelp {
CSPolygon edgeToGeometry(const EdgeTopology& edge, const InputInstance& input, const Settings& settings) {
    auto u = edge.source;
    auto v = edge.target;

    Circle<Exact> uCircle(input[u].point, CGAL::square(settings.edgeWidth / 2));
    Circle<Exact> vCircle(input[v].point, CGAL::square(settings.edgeWidth / 2));
    RationalRadiusCircle uRCircle(input[u].point, settings.edgeWidth / 2);
    RationalRadiusCircle vRCircle(input[v].point, settings.edgeWidth / 2);

    CSPolygon polygon;

    auto addTangent = [](auto& csPolygon, const auto& tangent) {
        if (auto* uvs = std::get_if<Segment<Exact>>(&tangent)) {
            CSXMCurve uv_xm(uvs->source(), uvs->target());
            csPolygon.push_back(uv_xm);
            return uvs->target();
        } else if (auto* uvsp = std::get_if<std::pair<Segment<Exact>, Segment<Exact>>>(&tangent)) {
            auto [uvs1, uvs2] = *uvsp;
            csPolygon.push_back({uvs1.source(), uvs1.target()});
            csPolygon.push_back({uvs2.source(), uvs2.target()});
            return uvs2.target();
        } else {
            throw std::runtime_error("Impossible: unexpected type in variant");
        }
    };

    std::vector<std::pair<RationalTangent, RationalRadiusCircle>> firstHalf;
    std::vector<std::pair<RationalTangent, RationalRadiusCircle>> secondHalf;

    using Pin = std::variant<Orbit, RationalRadiusCircle>;

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

    auto createTangent = [&input, &pinToOrientedCircle](const Pin& pin1, const Pin& pin2, bool firstHalf) {
        auto [c1, orient1] = pinToOrientedCircle(pin1, firstHalf);
        auto [c2, orient2] = pinToOrientedCircle(pin2, firstHalf);
        bool inner = orient1 != orient2;

        // todo: handle overlapping circles => no tangents
        auto tangents = rationalTangents(c1, c2, inner);
        bool one = firstHalf ? orient1 == CGAL::COUNTERCLOCKWISE : orient1 == CGAL::CLOCKWISE;
        auto tangent = one ? tangents->first : tangents->second;
        return std::pair(tangent, c2);
    };

    std::vector<Pin> pins;
    pins.push_back(uRCircle);
    for (const auto& orbit : edge.orbits) {
        pins.push_back(orbit);
    }
    pins.push_back(vRCircle);

    for (int i = 0; i < pins.size() - 1; ++i) {
        firstHalf.push_back(createTangent(pins[i], pins[i+1], true));
    }
    for (int i = pins.size() - 1; i >= 1; --i) {
        secondHalf.push_back(createTangent(pins[i], pins[i - 1], false));
    }

    auto rtSource = [](const RationalTangent& rt) {
        if (auto* uvs = std::get_if<Segment<Exact>>(&rt)) {
            return uvs->source();
        } else if (auto* uvsp = std::get_if<std::pair<Segment<Exact>, Segment<Exact>>>(&rt)) {
            return uvsp->first.source();
        } else {
            throw std::runtime_error("Impossible: unexpected type in variant.");
        }
    };

    auto rtTarget = [](const RationalTangent& rt) {
        if (auto* uvs = std::get_if<Segment<Exact>>(&rt)) {
            return uvs->target();
        } else if (auto* uvsp = std::get_if<std::pair<Segment<Exact>, Segment<Exact>>>(&rt)) {
            return uvsp->second.target();
        } else {
            throw std::runtime_error("Impossible: unexpected type in variant.");
        }
    };

    CSPolygon result;

    auto addArc = [&result](const RationalRadiusCircle& circle, CGAL::Orientation dir, const Point<Exact>& arcSource, const Point<Exact>& arcTarget) {
        OneRootPoint arcSourceORP(arcSource.x(), arcSource.y());
        OneRootPoint arcTargetORP(arcTarget.x(), arcTarget.y());
        CSCurve arc(circle.center, circle.radius, dir, arcSourceORP, arcTargetORP);
        std::vector<CSXMCurve> arc_xms;
        curveToXMonotoneCurves(arc, std::back_inserter(arc_xms));
        result.insert(arc_xms.begin(), arc_xms.end());
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

    if (!firstHalf.empty() && !secondHalf.empty()) {
        for (int i = 0; i < firstHalf.size(); ++i) {
            auto [tangent, circle] = firstHalf[i];
            addTangent(result, tangent);
            Point<Exact> arcSource = rtTarget(tangent);
            if (i + 1 < firstHalf.size()) {
                Point<Exact> arcTarget = rtSource(firstHalf[i + 1].first);
                auto pin = pins[i + 1];
                if (auto *pinP = std::get_if<Orbit>(&pin)) {
                    addArc(circle, pinP->dir, arcSource, arcTarget);
                }
            }
        }
        Point<Exact> arcvSource = rtTarget(firstHalf.back().first);
        Point<Exact> arcvTarget = rtSource(secondHalf.front().first);
        addArc(vRCircle, CGAL::COUNTERCLOCKWISE, arcvSource, arcvTarget);
        for (int i = 0; i < secondHalf.size(); ++i) {
            auto [tangent, circle] = secondHalf[i];
            addTangent(result, tangent);
            auto arcSource = rtTarget(tangent);

            if (i + 1 < secondHalf.size()) {
                auto arcTarget = rtSource(secondHalf[i + 1].first);
                auto pin = pins[pins.size() - i - 2];
                if (auto *pinP = std::get_if<Orbit>(&pin)) {
                    addArc(circle, oppositeDir(pinP->dir), arcSource, arcTarget);
                }
            }
        }
        Point<Exact> arcuSource = rtTarget(secondHalf.back().first);
        Point<Exact> arcuTarget = rtSource(firstHalf.front().first);
        addArc(uRCircle, CGAL::COUNTERCLOCKWISE, arcuSource, arcuTarget);
    }

    return result;
}

StateGeometry stateToGeometry(const State& state, const InputInstance& input, const Settings& settings) {
    StateGeometry stateGeometry;
    while (stateGeometry.mstGeometry.size() < input.numCategories()) {
        stateGeometry.mstGeometry.emplace_back();
    }
    for (auto [mstEdge, topology] : state.mstEdgeTopology) {
        auto geom = edgeToGeometry(topology, input, settings);
        stateGeometry.edgeGeometry[mstEdge] = geom;
        int k = input[mstEdge.first].category;
        CSPolygonSet& mstGeom = stateGeometry.mstGeometry[k];
        try {
            mstGeom.join(geom);
        } catch (...) {
            std::cerr << "Problems with edge geometry of edge " << mstEdge.first << " -> " << mstEdge.second << std::endl;
        }
    }
    for (int i = 0; i < input.size(); ++i) {
        auto circle = Circle<Exact>(input[i].point, CGAL::square(settings.pointRadius));
        stateGeometry.vertexGeometry[i] = circle;
        stateGeometry.mstGeometry[input[i].category].join(circleToCSPolygon(circle));
    }
    return stateGeometry;
}
}