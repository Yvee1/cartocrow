#include "route_edges.h"

#include "cartocrow/circle_segment_helpers/circle_tangents.h"
#include "cartocrow/circle_segment_helpers/cs_render_helpers.h"

using namespace cartocrow::renderer;

namespace cartocrow::kinetic_kelp {
void routeEdges(const InputInstance& input, const Settings& settings, GeometryRenderer& renderer) {
	using Object = std::variant<RationalRadiusCircle, Point<Exact>>;
	std::vector<Object> objects;
	for (const auto& [_, p] : input.catPoints()) {
		objects.push_back(RationalRadiusCircle(p, settings.vertexRadius + settings.edgeWidth / 2));
		objects.push_back(p);
	}

	std::vector<RationalTangent> tangents;

	std::function<void(const Object&, const Object&)> addTangents;
	addTangents = [&tangents, &addTangents](const Object& o1, const Object& o2) {
		if (auto cp1 = std::get_if<RationalRadiusCircle>(&o1)) {
			auto c1 = *cp1;
			if (auto cp2 = std::get_if<RationalRadiusCircle>(&o2)) {
				auto c2 = *cp2;
				auto outer = rationalBitangents(c1, c2, false);
				auto inner = rationalBitangents(c1, c2, true);
				if (outer.has_value()) {
					tangents.push_back(outer->first);
					tangents.push_back(outer->second);
				}
				if (inner.has_value()) {
					tangents.push_back(inner->first);
					tangents.push_back(inner->second);
				}
			} else {
				auto p2 = std::get<Point<Exact>>(o2);
				auto ts = rationalTangents(p2, c1);
				if (ts.has_value()) {
					tangents.push_back(ts->first);
					tangents.push_back(ts->second);
				}
			}
		} else {
			auto p1 = std::get<Point<Exact>>(o1);
			if (auto p2p = std::get_if<Point<Exact>>(&o2)) {
				auto p2 = *p2p;
				tangents.push_back(RationalTangent(Segment<Exact>(p1, p2)));
			} else {
				addTangents(o2, o1);
			}
		}
	};

	for (const auto& o1 : objects) {
		for (const auto& o2 : objects) {
			addTangents(o1, o2);
		}
		ArrCSTraits traits;
	}
	for (const auto& tangent : tangents) {
		renderer.draw(tangent.polyline());
	}
}
}