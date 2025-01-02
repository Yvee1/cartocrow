#include "choropleth.h"
#include "../core/arrangement_helpers.h"
#include "../core/centroid.h"
#include "../core/rectangle_helpers.h"

namespace cartocrow::chorematic_map {
using namespace renderer;

PolygonWithHoles<Inexact> transform(const CGAL::Aff_transformation_2<Inexact>& t, const PolygonWithHoles<Inexact>& pwh) {
    Polygon<Inexact> outerT;
    if (!pwh.is_unbounded()) {
        outerT = transform(t, pwh.outer_boundary());
    }
    std::vector<Polygon<Inexact>> holesT;
    for (const auto& h : pwh.holes()) {
        holesT.push_back(transform(t, h));
    }
    return {outerT, holesT.begin(), holesT.end()};
}

void ChoroplethPainting::paint(GeometryRenderer& renderer) const {
    std::cout << m_transformation.is_scaling() << " " << m_transformation.hm(0, 0) << std::endl;
	const auto& arr = *m_choropleth.m_arr;
	for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
		if (!fit->has_outer_ccb()) continue;
		auto region = fit->data();
		if (region.empty() || region == "#") {
            renderer.setMode(0);
            continue;
		} else {
            renderer.setMode(GeometryRenderer::fill);
		}
		std::optional<int> bin = m_choropleth.regionToBin(region);
		Color color = m_noDataColor;
		if (bin.has_value()) {
			if (m_colors.size() <= *bin) {
				std::cerr << "No color specified for bin " << *bin << std::endl;
			} else {
				color = m_colors.at(*bin);
			}
		}
		renderer.setFill(color);
		auto poly = approximate(face_to_polygon_with_holes<Exact>(fit));
		renderer.draw(transform(m_transformation, poly));
		if (m_drawLabels) {
			auto c = centroid(poly);
			renderer.setMode(GeometryRenderer::stroke);
			renderer.setStroke(Color{0, 0, 0}, m_strokeWidth);
            auto label = fit->data().empty() ? "empty" : fit->data();
			renderer.drawText(c.transform(m_transformation), label);
		}
	}
	renderer.setMode(GeometryRenderer::stroke);
	renderer.setStroke(Color{0, 0, 0}, m_strokeWidth);
	for (auto eit = arr.edges_begin(); eit != arr.edges_end(); ++eit) {
		renderer.draw(Segment<Inexact>(approximate(eit->source()->point()), approximate(eit->target()->point())).transform(m_transformation));
	}
}
}