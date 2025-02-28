#include "pseudotriangulation_painting.h"
#include "cartocrow/circle_segment_helpers/cs_polyline_helpers.h"
#include "cartocrow/circle_segment_helpers/poly_line_gon_intersection.h"

using namespace cartocrow::renderer;

namespace cartocrow::kinetic_kelp {
void PseudotriangulationPainting::paint(GeometryRenderer &renderer) const {
	renderer.setMode(GeometryRenderer::stroke);
	renderer.setStroke(Color(0, 102, 202), 3.0);
	for (const auto& [_, t] : m_ptg->m_tangents) {
		renderer.draw(t.polyline());
	}

	renderer.setStroke(Color(0, 0, 0), 3.0);
	for (const auto& [_, obj] : m_ptg->m_tangentObject) {
		if (auto cp = std::get_if<RationalRadiusCircle>(&obj)) {
			renderer.draw(cp->circle());
		} else {
			auto p = std::get<Point<Exact>>(obj);
			renderer.draw(p);
		}
	}
}

void PseudotriangulationCertificatesPainting::paint(GeometryRenderer &renderer) const {
	for (int pId = 0; pId < m_inputInstance->size(); ++pId) {
		renderer.drawText((*m_inputInstance)[pId].point, std::to_string(pId));
	}

	for (int pId = 0; pId < m_pt->m_pointIdToTangents.size(); ++pId) {
		auto& ts = m_pt->m_pointIdToTangents[pId];

		auto r = m_settings.vertexRadius + (static_cast<int>(m_state->pointIdToElbows[pId].size()) + 0.5) * m_settings.edgeWidth;
		auto center = (*m_inputInstance)[pId].point;
		Circle<Exact> circle(center, r * r);

		RenderPath path;
		for (const auto& t : ts) {
			auto p = m_ptg->tangentEndpoint(*t, pId);
			auto rt = m_ptg->m_tangents[*t];
			auto pl = rt.polyline();
			auto plCS = polylineToCSPolyline(pl);
			std::vector<OneRootPoint> ipts;
			intersectionPoints(plCS, circleToCSPolygon(circle), std::back_inserter(ipts));

			// intersect pl with circle
			if (ipts.empty()) continue;
			if (path.commands().empty()) {
				path.moveTo(approximateOneRootPoint(ipts[0]));
			} else {
				path.lineTo(approximateOneRootPoint(ipts[0]));
			}
		}
		path.close();

		renderer.setStroke(Color(200, 200, 200), 2.0);
		renderer.setMode(GeometryRenderer::stroke | GeometryRenderer::vertices);
		renderer.draw(path);
	}

	for (auto tpCertificate : m_pt->m_tangentEndpointCertificates) {
		auto pId = tpCertificate.pointId;
		auto& t1 = *(tpCertificate.t1);
		auto& t2 = *(tpCertificate.t2);

		auto r = m_settings.vertexRadius + (static_cast<int>(m_state->pointIdToElbows[pId].size()) + 0.5) * m_settings.edgeWidth;
		auto center = (*m_inputInstance)[pId].point;
		Circle<Exact> circle(center, r * r);

		auto rt1 = m_ptg->m_tangents[t1];
		auto pl1 = rt1.polyline();
		auto plCS1 = polylineToCSPolyline(pl1);
		std::vector<OneRootPoint> ipts1;
		intersectionPoints(plCS1, circleToCSPolygon(circle), std::back_inserter(ipts1));

		auto rt2 = m_ptg->m_tangents[t2];
		auto pl2 = rt2.polyline();
		auto plCS2 = polylineToCSPolyline(pl2);
		std::vector<OneRootPoint> ipts2;
		intersectionPoints(plCS2, circleToCSPolygon(circle), std::back_inserter(ipts2));

		if (ipts1.empty()) continue;
		if (ipts2.empty()) continue;

		auto p1 = ipts1[0];
		auto p2 = ipts2[0];

		if (tpCertificate.valid(*m_pt, *m_state, *m_stateGeometry, *m_inputInstance, m_settings)) {
			renderer.setStroke(Color(71, 142, 0), 3.0);
		} else {
			renderer.setStroke(Color(213, 0, 0), 3.0);
		}
		RenderPath path;

		path.moveTo(approximateOneRootPoint(p1));
		path.arcTo(approximate(center), false, approximateOneRootPoint(p2));
		renderer.draw(path);
	}
}
}