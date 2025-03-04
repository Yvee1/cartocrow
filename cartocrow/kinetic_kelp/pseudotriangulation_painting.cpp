#include "pseudotriangulation_painting.h"
#include "cartocrow/circle_segment_helpers/cs_polyline_helpers.h"
#include "cartocrow/circle_segment_helpers/poly_line_gon_intersection.h"
#include "state_geometry_painting.h"

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

		auto r = m_settings.kelpRadius + (static_cast<int>(m_state->pointIdToElbows[pId].size()) + 0.5) * m_settings.edgeWidth;
		auto center = (*m_inputInstance)[pId].point;
		Circle<Exact> circle(center, r * r);

		RenderPath path;
		for (const auto& t : ts) {
			auto p = m_ptg->tangentEndpoint(*t, pId);
			if (!m_ptg->m_tangents.contains(*t)) {
				continue;
			}
			auto rt = m_ptg->m_tangents.at(*t);
			if (rt.source() == rt.target()) continue;
			auto pl = rt.polyline();
			auto plCS = polylineToCSPolyline(pl);
			std::vector<OneRootPoint> ipts;
			intersectionPoints(plCS, circleToCSPolygon(circle), std::back_inserter(ipts));

			// intersect pl with circle
            Point<Inexact> interP;
			if (!ipts.empty()) {
                interP = approximateOneRootPoint(ipts[0]);
            } else {
                bool rev = t->target->pointId == pId;
                interP = approximate(rev ? rt.source() : rt.target());
            }
			if (path.commands().empty()) {
				path.moveTo(interP);
			} else {
				path.lineTo(interP);
			}
		}
		path.close();

		renderer.setStroke(Color(200, 200, 200), 2.0);
		renderer.setMode(GeometryRenderer::stroke | GeometryRenderer::vertices);
		renderer.draw(path);
	}

	for (auto certificate : m_pt->m_certificates) {
        if (!std::holds_alternative<Pseudotriangulation::ConsecutiveCertificate>(certificate)) continue;
        auto& tpCertificate = std::get<Pseudotriangulation::ConsecutiveCertificate>(certificate);
		auto pId = tpCertificate.pointId;
		auto& t1 = *(tpCertificate.t1);
		auto& t2 = *(tpCertificate.t2);

		auto r = m_settings.kelpRadius + (static_cast<int>(m_state->pointIdToElbows[pId].size()) + 0.5) * m_settings.edgeWidth;
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

        Point<Inexact> p1;
		if (!ipts1.empty()) {
            p1 = approximateOneRootPoint(ipts1[0]);
        } else {
            bool rev = t1.target->pointId == pId;
            p1 = approximate(rev ? pl1.source() : pl1.target());
        }
        Point<Inexact> p2;
		if (!ipts2.empty()) {
            p2 = approximateOneRootPoint(ipts2[0]);
        } else {
            bool rev = t2.target->pointId == pId;
            p1 = approximate(rev ? pl2.source() : pl2.target());
        }

		if (tpCertificate.valid(*m_pt, *m_state, *m_ptg, *m_inputInstance)) {
			renderer.setStroke(Color(71, 142, 0), 3.0);
		} else {
			renderer.setStroke(Color(213, 0, 0), 3.0);
		}
		RenderPath path;

		path.moveTo(p1);
		path.arcTo(approximate(center), false, p2);
		renderer.draw(path);
	}
}

void CertificateFailurePainting::paint(GeometryRenderer &renderer) const {
	auto ptSP = std::make_shared<Pseudotriangulation>(m_pt);
	auto ptgSP = std::make_shared<PseudotriangulationGeometry>(m_ptg);
	auto stateSP = std::make_shared<State>(m_state);
	auto inputInstanceSP = std::make_shared<InputInstance>(m_inputInstance);

	StateGeometryPainting stateGeometryP(m_stateGeometry);
	PseudotriangulationPainting ptP(ptgSP);
	PseudotriangulationCertificatesPainting ptcP(ptSP, ptgSP, stateSP, inputInstanceSP, m_settings);
	renderer.setStrokeOpacity(100);
	stateGeometryP.paint(renderer);
	ptP.paint(renderer);
	ptcP.paint(renderer);

    renderer.setStrokeOpacity(255);
    renderer.setStroke(Color{255, 0, 0}, 3.0);
    if (auto cc = std::get_if<Pseudotriangulation::ConsecutiveCertificate>(&*m_certificate)) {
        auto pl1 = m_ptg.m_tangents.at(*cc->t1).polyline();
        auto pl2 = m_ptg.m_tangents.at(*cc->t2).polyline();
        renderer.setMode(GeometryRenderer::stroke);
        renderer.draw(pl1);
        renderer.draw(pl2);
    } else if (auto pc = std::get_if<Pseudotriangulation::PointCertificate>(&*m_certificate)) {
        auto pl = m_ptg.m_tangents.at(*pc->t).polyline();
        renderer.setMode(GeometryRenderer::stroke);
        renderer.draw(pl);
    } else {
        throw std::runtime_error("Unhandled certificate type!");
    }
}
}