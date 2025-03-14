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
			if (!m_ptg->m_tangents.contains(*t)) {
				continue;
			}
			auto rt = m_ptg->m_tangents.at(*t);
			if (rt.source() == rt.target()) continue;
			auto pl = rt.polyline();
			auto plCS = polylineToCSPolyline(pl);

            bool reversed = t->target->pointId == pId;

            auto [extended, meh, bleh] = approximateExtend(plCS, 100.0, 0);
            std::vector<OneRootPoint> ipts;
            intersectionPoints(extended, circleToCSPolygon(circle), std::back_inserter(ipts));

            renderer.setMode(GeometryRenderer::stroke);

            Point<Inexact> p;
            if (!ipts.empty()) {
                p = approximateOneRootPoint(*std::min_element(ipts.begin(), ipts.end(), [&](const auto& ipt1, const auto& ipt2) {
                    return CGAL::squared_distance(approximateOneRootPoint(ipt1), approximate(CGAL::midpoint(rt.target(), rt.source()))) <=
                           CGAL::squared_distance(approximateOneRootPoint(ipt2), approximate(CGAL::midpoint(rt.target(), rt.source())));
                }));
            } else {
                p = approximate(reversed ? pl.source() : pl.target());
            }
			if (path.commands().empty()) {
				path.moveTo(p);
			} else {
				path.lineTo(p);
			}
		}
		path.close();

		renderer.setStroke(Color(200, 200, 200), 2.0);
		renderer.setMode(GeometryRenderer::stroke | GeometryRenderer::vertices);
		renderer.draw(path);
	}

    for (const auto& tObj : m_pt->m_tangentObjects) {
        if (m_ptg->m_tangentObject.contains(*tObj)) {
            auto& g = m_ptg->m_tangentObject[*tObj];
            Point<Exact> p;
            if (auto pointP = std::get_if<Point<Exact>>(&g)) {
                p = *pointP;
            } else {
                p = std::get<RationalRadiusCircle>(g).center;
            }
            if (tObj->type != Pseudotriangulation::Circle) {
                renderer.setStroke(Color(100, 100, 100), 2.0);
                renderer.drawText(p, name(tObj->type));
            }
        }
    }

	for (auto certificate : m_pt->m_certificates) {
        if (auto ccP = std::get_if<Pseudotriangulation::ConsecutiveCertificate>(&certificate)) {
            auto &tpCertificate = *ccP;
            auto pId = tpCertificate.pointId;
            auto &t1 = *(tpCertificate.t1);
            auto &t2 = *(tpCertificate.t2);
            bool reversed1 = t1.target->pointId == pId;
            bool reversed2 = t2.target->pointId == pId;

            auto r = m_settings.kelpRadius + (static_cast<int>(m_state->pointIdToElbows[pId].size()) + 0.5) * m_settings.edgeWidth;
            auto center = (*m_inputInstance)[pId].point;
            Circle<Exact> circle(center, r * r);

            if (!m_ptg->m_tangents.contains(t1)) continue;
            if (!m_ptg->m_tangents.contains(t2)) continue;
            auto rt1 = m_ptg->m_tangents[t1];
            auto pl1 = rt1.polyline();
            auto plCS1 = polylineToCSPolyline(pl1);
            auto [extended1, meh, bleh] = approximateExtend(plCS1, 100.0, 0);
            std::vector<OneRootPoint> ipts1;
            intersectionPoints(extended1, circleToCSPolygon(circle), std::back_inserter(ipts1));

            auto rt2 = m_ptg->m_tangents[t2];
            auto pl2 = rt2.polyline();
            auto plCS2 = polylineToCSPolyline(pl2);
            auto [extended2, _, idc] = approximateExtend(plCS2, 100.0, 0);
            std::vector<OneRootPoint> ipts2;
            intersectionPoints(extended2, circleToCSPolygon(circle), std::back_inserter(ipts2));

            Point<Inexact> p1;
            if (!ipts1.empty()) {
                p1 = approximateOneRootPoint(*std::min_element(ipts1.begin(), ipts1.end(), [&](const auto& ipt1, const auto& ipt2) {
                    return CGAL::squared_distance(approximateOneRootPoint(ipt1), approximate(CGAL::midpoint(rt1.target(), rt1.source()))) <=
                           CGAL::squared_distance(approximateOneRootPoint(ipt2), approximate(CGAL::midpoint(rt1.target(), rt1.source())));
                }));
            } else {
                p1 = approximate(reversed1 ? pl1.source() : pl1.target());
            }
            Point<Inexact> p2;
            if (!ipts2.empty()) {
                p2 = approximateOneRootPoint(*std::min_element(ipts2.begin(), ipts2.end(), [&](const auto& ipt1, const auto& ipt2) {
                    return CGAL::squared_distance(approximateOneRootPoint(ipt1), approximate(CGAL::midpoint(rt2.target(), rt2.source()))) <=
                           CGAL::squared_distance(approximateOneRootPoint(ipt2), approximate(CGAL::midpoint(rt2.target(), rt2.source())));
                }));
            } else {
                p2 = approximate(reversed2 ? pl2.source() : pl2.target());
            }

            if (tpCertificate.valid(*m_pt, *m_state, *m_ptg, *m_inputInstance)) {
                renderer.setStroke(Color(71, 142, 0), 3.0);
            } else {
                renderer.setStroke(Color(213, 0, 0), 3.0);
            }
            RenderPath path;

            renderer.setMode(GeometryRenderer::stroke | GeometryRenderer::vertices);
            path.moveTo(p1);
            path.arcTo(approximate(center), false, p2);
            renderer.draw(path);
        } else if (auto pcP = std::get_if<Pseudotriangulation::PointCertificate>(&certificate)) {
            auto t = *pcP->t;
            if (!m_ptg->m_tangents.contains(t)) continue;
            auto rt = m_ptg->m_tangents[t];

            auto pId = pcP->pointId;

            auto r = m_settings.kelpRadius + (static_cast<int>(m_state->pointIdToElbows[pId].size()) + 0.5) * m_settings.edgeWidth;
            auto center = (*m_inputInstance)[pId].point;
            Circle<Exact> circle(center, r * r);

            bool reversed = t.target->pointId == pId;

            auto pl = rt.polyline();
            auto plCS = polylineToCSPolyline(pl);
            auto [extended, meh, bleh] = approximateExtend(plCS, 100.0, 0);
            std::vector<OneRootPoint> ipts;
            intersectionPoints(extended, circleToCSPolygon(circle), std::back_inserter(ipts));

            renderer.setMode(GeometryRenderer::stroke);

            Point<Inexact> p;
            if (!ipts.empty()) {
                p = approximateOneRootPoint(*std::min_element(ipts.begin(), ipts.end(), [&](const auto& ipt1, const auto& ipt2) {
                    return CGAL::squared_distance(approximateOneRootPoint(ipt1), approximate(CGAL::midpoint(rt.target(), rt.source()))) <=
                           CGAL::squared_distance(approximateOneRootPoint(ipt2), approximate(CGAL::midpoint(rt.target(), rt.source())));
                }));
            } else {
                p = approximate(reversed ? pl.source() : pl.target());
            }

            if (pcP->valid(*m_pt, *m_state, *m_ptg, *m_inputInstance)) {
                renderer.setStroke(Color(71, 142, 0), 3.0);
            } else {
                renderer.setStroke(Color(213, 0, 0), 3.0);
            }
            auto rtEndpoint = approximate(reversed ? pl.target() : pl.source());
            renderer.draw(Segment<Inexact>(rtEndpoint, p));
        }
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
        if (!m_ptg.m_tangents.contains(*cc->t1)) return;
        if (!m_ptg.m_tangents.contains(*cc->t2)) return;
        auto pl1 = m_ptg.m_tangents.at(*cc->t1).polyline();
        auto pl2 = m_ptg.m_tangents.at(*cc->t2).polyline();
        renderer.setMode(GeometryRenderer::stroke);
        renderer.draw(pl1);
        renderer.draw(pl2);
    } else if (auto pc = std::get_if<Pseudotriangulation::PointCertificate>(&*m_certificate)) {
        if (!m_ptg.m_tangents.contains(*pc->t)) return;
        auto pl = m_ptg.m_tangents.at(*pc->t).polyline();
        renderer.setMode(GeometryRenderer::stroke);
        renderer.draw(pl);
    } else {
        // we don't draw other certificates for now.
//        throw std::runtime_error("Unhandled certificate type!");
    }
}
}