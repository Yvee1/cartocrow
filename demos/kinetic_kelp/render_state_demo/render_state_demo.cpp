#include "render_state_demo.h"

#include <QApplication>

#include "cartocrow/renderer/geometry_widget.h"
#include "cartocrow/renderer/painting_renderer.h"
#include "cartocrow/kinetic_kelp/state_geometry_painting.h"
#include "cartocrow/kinetic_kelp/kinetic_kelp_painting.h"
#include "cartocrow/kinetic_kelp/route_edges.h"
#include "cartocrow/kinetic_kelp/pseudotriangulation.h"

#include "../colors/colors.h"

using namespace cartocrow;
using namespace cartocrow::renderer;
using namespace cartocrow::kinetic_kelp;

RenderStateDemo::RenderStateDemo() {
    setWindowTitle("KineticKelp: Initialization");
    m_renderer = new GeometryWidget();
    m_renderer->setDrawAxes(false);
    setCentralWidget(m_renderer);

	m_initialCatPoints = {
	    {0, {-100, 50}},
	    {0, {0, 40}},
	    {0, {-50, 71}},
	    {1, {-25, -10}},
	    {1, {-75, 2}},
	    {2, {-125, 0}},
	    {2, {25, 0}},
	    {3, {-275, 18}},
	    {3, {175, 18}}
	};
	for (const auto& [_, p] : m_initialCatPoints) {
		m_pointPointers.push_back(std::make_shared<Point<Inexact>>(approximate(p)));
		m_renderer->registerEditable(m_pointPointers.back());
	}

	connect(m_renderer, &GeometryWidget::edited, [this]() {
		recalculate();
	});

	recalculate();
}

void RenderStateDemo::recalculate() {
	m_renderer->clear();

	Settings settings;
	settings.vertexRadius = 10.0;
	settings.edgeWidth = 5.0;

	std::vector<CatPoint> catPoints;
	for (int i = 0; i < m_pointPointers.size(); ++i) {
		catPoints.emplace_back(m_initialCatPoints[i].category, pretendExact(*m_pointPointers[i]));
	}

	auto input = std::make_shared<InputInstance>(catPoints);

	auto pr = std::make_shared<PaintingRenderer>();
	auto [stateTopology, stateGeometry] = routeEdges(*input, settings, *pr);
	m_renderer->addPainting(pr, "routeEdges");

    auto stateGeometryP = std::make_shared<StateGeometryPainting>(stateGeometry);
    m_renderer->addPainting(stateGeometryP, "State geometry");

    auto pr1 = std::make_shared<PaintingRenderer>();
    auto [pt, ptg] = PseudotriangulationGeometry::pseudotriangulationTangents(stateTopology, *stateGeometry);

    m_renderer->addPainting([pt, ptg](GeometryRenderer& renderer) {
        renderer.setMode(GeometryRenderer::stroke);
        renderer.setStroke(Color(0, 102, 202), 3.0);
        for (const auto& [_, t] : ptg.m_tangents) {
            renderer.draw(t.polyline());
        }

//        renderer.setMode(GeometryRenderer::stroke | GeometryRenderer::fill);
        renderer.setStroke(Color(0, 0, 0), 3.0);
//        renderer.setFill(Color(200, 200, 200));
        for (const auto& [_, obj] : ptg.m_tangentObject) {
            if (auto cp = std::get_if<RationalRadiusCircle>(&obj)) {
                renderer.draw(cp->circle());
            } else {
                auto p = std::get<Point<Exact>>(obj);
                renderer.draw(p);
            }
        }
    }, "Tangents");

	KineticKelpPainting::DrawSettings ds;
	ds.colors = {CB::light_blue, CB::light_red, CB::light_green, CB::light_purple, CB::light_orange};
	ds.markRadius = 1.0;
	ds.strokeWidth = 1.0;
	ds.smoothing = 5.0;

	try {
		auto kelps = std::make_shared<std::vector<Kelp>>();
		stateGeometrytoKelps(*stateGeometry, *input, ds.smoothing, std::back_inserter(*kelps));

		auto kkPainting = std::make_shared<KineticKelpPainting>(kelps, input, ds);
		m_renderer->addPainting(kkPainting, "KineticKelp");
	} catch (...) {

	}
}

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    RenderStateDemo demo;
    demo.show();
    app.exec();
}
