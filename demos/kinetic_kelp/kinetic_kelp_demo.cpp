#include "kinetic_kelp_demo.h"

#include <QApplication>
#include <QTimer>
#include <QElapsedTimer>
#include <QDockWidget>
#include <QHBoxLayout>

#include "cartocrow/renderer/svg_renderer.h"

#include "cartocrow/kinetic_kelp/parse_input.h"
#include "cartocrow/kinetic_kelp/moving_cat_point.h"
#include "cartocrow/kinetic_kelp/route_edges.h"
#include "cartocrow/kinetic_kelp/pseudotriangulation.h"
#include "cartocrow/kinetic_kelp/pseudotriangulation_painting.h"
#include "cartocrow/kinetic_kelp/kinetic_kelp_painting.h"
#include "cartocrow/kinetic_kelp/state_geometry_painting.h"

#include "cartocrow/renderer/painting_renderer.h"
#include "colors/colors.h"

#include <fstream>

using namespace cartocrow::renderer;

Box bounds(const std::vector<CatPoint>& points) {
    std::vector<Point<Inexact>> pts;
    for (const auto& [_, p] : points) {
        pts.push_back(approximate(p));
    }
    return CGAL::bbox_2(pts.begin(), pts.end());
}

Box bounds(const std::vector<MovingCatPoint>& movingPoints) {
    std::vector<Point<Inexact>> pts;
    for (const auto& [_, t] : movingPoints) {
        for (const auto& p : t.m_points) {
            pts.push_back(p.point);
        }
    }
    return CGAL::bbox_2(pts.begin(), pts.end());
}

struct DrawSettings {
    std::vector<Color> colors;
    double strokeWidth = 2.0;
};

void drawTrajectory(GeometryRenderer& renderer, const MovingCatPoint& mcp, const DrawSettings& ds) {
    renderer.setStroke(ds.colors[mcp.category], ds.strokeWidth);
    renderer.draw(mcp.trajectory.polyline());
    for (const auto& [_, p] : mcp.trajectory.m_points) {
        renderer.draw(p);
    }
}

void drawMovingCatPoint(GeometryRenderer& renderer, const MovingCatPoint& mcp, double time, const DrawSettings& ds) {
    renderer.setStroke(ds.colors[mcp.category], ds.strokeWidth);
    renderer.draw(mcp.trajectory.posAtTime(time));
}

KineticKelpDemo::KineticKelpDemo() {
	bool saveToSvg = false;

    setWindowTitle("KineticKelp");
    m_renderer = new GeometryWidget();
    m_renderer->setDrawAxes(false);
    setCentralWidget(m_renderer);

    std::string filePath = "data/kinetic_kelp/trajectory.ipe";
    m_input = Input(parseMovingPoints(filePath));

    m_timeControl = new TimeControlToolBar(m_renderer, m_input.timespan().second);

	Settings settings;
	settings.vertexRadius = 10.0;
	settings.edgeWidth = 5.0;

	auto input = std::make_shared<InputInstance>(m_input.instance(m_timeControl->time()));
	auto pr = std::make_shared<PaintingRenderer>();
	auto [stateTopology, stateGeometry] = routeEdges(*input, settings, *pr);

	auto stateGeometryP = std::make_shared<StateGeometryPainting>(stateGeometry);
	m_renderer->addPainting(stateGeometryP, "State geometry");

	auto pr1 = std::make_shared<PaintingRenderer>();
	auto [pt, ptg] = PseudotriangulationGeometry::pseudotriangulationTangents(stateTopology, *stateGeometry);
	auto ptP = std::make_shared<Pseudotriangulation>(pt);
	auto ptgP = std::make_shared<PseudotriangulationGeometry>(ptg);
	auto ptPainting = std::make_shared<PseudotriangulationPainting>(ptP, ptgP);
	m_renderer->addPainting(ptPainting, "Pseudotriangulation");

	KineticKelpPainting::DrawSettings ds;
	ds.colors = {CB::light_blue, CB::light_red, CB::light_green, CB::light_purple, CB::light_orange};
	ds.markRadius = 1.0;
	ds.strokeWidth = 1.0;
	ds.smoothing = 5.0;

	auto kelps = std::make_shared<std::vector<Kelp>>();
	try {
		stateGeometrytoKelps(*stateGeometry, *input, ds.smoothing, std::back_inserter(*kelps));
	} catch (...) {}
	auto kkPainting = std::make_shared<KineticKelpPainting>(kelps, input, ds);
	m_renderer->addPainting(kkPainting, "KineticKelp");

    m_renderer->fitInView(bounds(m_input.movingCatPoints()));

	connect(m_timeControl, &TimeControlToolBar::ticked, [saveToSvg, input, settings, stateGeometry, kelps, ds, ptP, ptgP, kkPainting, this](int tick, double time) {
		*input = m_input.instance(time);
		PaintingRenderer trash;
		auto [newStateTopology, newStateGeometry] = routeEdges(*input, settings, trash);
		*stateGeometry = std::move(*newStateGeometry);

	  	auto [pt, ptg] = PseudotriangulationGeometry::pseudotriangulationTangents(newStateTopology, *stateGeometry);
		*ptP = pt;
	  	*ptgP = ptg;

		kelps->clear();
		try {
			stateGeometrytoKelps(*stateGeometry, *input, ds.smoothing, std::back_inserter(*kelps));
		} catch(...) {}

		if (saveToSvg) {
			SvgRenderer svgRenderer;
			svgRenderer.addPainting(
				[kkPainting](GeometryRenderer& renderer) {
				    kkPainting->paint(renderer);
				},
				"KineticKelp");
			std::stringstream filename;
			filename << "frames/frame-" << std::setfill('0') << std::setw(5) << tick;
			svgRenderer.save(filename.str() + ".svg");
		}
	});
}

void KineticKelpDemo::fitToScreen() {
    Box box = bounds(m_input.movingCatPoints());
    auto delta = CGAL::to_double(8 * m_pointRadius);
    Box expanded(box.xmin() - delta, box.ymin() - delta, box.xmax() + delta, box.ymax() + delta);
    m_renderer->fitInView(expanded);
}

void KineticKelpDemo::resizeEvent(QResizeEvent *event) {
    fitToScreen();
    m_timeControl->resized();
}

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    KineticKelpDemo demo;
    demo.show();
    app.exec();
}
