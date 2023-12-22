//
// Created by steven on 12/22/23.
//

#include "cgal_tutorial.h"
#include <QApplication>
#include <utility>
#include "cartocrow/core/core.h"
#include <CGAL/convex_hull_2.h>

using namespace cartocrow;
using namespace cartocrow::renderer;

int main(int argc, char* argv[]) {
	QApplication app(argc, argv);
	CGALTutorial demo;
	demo.show();
	QApplication::exec();
}

CGALTutorial::CGALTutorial() {
	setWindowTitle("CGAL: Convex hull");

	m_points.push_back(std::make_shared<Point<Inexact>>(0, 0));
	m_points.push_back(std::make_shared<Point<Inexact>>(10, 0));
	m_points.push_back(std::make_shared<Point<Inexact>>(10, 10));
	m_points.push_back(std::make_shared<Point<Inexact>>(6, 5));
	m_points.push_back(std::make_shared<Point<Inexact>>(4, 1));

	m_renderer = new GeometryWidget();
	setCentralWidget(m_renderer);

	for (auto &p : m_points) {
		m_renderer->registerEditable(p);
	}

	connect(m_renderer, &GeometryWidget::edited, [&]() {
		recalculate();
	});

	recalculate();
}

void CGALTutorial::recalculate() {
	m_renderer->clear();

	auto real_points = std::make_shared<std::vector<Point<Inexact>>>();
	real_points->reserve(m_points.size());
	std::transform(m_points.begin(), m_points.end(), std::back_inserter(*real_points), [](auto p) { return *p; });

	m_result.clear();
	CGAL::convex_hull_2(real_points->begin(), real_points->end(), std::back_inserter(m_result));

	auto polygon = std::make_unique<Polygon<Inexact>>(m_result.begin(), m_result.end());
	auto painting = std::make_shared<ConvexHullPainting>(real_points, std::move(polygon));
	m_renderer->addPainting(painting, "Convex hull painting");
	m_renderer->update();
}

void ConvexHullPainting::paint(GeometryRenderer& renderer) const {
	renderer.setMode(GeometryRenderer::stroke | GeometryRenderer::vertices | GeometryRenderer::fill);

	renderer.draw(*m_convex_hull);
	for (auto &p : *m_points) {
		renderer.draw(p);
	}
}
ConvexHullPainting::ConvexHullPainting(std::shared_ptr<std::vector<Point<Inexact>>> points,
                                       std::shared_ptr<Polygon<Inexact>> convex_hull)
    : m_points(std::move(points)), m_convex_hull(std::move(convex_hull)) {}
