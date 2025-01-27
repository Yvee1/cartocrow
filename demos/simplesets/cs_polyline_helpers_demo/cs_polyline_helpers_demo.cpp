#include "cs_polyline_helpers_demo.h"
#include "cartocrow/core/cs_polyline_helpers.h"
#include "cartocrow/renderer/cs_render_helpers.h"
#include "cartocrow/renderer/geometry_widget.h"
#include <QApplication>

using namespace cartocrow;
using namespace cartocrow::renderer;

CSPolylineHelpersDemo::CSPolylineHelpersDemo() {
	setWindowTitle("Circle-segment polyline helpers demo");
	auto renderer = new GeometryWidget();
	renderer->setDrawAxes(false);
	setCentralWidget(renderer);

	Circle<Exact> circle({0, 0}, 1);
	X_monotone_curve_2 arc(circle, {-1, 0}, {1, 0}, CGAL::CLOCKWISE);
	X_monotone_curve_2 ls0({-2, 0}, {-1, 0});
	X_monotone_curve_2 ls1({1, 0}, {1, -1});
	X_monotone_curve_2 ls2({1, -1}, {2, -1});
	std::vector<X_monotone_curve_2> curves({arc, ls1});
	CSPolyline pl(curves.begin(), curves.end());

	std::function<void(GeometryRenderer&)> drawFunc = [pl](GeometryRenderer& renderer) {
		renderer.setMode(GeometryRenderer::stroke);
		auto [extended, source, target] = extend(pl, 1.0, 1);
	  	auto pgn = closeAroundBB(extended, CGAL::COUNTERCLOCKWISE, 1.0, source, target);
	  	renderer.setStroke(Color{255, 0, 0}, 3.0);
		renderer.draw(renderPath(pgn));
	  	renderer.setStroke(Color{0, 0, 255}, 3.0);
	  	renderer.draw(renderPath(extended));
	  	renderer.setStroke(Color{0, 0, 0}, 3.0);
	  	renderer.draw(renderPath(pl));
	};
	renderer->addPainting(drawFunc, "Polyline");
}

int main(int argc, char* argv[]) {
	QApplication app(argc, argv);
	CSPolylineHelpersDemo demo;
	demo.show();
	app.exec();
}
