#include "bezier_demo.h"

#include <QApplication>

#include "cartocrow/core/core.h"
#include "cartocrow/core/cubic_bezier.h"

BezierDemo::BezierDemo() {
	setWindowTitle("Bézier demo");

	m_renderer = new GeometryWidget();
	m_renderer->setDrawAxes(false);
	m_renderer->setMinZoom(50.0);
	m_renderer->setMaxZoom(10000.0);
	m_renderer->fitInView(Box(-1, -3, 4, 4));
	setCentralWidget(m_renderer);

	// Segment endpoints
	auto p1 = std::make_shared<Point<Inexact>>(-0.5, 0.3);
	m_renderer->registerEditable(p1);
	auto p2 = std::make_shared<Point<Inexact>>(3.5, 0.3);
	m_renderer->registerEditable(p2);

	// Curve control points
	auto c0 = std::make_shared<Point<Inexact>>(0, 0);
	m_renderer->registerEditable(c0);
	auto c1 = std::make_shared<Point<Inexact>>(1, 0);
	m_renderer->registerEditable(c1);
	auto c2 = std::make_shared<Point<Inexact>>(1, 2);
	m_renderer->registerEditable(c2);
	auto c3 = std::make_shared<Point<Inexact>>(1.5, 1);
	m_renderer->registerEditable(c3);
	auto c4 = std::make_shared<Point<Inexact>>(2, -1);
	m_renderer->registerEditable(c4);
	auto c5 = std::make_shared<Point<Inexact>>(2.5, -0.8);
	m_renderer->registerEditable(c5);
	auto c6 = std::make_shared<Point<Inexact>>(3, 0);
	m_renderer->registerEditable(c6);

	m_renderer->addPainting([p1, p2, c0, c1, c2, c3, c4, c5, c6](GeometryRenderer& renderer) {
		// Define segment, cubic Bézier spline and its extrema, bounding box and inflection points
	  	Segment<Inexact> seg(*p1, *p2);
	  	CubicBezierCurve curve1(*c0, *c1, *c2, *c3);
	  	CubicBezierCurve curve2(*c3, *c4, *c5, *c6);
		CubicBezierSpline spline;
		spline.appendCurve(curve1);
	  	spline.appendCurve(curve2);
		auto [left, bottom, right, top] = spline.extrema();
		Box box = spline.bbox();
		std::vector<CubicBezierSpline::SplinePoint> inflects;
		spline.inflections(std::back_inserter(inflects));

		// Draw curvature lines of the curve
	    renderer.setStroke(Color(155, 50, 255), 1.0);
		for (int curveIndex = 0; curveIndex < spline.numCurves(); ++curveIndex) {
			for (int i = 0; i <= 200; ++i) {
				double t = i / 200.0;
				auto n = spline.normal({curveIndex, t});
				n /= sqrt(n.squared_length());
				auto p = spline.position({curveIndex, t});
				renderer.draw(Segment<Inexact>(p, p + n * spline.curvature({curveIndex, t}) / 5));
			}
		}

		// Draw bounding box of the spline
		renderer.setMode(GeometryRenderer::stroke | GeometryRenderer::fill);
	  	renderer.setStroke(Color(0, 120, 215), 1.0);
	  	renderer.setFill(Color(0, 120, 215));
		renderer.setFillOpacity(5);
		renderer.draw(box);
	  	renderer.setFillOpacity(255);

	  	// Draw the spline itself and the segment
	  	renderer.setMode(GeometryRenderer::stroke);
	    renderer.setStroke(Color(0, 0, 0), 3.0);
		renderer.draw(spline);
		renderer.draw(seg);
		renderer.draw(*p1);
	  	renderer.draw(*p2);
	  	renderer.draw(*c0);
	  	renderer.draw(*c3);
	    renderer.draw(*c6);

	  	// Draw the control points in grey
	  	renderer.setStroke(Color(200, 200, 200), 3.0);
	  	renderer.draw(*c1);
	  	renderer.draw(*c2);
	  	renderer.draw(*c4);
	  	renderer.draw(*c5);

	  	// Draw the extrema
	  	renderer.setStroke(Color(0, 120, 215), 1.0);
	  	renderer.draw(left.point);
	  	renderer.draw(bottom.point);
	  	renderer.draw(right.point);
	  	renderer.draw(top.point);

	  	// Draw the inflection points
	  	renderer.setStroke(Color(155, 50, 255), 1.0);
		for (const auto& inflect : inflects) {
			renderer.draw(inflect.point);
		}

		// Draw intersections of spline with line segment
	  	renderer.setStroke(Color(200, 0, 0), 1.0);
	  	std::vector<CubicBezierSpline::SplinePoint> inters;
	  	spline.intersections(seg, std::back_inserter(inters));
	  	for (const auto& inter : inters) {
			  renderer.draw(inter.point);
	  	}
	}, "Bézier spline");
}

int main(int argc, char* argv[]) {
	QApplication app(argc, argv);
	BezierDemo demo;
	demo.show();
	app.exec();
}
