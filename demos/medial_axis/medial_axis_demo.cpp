//
// Created by steven on 12/21/23.
//

#include "medial_axis_demo.h"
#include <QApplication>
#include <utility>
#include "cartocrow/core/ipe_reader.h"
#include "cartocrow/renderer/ipe_renderer.h"
#include <ipepath.h>
#include "medial_axis.h"

using namespace cartocrow;
using namespace cartocrow::renderer;

MedialAxisDemo::MedialAxisDemo() {
	setWindowTitle("Medial Axis");

	std::shared_ptr<ipe::Document> document = IpeReader::loadIpeFile("/home/steven/Documents/cartocrow/polygon.ipe");
	ipe::Page* page = document->page(0);
	m_isolines = isolinesInPage(page);

	for (const auto& isoline : m_isolines) {
		addIsolineToVoronoi(isoline);
	}

	m_renderer = new GeometryWidget();
	setCentralWidget(m_renderer);

	recalculate();
}

void MedialAxisDemo::recalculate() {
	auto painting = std::make_shared<MedialAxisPainting>(&m_isolines, &m_voronoi);
	m_renderer->addPainting(painting, "Medial axis painting");
	m_renderer->update();

	IpeRenderer ipeRenderer(painting);
	ipeRenderer.save("/home/steven/Documents/cartocrow/output.ipe");
}

std::vector<Isoline> MedialAxisDemo::isolinesInPage(ipe::Page* page) {
	auto isolines = std::vector<Isoline>();

	for (int i = 0; i < page->count(); i++) {
		auto object = page->object(i);
		if (object->type() != ipe::Object::Type::EPath) continue;
		auto path = object->asPath();
		auto matrix = object->matrix();
		auto shape = path->shape();
		for (int j = 0; j < shape.countSubPaths(); j++) {
			auto subpath = shape.subPath(j);
			if (subpath->type() != ipe::SubPath::Type::ECurve) continue;
			auto curve = subpath->asCurve();

			std::vector<Segment<Inexact>> segments;

			for (int k = 0; k < curve->countSegmentsClosing(); k++) {
				auto segment = curve->segment(k);
				auto start = matrix * segment.cp(0);
				auto end = matrix * segment.last();
				auto cgalSegment = Segment<Inexact>(Point<Inexact>(start.x, start.y), Point<Inexact>(end.x, end.y));
				segments.push_back(cgalSegment);
			}

			isolines.emplace_back(segments);
		}
	}

	return isolines;
}

void MedialAxisDemo::addIsolineToVoronoi(const Isoline& isoline) {
	for (auto const& s : isoline.m_segments) {
		m_voronoi.insert(Site::construct_site_2(s.source(), s.target()));
	}
}

int main(int argc, char* argv[]) {
	QApplication app(argc, argv);
	MedialAxisDemo demo;
	demo.show();
	app.exec();
}

MedialAxisPainting::MedialAxisPainting(std::vector<Isoline>* isolines, VD* voronoi): m_isolines(isolines), m_voronoi(voronoi) {}

void MedialAxisPainting::paint(GeometryRenderer& renderer) const {
	// Draw medial axis
	renderer.setStroke(Color(100, 100, 100), 1);
	renderer.setMode(GeometryRenderer::stroke);
	auto voronoiDrawer = VoronoiDrawer(&renderer);
	draw_skeleton<VoronoiDrawer, Inexact>(m_voronoi->dual(),voronoiDrawer);

	// Draw isolines
	renderer.setMode(GeometryRenderer::stroke | GeometryRenderer::vertices);
	renderer.setStroke(Color(0, 0, 0), 2);
	for (const auto& isoline : *m_isolines) {
		for (const auto& s : isoline.m_segments) {
			renderer.draw(s);
		}
	}
}

VoronoiDrawer::VoronoiDrawer(GeometryRenderer* renderer): m_renderer(renderer) {}

VoronoiDrawer& VoronoiDrawer::operator<<(const Gt::Segment_2& s) {
	m_renderer->draw(s);
	return *this;
}

VoronoiDrawer& VoronoiDrawer::operator<<(const CGAL::Parabola_segment_2<Gt>& p) {
	// Directrix
	auto dir = p.line();
	// Focus
	auto focus = p.center();

	// Roundabout way to obtain start and end of parabolic segment because they are protected -_-
	std::vector<Point<Inexact>> pts;
	p.generate_points(pts, 100000000);
	auto start = pts.front();
	auto end = pts.back();

	// Geometric magic: the intersection of the tangents at points p and q of the parabola is
	// the circumcenter of the focus and the projections of p and q on the directrix.
	auto control = CGAL::circumcenter(focus, dir.projection(start), dir.projection(end));
	auto bezier = BezierCurve(start, control, end);

	m_renderer->draw(bezier);

	return *this;
}

VoronoiDrawer& VoronoiDrawer::operator<<(const Gt::Line_2& l) {
	m_renderer->draw(l);
	return *this;
}

VoronoiDrawer& VoronoiDrawer::operator<<(const Gt::Ray_2& r) {
	m_renderer->draw(r);
	return *this;
}

Isoline::Isoline(std::vector<Segment<Inexact>> segments): m_segments(std::move(segments)) {}