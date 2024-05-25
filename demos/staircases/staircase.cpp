#include "staircase.h"
#include "colors.h"

Staircase::Staircase(std::vector<Number<K>> xs, std::vector<Number<K>> ys):
      m_xs(std::move(xs)), m_ys(std::move(ys)) {
	assert(m_xs.size() == m_ys.size());
}

bool Staircase::supported_by(const Staircase& other) const {
	auto these_steps = edges();
	auto other_steps = other.edges();

	bool good = true;
	for (const auto& s1 : these_steps) {
		bool found = false;
		for (const auto& s2 : other_steps) {
			if (s1.segment.has_on(s2.segment.source()) && s1.segment.has_on(s2.segment.target())) {
				found = true;
				break;
			}
		}
		if (!found) {
			good = false;
			break;
		}
	}

	return good;
}


bool Staircase::is_valid() const {
	if (m_xs.size() != m_ys.size()) return false;

	for (int i = 1; i < m_xs.size(); i++) {
		if (m_xs[i-1] > m_xs[i]) {
			return false;
		}
	}

	for (int i = 1; i < m_ys.size(); i++) {
		if (m_ys[i - 1] > m_ys[i]) {
			return false;
		}
	}

	return true;
}

std::vector<Edge> Staircase::edges() const {
	std::vector<Edge> result;
	for (int i = 0; i < 2 * m_xs.size() - 2; i++) {
		Point<K> p1(m_xs[ceil((i)/2)], m_ys[ceil((i+1)/2)]);
		Point<K> p2(m_xs[ceil((i+1)/2)], m_ys[ceil((i+2)/2)]);

		result.emplace_back(i % 2 == 0, ceil((i + 1) / 2), Segment<K>(p1, p2));
	}
	return result;
}

std::vector<Point<K>> Staircase::corners() const {
	std::vector<Point<K>> result;
	int n = 2 * m_xs.size() - 2;
	for (int i = 0; i < n; i++) {
		Point<K> p1(m_xs[ceil((i)/2)], m_ys[ceil((i+1)/2)]);
		result.push_back(p1);
	}
	Point<K> p1(m_xs[ceil(n/2)], m_ys[ceil((n+1)/2)]);
	result.push_back(p1);

	return result;
}

Polyline<K> Staircase::polyline() const {
	auto pts = corners();
	return {pts.begin(), pts.end()};
}

std::vector<MoveBox> Staircase::moves() const {
	std::vector<MoveBox> boxes;
	for (int i = 0; i < 2 * static_cast<int>(m_xs.size()) - 3; i++) {
		Point<K> top_left(m_xs[floor(i / 2)], m_ys[floor((i + 1) / 2)]);
		Point<K> bottom_right(m_xs[floor((i / 2) + 1)], m_ys[floor((i + 1) / 2 + 1)]);
		boxes.emplace_back(i, Rectangle<K>(top_left, bottom_right));
	}
	return boxes;
}

MoveBox Staircase::move(int i) const {
	Point<K> top_left(m_xs[floor(i / 2)], m_ys[floor((i + 1) / 2)]);
	Point<K> bottom_right(m_xs[floor((i / 2) + 1)], m_ys[floor((i + 1) / 2 + 1)]);
	return { i, Rectangle<K>(top_left, bottom_right) };
}

UniformStaircase::UniformStaircase(int n) : Staircase(std::vector<Number<K>>(n+1), std::vector<Number<K>>(n+1)) {
	std::iota(m_xs.begin(), m_xs.end(), 0);
	std::iota(m_ys.begin(), m_ys.end(), 0);
}

StaircasePainting::StaircasePainting(const std::shared_ptr<Staircase>& staircase, bool light):
      m_staircase(staircase), m_light(light) {}

void StaircasePainting::paint(GeometryRenderer& renderer) const {
	auto& s = *m_staircase;
	if (m_light) {
		renderer.setStroke(CB::red, 1.5);
	} else {
		renderer.setStroke(CB::blue, 2.5);
	}
	draw_staircase(renderer, s);
}

MovesPainting::MovesPainting(const std::shared_ptr<Staircase>& staircase): m_staircase(staircase) {}

void MovesPainting::paint(GeometryRenderer& renderer) const {
	renderer.setMode(GeometryRenderer::fill);
	renderer.setFill(CB::light_red);

	auto moves = m_staircase->moves();
	for (const auto& m : moves) {
		auto& r = m.rectangle;
		Polygon<K> poly;
		poly.push_back(r.vertex(0));
		poly.push_back(r.vertex(1));
		poly.push_back(r.vertex(2));
		poly.push_back(r.vertex(3));
		renderer.draw(poly);
	}
}

void draw_staircase(GeometryRenderer& renderer, const Staircase& s) {
	renderer.setMode(GeometryRenderer::stroke);
	Ray<K> left(Point<K>(s.m_xs.front(), s.m_ys.front()), Vector<K>(-1.0, 0.0));
	renderer.draw(left);
	auto polyline = s.polyline();
	renderer.draw(polyline);
	Ray<K> up(Point<K>(s.m_xs.back(), s.m_ys.back()), Vector<K>(0.0, 1.0));
	renderer.draw(up);
}
