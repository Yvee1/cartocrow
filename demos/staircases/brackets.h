#ifndef CARTOCROW_BRACKETS_H
#define CARTOCROW_BRACKETS_H

#include "types.h"
#include "staircase.h"
#include <QCheckBox>

bool supported_by(Segment<K> segment, Segment<K> other);

struct Bracket {
	int start;
	int end;

	std::optional<Polygon<K>> polygon(const Staircase& input) const {
		bool vertical = start % 2 == 0;
		if (start + 1 == end) return std::nullopt;

		std::vector<Point<K>> pts;
		pts.emplace_back(input.m_xs[ceil((start + 1)/2)], input.m_ys[ceil((start+2)/2)]);
		for (int i = start + 1; i < end; i++) {
			pts.emplace_back(input.m_xs[ceil((i+1)/2)], input.m_ys[ceil((i+2)/2)]);
		}
		if (vertical) {
			pts.emplace_back(pts.front().x(), pts.back().y());
		} else {
			pts.emplace_back(pts.back().x(), pts.front().y());
		}

		return Polygon<K>(pts.begin(), pts.end());
	}

	int complexity() const {
		return (end - start - 1) / 2;
	}
};

std::vector<Bracket> brackets(const Staircase& input, const Staircase& simplification);

Bracket bracket(const Staircase& input, const Staircase& simplification, int i);

class BracketPainting : public GeometryPainting {
  public:
	BracketPainting(const std::shared_ptr<Staircase>& input,
	                const std::shared_ptr<Staircase>& simplification,
	                QCheckBox* show_bracket_complexity,
	                QCheckBox* show_bracket_dimensions);
	void paint(GeometryRenderer& renderer) const override;

  private:
	const std::shared_ptr<Staircase> m_input;
	const std::shared_ptr<Staircase> m_simplification;
	QCheckBox* m_show_bracket_complexity;
	QCheckBox* m_show_bracket_dimensions;

};

#endif //CARTOCROW_BRACKETS_H
