#ifndef CARTOCROW_STAIRCASE_H
#define CARTOCROW_STAIRCASE_H

#include "types.h"

class OrthoPolygon {
	std::vector<Number<K>> m_xs;
	std::vector<Number<K>> m_ys;
};

struct Edge {
	bool vertical;
	int index;
	Segment<K> segment;
};

struct MoveBox {
	int index;
	Rectangle<K> rectangle;
};

class Staircase {
  public:
	Staircase(std::vector<Number<K>> xs, std::vector<Number<K>> ys);

	std::vector<Number<K>> m_xs;
	std::vector<Number<K>> m_ys;

	[[nodiscard]] Number<K> x(size_t i) const { return m_xs[i]; };
	[[nodiscard]] Number<K> y(size_t i) const { return m_ys[i];};
	[[nodiscard]] Number<K> coord(size_t i) const { if (i % 2 == 0) return m_ys.at(i/2); else return m_xs.at(i/2); };
	[[nodiscard]] std::vector<Edge> edges() const;
	[[nodiscard]] bool is_valid() const;
	[[nodiscard]] bool supported_by(const Staircase& other) const;
	MoveBox move(int i) const;
	std::vector<MoveBox> moves() const;
	[[nodiscard]] size_t num_of_segments() const {
		return m_xs.size() + m_ys.size();
	}

  private:
};

class UniformStaircase : public Staircase {
  public:
	/// Create a uniform staircase with n steps (vertical segments)
	UniformStaircase(int n);
};

class StaircasePainting : public GeometryPainting {
  public:
	StaircasePainting(const std::shared_ptr<Staircase>& staircase, bool light);
	void paint(GeometryRenderer& renderer) const override;

  private:
	const std::shared_ptr<Staircase> m_staircase;
	bool m_light;
};

class MovesPainting : public GeometryPainting {
  public:
	MovesPainting(const std::shared_ptr<Staircase>& staircase);
	void paint(GeometryRenderer& renderer) const override;

  private:
	const std::shared_ptr<Staircase> m_staircase;
};

void draw_staircase(GeometryRenderer& renderer, const Staircase& s);

#endif //CARTOCROW_STAIRCASE_H
