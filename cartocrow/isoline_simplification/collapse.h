//
// Created by steven on 1/23/24.
//

#ifndef CARTOCROW_COLLAPSE_H
#define CARTOCROW_COLLAPSE_H

#include "types.h"
#include "cartocrow/renderer/geometry_painting.h"
#include "ipeshape.h"
#include "ipegeo.h"

namespace cartocrow::isoline_simplification {
class SlopeLadder {
  public:
	SlopeLadder() = default;
	std::deque<Gt::Segment_2> m_rungs;
	std::unordered_map<CGAL::Sign, Gt::Point_2> m_cap;
	std::vector<Gt::Point_2> m_collapsed;
	double m_cost = 0.0;
	bool m_valid = true;
	bool m_old = false;
	bool m_intersects = false;
	void compute_cost(const PointToPoint& p_prev, const PointToPoint& p_next);
};

class LadderCollapse {
  public:
	virtual void operator()(SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next) = 0;
	virtual std::shared_ptr<renderer::GeometryPainting> painting(const SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next) = 0;
};

//typedef std::function<void(SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next)> LadderCollapse;
typedef std::function<Gt::Point_2(Gt::Point_2& s, Gt::Point_2& t, Gt::Point_2& u, Gt::Point_2& v, Gt::Line_2& l)> RungCollapse;

Gt::Point_2 min_sym_diff_point(Gt::Point_2 s, Gt::Point_2 t, Gt::Point_2 u, Gt::Point_2 v, Gt::Line_2 l);
Gt::Point_2 projected_midpoint(Gt::Point_2 s, Gt::Point_2 t, Gt::Point_2 u, Gt::Point_2 v, Gt::Line_2 l);

//LadderCollapse spline_collapse(const RungCollapse& rung_collapse, int repititions);
class SplineCollapse : public LadderCollapse {
  public:
	SplineCollapse(int repetitions, int samples = 50);
	void operator()(SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next) override;
	std::shared_ptr<renderer::GeometryPainting> painting(const SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next) override;

	int m_repetitions;
	int m_samples;

	std::vector<ipe::Vector> controls_from_intersections(const std::vector<Gt::Line_2>& lines,
																	     const std::optional<ipe::Vector>& start,
																	     const std::vector<ipe::Vector>& control_points,
																	     const std::optional<ipe::Vector>& end) const;
	std::vector<ipe::Bezier> controls_to_beziers(const std::vector<ipe::Vector>& control_points) const;
	std::optional<Gt::Point_2> intersection(const std::vector<ipe::Bezier>& bzs, const Gt::Line_2& l) const;
	double cost(const SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next, const std::vector<ipe::Vector>& new_vertices) const;
};

class SplineCollapsePainting : public renderer::GeometryPainting {
  public:
	SplineCollapsePainting(const SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next, SplineCollapse spline_collapse);
	void paint(renderer::GeometryRenderer &renderer) const override;

  private:
	const SlopeLadder& m_ladder;
	const PointToPoint& m_p_prev;
	const PointToPoint& m_p_next;
	const SplineCollapse m_spline_collapse;
};

//void midpoint_collapse(SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next);

class PointCollapsePainting : public renderer::GeometryPainting {
  public:
	PointCollapsePainting(const SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next);
	void paint(renderer::GeometryRenderer &renderer) const override;

  private:
	const SlopeLadder& m_ladder;
	const PointToPoint& m_p_prev;
	const PointToPoint& m_p_next;
};

class MidpointCollapse : public LadderCollapse {
  public:
	MidpointCollapse() = default;
	void operator()(SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next) override;
	std::shared_ptr<renderer::GeometryPainting> painting(const SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next) override;
};

class MinSymDiffCollapse : public LadderCollapse {
  public:
	MinSymDiffCollapse() = default;
	void operator()(SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next) override;
	std::shared_ptr<renderer::GeometryPainting> painting(const SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next) override;
};

class HarmonyLineCollapse : public LadderCollapse {
  public:
	explicit HarmonyLineCollapse(int samples = 50);
	void operator()(SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next) override;
	std::shared_ptr<renderer::GeometryPainting> painting(const SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next) override;

	static std::pair<Gt::Point_2, bool> new_vertex(const Gt::Line_2& harmony_line, const Gt::Point_2& s, const Gt::Point_2& t, const Gt::Point_2& u, const Gt::Point_2& v);

	int m_samples;
};

class LineSplineHybridCollapse : public LadderCollapse {
  public:
	LineSplineHybridCollapse(SplineCollapse spline_collapse, HarmonyLineCollapse line_collapse);
	void operator()(SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next) override;
	std::shared_ptr<renderer::GeometryPainting> painting(const SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next) override;

	SplineCollapse m_spline_collapse;
	HarmonyLineCollapse m_line_collapse;

  private:
	static bool do_line_collapse(const SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next);
};

class HarmonyLinePainting : public renderer::GeometryPainting {
  public:
	HarmonyLinePainting(const SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next, const HarmonyLineCollapse& line_collapse);
	void paint(renderer::GeometryRenderer &renderer) const override;

	const int m_samples;
	const SlopeLadder& m_ladder;
	const PointToPoint& m_p_prev;
	const PointToPoint& m_p_next;
};

//void min_sym_diff_collapse(SlopeLadder& ladder, const PointToPoint& p_prev, const PointToPoint& p_next);

typedef std::unordered_map<Gt::Point_2, std::vector<std::shared_ptr<SlopeLadder>>> PointToSlopeLadders;
typedef std::unordered_map<Gt::Segment_2, std::vector<std::shared_ptr<SlopeLadder>>> EdgeToSlopeLadders;

Gt::Line_2 area_preservation_line(Gt::Point_2 s, Gt::Point_2 t, Gt::Point_2 u, Gt::Point_2 v);
double symmetric_difference(const Gt::Point_2& s, const Gt::Point_2& t, const Gt::Point_2& u, const Gt::Point_2& v, const Gt::Point_2& p);
double signed_area(const std::vector<Gt::Point_2>& pts);
double area(const std::vector<Gt::Point_2>& pts);
bool point_order_on_line(Gt::Line_2 l, Gt::Point_2 a, Gt::Point_2 b);
}
#endif //CARTOCROW_COLLAPSE_H
