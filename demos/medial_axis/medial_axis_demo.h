//
// Created by steven on 12/21/23.
//

#ifndef CARTOCROW_MEDIAL_AXIS_DEMO_H
#define CARTOCROW_MEDIAL_AXIS_DEMO_H

#include <QMainWindow>
#include "cartocrow/core/core.h"
#include "cartocrow/renderer/geometry_painting.h"
#include "cartocrow/renderer/geometry_widget.h"
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Segment_Delaunay_graph_2.h>
#include <CGAL/Segment_Delaunay_graph_adaptation_policies_2.h>
#include <CGAL/Segment_Delaunay_graph_adaptation_traits_2.h>
#include <CGAL/Segment_Delaunay_graph_traits_2.h>
#include "cartocrow/core/ipe_reader.h"
#include <ipepath.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>

//typedef CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt    K;
typedef CGAL::Exact_predicates_inexact_constructions_kernel    K;
//typedef CGAL::Segment_Delaunay_graph_traits_2<K>                       Gt;
typedef CGAL::Segment_Delaunay_graph_filtered_traits_2<K, CGAL::Field_with_sqrt_tag>  Gt;
typedef CGAL::Segment_Delaunay_graph_2<Gt>                             SDG2;
typedef CGAL::Segment_Delaunay_graph_adaptation_traits_2<SDG2>         AT;
typedef AT::Site_2                                 Site;
//typedef CGAL::Segment_Delaunay_graph_degeneracy_removal_policy_2<SDG2> AP;
//typedef CGAL::Voronoi_diagram_2<SDG2, AT, AP>      VD;

using namespace cartocrow;
using namespace cartocrow::renderer;

class Isoline {
  public:
	Isoline(std::vector<Point<K>> points, bool closed);

	std::vector<Point<K>> m_points;
	bool m_closed;

	std::variant<Polyline<K>, Polygon<K>> m_drawing_representation;

	Polyline<K> m_polyline;

	[[nodiscard]] Polyline<K>::Edge_iterator edges_begin() const {
		return m_polyline.edges_begin();
	}

	[[nodiscard]] Polyline<K>::Edge_iterator edges_end() const {
		return m_polyline.edges_end();
	}
};

class MedialAxisDemo : public QMainWindow {
	Q_OBJECT

  public:
	MedialAxisDemo();
	void recalculate(bool voronoi, int target);
	void addIsolineToVoronoi(const Isoline& isoline);

  private:
	std::vector<Isoline> m_isolines;
	std::vector<Isoline> m_cgal_simplified;
	SDG2 m_delaunay;
	cartocrow::renderer::GeometryWidget* m_renderer;
};

std::vector<Isoline> isolinesInPage(ipe::Page* page);

class MedialAxisPainting : public cartocrow::renderer::GeometryPainting {
  public:
	MedialAxisPainting(SDG2& delaunay);

  protected:
	void paint(cartocrow::renderer::GeometryRenderer& renderer) const override;

  private:
	SDG2& m_delaunay;
};

class IsolinePainting : public cartocrow::renderer::GeometryPainting {
  public:
	IsolinePainting(std::vector<Isoline>& isolines);

  protected:
	void paint(cartocrow::renderer::GeometryRenderer& renderer) const override;

  private:
	std::vector<Isoline>& m_isolines;
};

class VoronoiDrawer {
  public:
	cartocrow::renderer::GeometryRenderer* m_renderer;

	explicit VoronoiDrawer(cartocrow::renderer::GeometryRenderer* renderer);

	VoronoiDrawer& operator<<(const Gt::Segment_2& s);
	VoronoiDrawer& operator<<(const Gt::Line_2& l);
	VoronoiDrawer& operator<<(const Gt::Ray_2& r);
	VoronoiDrawer& operator<<(const CGAL::Parabola_segment_2<Gt>& p);
};

#endif //CARTOCROW_MEDIAL_AXIS_DEMO_H
