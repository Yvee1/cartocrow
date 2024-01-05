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

typedef cartocrow::Inexact K;
//typedef CGAL::Delaunay_triangulation_2<K>                                    DT;
//typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DT>                 AT;
//typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DT> AP;
//typedef CGAL::Voronoi_diagram_2<DT,AT,AP>                                    VD;
//typedef AT::Site_2                          							     Site;
typedef CGAL::Segment_Delaunay_graph_traits_2<K>                       Gt;
typedef CGAL::Segment_Delaunay_graph_2<Gt>                             SDG2;
typedef CGAL::Segment_Delaunay_graph_adaptation_traits_2<SDG2>         AT;
typedef CGAL::Segment_Delaunay_graph_degeneracy_removal_policy_2<SDG2> AP;
typedef CGAL::Voronoi_diagram_2<SDG2, AT, AP>      VD;
typedef AT::Site_2                                 Site;

using namespace cartocrow;
using namespace cartocrow::renderer;

class Isoline {
  public:
	Isoline(std::vector<Segment<Inexact>> segments);

	std::vector<Segment<Inexact>> m_segments;
};

class MedialAxisDemo : public QMainWindow {
	Q_OBJECT

  public:
	MedialAxisDemo();
	void recalculate();
	void addIsolineToVoronoi(const Isoline& isoline);
	std::vector<Isoline> isolinesInPage(ipe::Page* page);

  private:
	std::vector<Isoline> m_isolines;
	VD m_voronoi;
	cartocrow::renderer::GeometryWidget* m_renderer;
};

class MedialAxisPainting : public cartocrow::renderer::GeometryPainting {
  public:
	MedialAxisPainting(std::vector<Isoline>* isolines, VD* voronoi);

  protected:
	void paint(cartocrow::renderer::GeometryRenderer& renderer) const override;

  private:
	std::vector<Isoline>* m_isolines;
	VD* m_voronoi;
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
