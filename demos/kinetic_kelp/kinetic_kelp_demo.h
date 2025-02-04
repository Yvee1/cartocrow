#ifndef CARTOCROW_KINETIC_KELP_DEMO_H
#define CARTOCROW_KINETIC_KELP_DEMO_H

#include <QMainWindow>
#include <QToolBar>

#include "cartocrow/core/core.h"
#include "cartocrow/kinetic_kelp/input.h"

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/boost/graph/graph_traits_Delaunay_triangulation_2.h>
#include <CGAL/boost/graph/kruskal_min_spanning_tree.h>

#include "cartocrow/renderer/geometry_widget.h"

#include "time_control_widget.h"

#include <map>

using namespace cartocrow;
using namespace cartocrow::kinetic_kelp;
using namespace cartocrow::renderer;

typedef CGAL::Delaunay_triangulation_2<Inexact>    DT;
typedef boost::graph_traits<DT>::vertex_descriptor TGVertex;
typedef boost::graph_traits<DT>::edge_descriptor   TGEdge;

typedef std::map<TGVertex,int> VertexIndexMap;
typedef boost::associative_property_map<VertexIndexMap> VertexIdPropertyMap;

class KineticKelpDemo: public QMainWindow {
Q_OBJECT

public:
    KineticKelpDemo();

private:
    GeometryWidget* m_renderer;
    Input m_input;
    std::vector<DT> m_dts;
    std::vector<std::list<TGEdge>> m_msts;
    Box m_bbox;
    double m_pointRadius = 1;
    std::optional<qint64> m_pausedTime;
    TimeControlToolBar* m_timeControl;

    void computeMSTs(double time);
    void fitToScreen();
    void resizeEvent(QResizeEvent *event) override;
};

#endif //CARTOCROW_KINETIC_KELP_DEMO_H
