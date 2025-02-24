#ifndef CARTOCROW_KINETIC_KELP_DEMO_H
#define CARTOCROW_KINETIC_KELP_DEMO_H

#include <QMainWindow>
#include <QToolBar>
#include <QSpinBox>

#include "cartocrow/core/core.h"
#include "cartocrow/kinetic_kelp/input.h"
#include "cartocrow/kinetic_kelp/settings.h"
#include "cartocrow/kinetic_kelp/draw_settings.h"
#include "cartocrow/kinetic_kelp/state.h"
#include "cartocrow/kinetic_kelp/state_geometry.h"
#include "cartocrow/kinetic_kelp/pseudotriangulation.h"
#include "cartocrow/kinetic_kelp/kinetic_kelp_painting.h"

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/boost/graph/graph_traits_Delaunay_triangulation_2.h>
#include <CGAL/boost/graph/kruskal_min_spanning_tree.h>

#include "cartocrow/renderer/geometry_widget.h"

#include "time_control_widget.h"

#include <map>
#include <filesystem>

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
    Settings m_settings;
    DrawSettings m_drawSettings;
    std::vector<DT> m_dts;
    std::vector<std::list<TGEdge>> m_msts;
    Box m_bbox;
    double m_pointRadius = 1;
    std::optional<qint64> m_pausedTime;
    TimeControlToolBar* m_timeControl;
    QSpinBox* m_interpolationTimeSpinBox;

    std::filesystem::path m_filePath;

    bool m_recompute = false;

    std::shared_ptr<InputInstance> m_inputInstance;
    std::shared_ptr<State> m_state;
    std::shared_ptr<StateGeometry> m_stateGeometry;
    std::shared_ptr<Pseudotriangulation> m_pt;
    std::shared_ptr<PseudotriangulationGeometry> m_ptg;

    std::shared_ptr<std::vector<Kelp>> m_kelps;
    std::shared_ptr<KineticKelpPainting> m_kkPainting;

    void fitToScreen();
    void resizeEvent(QResizeEvent *event) override;
    void initialize();
    bool update(double time);
};

#endif //CARTOCROW_KINETIC_KELP_DEMO_H
