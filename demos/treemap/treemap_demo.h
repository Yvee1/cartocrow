#ifndef CARTOCROW_TREEMAP_DEMO_H
#define CARTOCROW_TREEMAP_DEMO_H

#include "cartocrow/treemap/orthoconvex.h"
#include "parse_csv_to_tree.h"
#include "treemap_painting.h"
#include "treemap_painting.hpp"
#include "cartocrow/core/core.h"
#include "cartocrow/core/ipe_reader.h"
#include "cartocrow/renderer/geometry_painting.h"
#include "cartocrow/renderer/geometry_widget.h"
#include <QMainWindow>

using namespace cartocrow;
using namespace cartocrow::renderer;
using namespace cartocrow::treemap;

class TreemapDemo : public QMainWindow {
	Q_OBJECT

  public:
	TreemapDemo();
	void resizeEvent(QResizeEvent *event) override;

  private:
	void clear_info_box();
	void create_info_box(Point<Inexact> pt, const NPN& node);
	void updated_treemap();
	void load_file(const std::filesystem::path& filePath);

	GeometryWidget* m_renderer;
	QFrame* m_info_box;
	std::optional<Point<Inexact>> m_info_box_position;
	std::optional<NPN> m_selected_node;
	std::optional<Treemap<Named>> m_treemap;
	std::shared_ptr<TreemapPainting<Named>> m_tmp;
	Arrangement<K>::Face_handle face_at_point(const Point<K>& point);
	int m_timestep = 0;
};

#endif //CARTOCROW_TREEMAP_DEMO_H
