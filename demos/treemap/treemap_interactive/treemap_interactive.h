#ifndef CARTOCROW_TREEMAP_INTERACTIVE_H
#define CARTOCROW_TREEMAP_INTERACTIVE_H

#include "cartocrow/core/core.h"
#include "cartocrow/reader/ipe_reader.h"
#include "cartocrow/renderer/geometry_painting.h"
#include "cartocrow/renderer/geometry_widget.h"
#include "cartocrow/treemap/parse_csv_to_tree.h"
#include "cartocrow/treemap/treemap_painting.h"
#include "cartocrow/treemap/treemap_painting.hpp"
#include <QMainWindow>

using namespace cartocrow;
using namespace cartocrow::renderer;
using namespace cartocrow::treemap;

typedef std::function<std::shared_ptr<Treemap<Named>>(NP<Named>& tree, const Rectangle<K>& rect, NodeWeight<Named> w)> TreemapBuilder;

class TreemapInteractive : public QMainWindow {
	Q_OBJECT

  public:
	TreemapInteractive();
	void resizeEvent(QResizeEvent *event) override;

  private:
	void clear_info_box();
	void create_info_box(Point<Inexact> pt, const NPN& node);
	void updated_treemap();
	void load_file(const std::filesystem::path& filePath);

	Rectangle<K> m_rect = Rectangle<K>(Point<K>(0, 0), Point<K>(100, 100));

	GeometryWidget* m_renderer;
	QFrame* m_info_box = nullptr;
	std::optional<Point<Inexact>> m_info_box_position;
	std::optional<NPN> m_selected_node;
	std::shared_ptr<Treemap<Named>> m_treemap;
	std::shared_ptr<TreemapPainting<Named>> m_tmp;
	Arrangement<K>::Face_handle face_at_point(const Point<K>& point);
	int m_timestep = 0;
	TreemapBuilder m_treemap_builder;

};

class TreemapEditable : public renderer::GeometryWidget::Editable {
  public:
	TreemapEditable(GeometryWidget* widget, std::shared_ptr<Treemap<Named>> treemap);

	bool drawHoverHint(Point<Inexact> location, Number<Inexact> radius) const override;
	bool startDrag(Point<Inexact> location, Number<Inexact> radius) override;
	void handleDrag(Point<Inexact> to) override;
	void endDrag() override;
	std::optional<MaximalSegment> closestMaximalSegment(Point<Inexact> location, Number<Inexact> radius) const;

  private:
	std::shared_ptr<Treemap<Named>> m_treemap;
	std::vector<MaximalSegment> m_maximalSegments;
	std::optional<MaximalSegment> m_dragging;
	CGAL::Arr_accessor<TMArrangement> m_arrAcc;
};

#endif //CARTOCROW_TREEMAP_INTERACTIVE_H
