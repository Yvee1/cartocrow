#ifndef CARTOCROW_TREEMAP_DEMO_H
#define CARTOCROW_TREEMAP_DEMO_H

#include "tree.h"
#include "cartocrow/core/core.h"
#include "cartocrow/core/ipe_reader.h"
#include "cartocrow/renderer/geometry_painting.h"
#include "cartocrow/renderer/geometry_widget.h"
#include <QMainWindow>
#include <CGAL/Arr_non_caching_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>

using namespace cartocrow;
using namespace cartocrow::renderer;

template <class K> using Rectangle = CGAL::Iso_rectangle_2<K>;

typedef Inexact K;

class TreemapDemo : public QMainWindow {
	Q_OBJECT

  public:
	TreemapDemo();

  private:
	GeometryWidget* m_renderer;
};

struct Value {
	int label;
	double weight;
};

template <class V> using NP = std::shared_ptr<Node<V>>;
typedef NP<Value> NPV;

struct MarkedNPV {
	NPV tree;
	NPV marked;
};

enum Side {
	Left,
	Bottom,
	Right,
	Top
};

enum Corner {
	BL,
	BR,
	TR,
	TL,
};

Corner opposite(Corner corner);

std::pair<NPV, int> label_tree(const NP<double>& tree, int start = 0);

Arrangement<K> build_treemap(const NPV& tree);
void recurse_treemap(const NPV& tree, const NPV& marked, Arrangement<K>& arr, Arrangement<K>::Face_handle& face, Corner corner);

class TreemapPainting : public GeometryPainting {
  public:
	TreemapPainting(Arrangement<K> arr);
	void paint(GeometryRenderer& renderer) const override;

  private:
	Arrangement<K> m_arr;
};

#endif //CARTOCROW_TREEMAP_DEMO_H
