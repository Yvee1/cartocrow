#ifndef CARTOCROW_TREEMAP_PAINTING_HPP
#define CARTOCROW_TREEMAP_PAINTING_HPP

#include "cartocrow/core/core.h"
#include "cartocrow/renderer/geometry_painting.h"
//#include "cartocrow/treemap/ok_hsv_hsl.h"
#include "cartocrow/treemap/treemap.h"
#include "cartocrow/treemap/treemap_helpers.h"
#include "parse_csv_to_tree.h"

template <class V>
class TreemapPainting : public renderer::GeometryPainting {
  public:
	TreemapPainting(Treemap<V> treemap, Treemap<V> initial_treemap):
	      m_treemap(std::move(treemap)), m_initial_treemap(std::move(initial_treemap)) {};
	void paint(renderer::GeometryRenderer& renderer) const override;

	Treemap<V> m_treemap;
	Treemap<V> m_initial_treemap;

  private:
};

template <class V>
void TreemapPainting<V>::paint(renderer::GeometryRenderer& renderer) const {
	auto arr = *m_treemap.m_arrangement;

	//	draw_node_in_hue_range(m_treemap, m_treemap.m_tree, renderer, 0.0, 1.0);
	for (auto [leaf, _] : m_treemap.m_leaf_to_face) {
		std::optional<Polygon<K>> initial_poly = m_initial_treemap.node_region(leaf);
		Polygon<K> current_poly = *(m_treemap.node_region(leaf));
		CGAL::Bbox_2 bb;
		if (!initial_poly.has_value()) {
//			throw std::runtime_error("Don't know how to color node that is not part of treemap of first timestep.");
//			std::cerr << "Don't know how to color a node that is not part of the initial treemap." << std::endl;
			bb = current_poly.bbox();
		} else {
			bb = initial_poly->bbox();
		}
		Point<K> centroid(bb.xmin() + bb.x_span() / 2, bb.ymin() + bb.y_span() / 2);
		auto v = approximate(centroid);
		renderer.setFill(color_at(v.x() / 100, (100 - v.y()) / 100));
		renderer.setMode(renderer::GeometryRenderer::fill);
		renderer.draw(current_poly);
	}

	for (auto eit = arr.edges_begin(); eit != arr.edges_end(); eit++) {
		auto edge = *eit;
		Segment<K> seg(edge.source()->point(), edge.target()->point());
		//		renderer.setStroke(Color{0, 0, 0}, 1.0);
		renderer.setStroke(Color{255, 255, 255}, 1.0);
		renderer.setMode(renderer::GeometryRenderer::stroke);
		renderer.draw(seg);
	}
}

//template <class V>
//void draw_node_in_hue_range(const Treemap<V>& treemap, const NP<V>& node,
//                            renderer::GeometryRenderer& renderer, float lower, float upper, float s = 0.55,
//                            float v = 0.93, float padding = 0.3, float shift = 0.2) {
//	if (node->is_leaf()) {
//		auto region = treemap.node_region(node);
//		if (!region.has_value()) return;
//
//		renderer.setMode(renderer::GeometryRenderer::fill);
//		auto h = lower + (upper - lower) / 2;
//		auto rgb = ok_color::okhsv_to_srgb({(h + shift) - static_cast<int>(h + shift), s, v});
//		renderer.setFill(Color{static_cast<int>(rgb.r * 255), static_cast<int>(rgb.g * 255),
//		                       static_cast<int>(rgb.b * 255)});
//		renderer.draw(*region);
//		return;
//	}
//
//	int n = node->children.size();
//	for (int i = 0; i < n; i++) {
//		float l = lower + (static_cast<float>(i) + padding) /
//		                      (static_cast<float>(n) + 2 * padding) * (upper - lower);
//		float u = lower + (static_cast<float>(i) + 1 + padding) /
//		                      (static_cast<float>(n) + 2 * padding) * (upper - lower);
//		draw_node_in_hue_range(treemap, node->children[i], renderer, l, u, s, v);
//	}
//}


template <class V>
class NodePainting : public renderer::GeometryPainting {
  public:
	NodePainting(const Treemap<V>& treemap, const std::optional<NP<V>>& node);
	void paint(renderer::GeometryRenderer& renderer) const override;

  private:
	const Treemap<V>& m_treemap;
	const std::optional<NP<V>>& m_node;
};

template <class V>
NodePainting<V>::NodePainting(const Treemap<V>& treemap, const std::optional<NP<V>>& node) : m_node(node), m_treemap(treemap) {}

template <class V>
void NodePainting<V>::paint(renderer::GeometryRenderer& renderer) const {
	if (!m_node.has_value()) return;
	auto n = *m_node;
	renderer.setMode(renderer::GeometryRenderer::stroke);
	renderer.setStroke({0, 0, 0}, 3.0);
	renderer.draw(*(m_treemap.node_region(n)));
}

#endif //CARTOCROW_TREEMAP_PAINTING_HPP
