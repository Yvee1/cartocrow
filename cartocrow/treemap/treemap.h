#ifndef CARTOCROW_TREEMAP_H
#define CARTOCROW_TREEMAP_H

#include "treemap_helpers.h"
#include "arrangement_helpers.h"
#include "cartocrow/treemap/arrangement_helpers.hpp"

namespace cartocrow::treemap {
template <class V>
class Treemap {
  private:
	void nodeFacesHelper(const NP<V>& node, std::unordered_set<FaceH>& faces) const {
		if (node->is_leaf()) {
			if (m_leafToFace.contains(node)) { // if node has weight zero it is not part of the treemap
				faces.insert(m_leafToFace.at(node));
			}
		} else {
			for (const auto& child : node->children) {
				nodeFacesHelper(child, faces);
			}
		}
	}

  public:
	Treemap(NP<V> tree, std::shared_ptr<TMArrangement> arr, std::unordered_map<NP<V>, FaceH> leaf_to_face, Rectangle<K> rect):
	      m_tree(std::move(tree)), m_arrangement(std::move(arr)),
	      m_leafToFace(std::move(leaf_to_face)), m_rectangle(rect), m_arrAcc(*m_arrangement) {
		for (const auto& [leaf, face] : m_leafToFace) {
			m_faceToLeaf[face] = leaf;
		}
		maximalSegments(*m_arrangement, std::back_inserter(m_maximalSegments));
	};
	std::unordered_set<FaceH> nodeFaces(const NP<V>& node) const {
		std::unordered_set<FaceH> faces;
		nodeFacesHelper(node, faces);
		return faces;
	};
	std::optional<Polygon<K>> nodeRegion(const NP<V>& node) const {
		if (node->is_leaf()) {
			if (m_leafToFace.contains(node)) {
				return face_to_polygon(m_leafToFace.at(node));
			} else {
				return std::nullopt; // This should happen only if the node has weight zero
			}
		} else {
			auto faces = nodeFaces(node);
			return faces_to_polygon(faces);
		}
	};
	
	using MaximalSegmentId = int;

	/// Currently only works only when maximal segments are either vertical or horizontal.
	void moveMaximalSegment(MaximalSegmentId msId, Number<K> delta) {
		MaximalSegment& mSeg = m_maximalSegments[msId];
		std::vector<std::tuple<Segment<K>, bool, bool>> newCurves;
		for (auto& e : mSeg.halfedges) {
			auto& curve = e->curve();
			bool rev = e->source()->point() == curve.target();
			bool vertical = abs(curve.source().x() - curve.target().x()) < M_EPSILON;
			Segment<K> newCurve;
			if (vertical) {
				newCurve = Segment<K>({curve.source().x() + delta, curve.source().y()}, {curve.source().x() + delta, curve.target().y()});
			} else {
				newCurve = Segment<K>({curve.source().x(), curve.target().y() + delta}, {curve.target().x(), curve.target().y() + delta});
			}
			newCurves.push_back({newCurve, rev, vertical});
		}
		
		auto [cf, rf, vertf] = newCurves.front();
		auto source = rf ? cf.target() : cf.source();
		auto [cb, rb, vertb] = newCurves.back();
		auto target = rb ? cb.source() : cb.target();
		auto vf = mSeg.halfedges.front()->source();
		auto vb = mSeg.halfedges.back()->target();
		auto vsf = perpVertices(mSeg.halfedges.front()->twin());
		auto vsb = perpVertices(mSeg.halfedges.back());
		std::vector<Point<K>> psf;
		for (const auto& v : vsf) {
			psf.push_back(v->point());
		}
		std::sort(psf.begin(), psf.end(), [vertf](Point<K> p1, Point<K> p2) { return vertf ? p1.x() <= p2.x() : p1.y() <= p2.y(); });
		std::vector<Point<K>> psb;
		for (const auto& v : vsb) {
			psb.push_back(v->point());
		}
		std::sort(psb.begin(), psb.end(), [vertb](Point<K> p1, Point<K> p2) { return vertb ? p1.x() <= p2.x() : p1.y() <= p2.y(); });

		bool problem = false;
		if (psf.size() > 1 && std::upper_bound(psf.begin(), psf.end(), source, [vertf](Point<K> p1, Point<K> p2) { return vertf ? p1.x() <= p2.x() : p1.y() <= p2.y(); }) != ++psf.begin()) {
			std::cout << "! Non-order-equivalent layout !" << std::endl;
			problem = true;
		}
		if (psb.size() > 1 && std::upper_bound(psb.begin(), psb.end(), target, [vertb](Point<K> p1, Point<K> p2) { return vertb ? p1.x() <= p2.x() : p1.y() <= p2.y(); }) != ++psb.begin()) {
			std::cout << "! Non-order-equivalent layout !" << std::endl;
			problem = true;
		}

		for (int i = 0; !problem && i < newCurves.size(); ++i) {
			auto& [newCurve, rev, vert] = newCurves[i];
			auto& e = mSeg.halfedges[i];
			auto modifyVertex = [this, vert](TMArrangement::Vertex_handle v, auto p) {
				auto start = v->incident_halfedges();
				auto eit = start;
				do {
					if (vert && isVertical(eit) || !vert && isHorizontal(eit)) continue;
					auto curve = eit->curve();
					auto dir = (curve.target() - curve.source()).direction();
					auto replaceSource = curve.source() == v->point();
					auto replaceTarget = curve.target() == v->point();
					auto newCurve = Segment<K>(replaceSource ? p : curve.source(), replaceTarget ? p : curve.target());
					auto newDir = (newCurve.target() - newCurve.source()).direction();
					if (!approx_same_direction(dir, newDir)) {
						auto prev = prevOnMaximalSegment(eit);
						auto next = nextOnMaximalSegment(eit);
					}
					m_arrAcc.modify_edge_ex(eit, newCurve);
				} while(++eit != start);
				m_arrAcc.modify_vertex_ex(v, p);
			};
			modifyVertex(e->source(), rev ? newCurve.target() : newCurve.source());
			modifyVertex(e->target(), rev ? newCurve.source() : newCurve.target());
			m_arrAcc.modify_edge_ex(e, newCurve);
		}

		m_maximalSegments.clear();
		maximalSegments(*m_arrangement, std::back_inserter(m_maximalSegments));
	}
	
	std::shared_ptr<TMArrangement> m_arrangement;
	CGAL::Arr_accessor<TMArrangement> m_arrAcc;
	std::unordered_map<NP<V>, FaceH> m_leafToFace;
	std::unordered_map<FaceH, NP<V>> m_faceToLeaf;
	std::vector<MaximalSegment> m_maximalSegments;
	NP<V> m_tree;
	Rectangle<K> m_rectangle;
};
}

#endif //CARTOCROW_TREEMAP_H
