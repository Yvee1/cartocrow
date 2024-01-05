//
// Created by steven on 1/4/24.
//

#include "medial_axis.h"
#include <CGAL/Segment_Delaunay_graph_2.h>
#include <CGAL/Segment_Delaunay_graph_adaptation_policies_2.h>
#include <CGAL/Segment_Delaunay_graph_adaptation_traits_2.h>
#include <CGAL/Segment_Delaunay_graph_traits_2.h>

using namespace CGAL;

//bool is_endpoint_of_segment(Site site2, Site site21);
//bool same_points(Site& p, Site& q);

//template <class Stream,
//          class K,
//          class Gt,
//          class SDG
//          >
//Stream& draw_skeleton(const SDG& dg, Stream& str) {
////	typedef Segment_Delaunay_graph_traits_2<K> Gt;
////	typedef Segment_Delaunay_graph_2<Gt> SDG;
////	typedef Segment_Delaunay_graph_adaptation_traits_2<SDG>	AT;
////	typedef typename AT::Site_2 Site;
////	typedef Segment_Delaunay_graph_degeneracy_removal_policy_2<SDG>	AP;
////	typedef Segment_Delaunay_graph_storage_traits_2<Gt> ST;
////	typedef Triangulation_data_structure_2<Segment_Delaunay_graph_vertex_base_2<ST>, Segment_Delaunay_graph_face_base_2<Gt>> D_S;
////	typedef Segment_Delaunay_graph_traits_wrapper_2<Gt>	Modified_traits;
////	typedef Triangulation_2<Modified_traits,D_S> DG;
////	typedef typename DG::Finite_edges_iterator Finite_edges_iterator;
//
//	auto eit = dg.finite_edges_begin();
//	for (; eit != dg.finite_edges_end(); ++eit) {
//		auto p = eit->first->vertex(  SDG::cw(eit->second) )->site();
//		auto q = eit->first->vertex( SDG::ccw(eit->second) )->site();
//
//		bool is_endpoint_of_seg =
//		    ( p.is_segment() && q.is_point() &&
//		     is_endpoint_of_segment(q, p) ) ||
//		    ( p.is_point() && q.is_segment() &&
//		     is_endpoint_of_segment(p, q) );
//
//		if ( !is_endpoint_of_seg ) {
//			dg.draw_dual_edge(*eit, str);
//		}
//	}
//	return str;
//}
//
//template<class K, class Gt, class SDG, class AT>
//bool is_endpoint_of_segment(typename AT::Site_2 p, typename AT::Site_2 s) {
//	CGAL_precondition( p.is_point() && s.is_segment() );
//	return ( same_points(p, s.source_site()) ||
//	        same_points(p, s.target_site()) );
//}
//
//template<class K, class Gt, class SDG, class AT, class ST, class D_S, class Modified_traits, class DG>
//bool same_points(typename AT::Site_2 & p, typename AT::Site_2 & q) {
//	return DG::geom_traits().equal_2_object()(p.site(), q.site());
//}
