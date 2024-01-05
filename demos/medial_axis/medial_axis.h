//
// Created by steven on 1/4/24.
//

#ifndef CARTOCROW_MEDIAL_AXIS_H
#define CARTOCROW_MEDIAL_AXIS_H

#include <CGAL/Segment_Delaunay_graph_2.h>
#include <CGAL/Segment_Delaunay_graph_adaptation_policies_2.h>
#include <CGAL/Segment_Delaunay_graph_adaptation_traits_2.h>
#include <CGAL/Segment_Delaunay_graph_traits_2.h>

template<class K,
          class Gt  = CGAL::Segment_Delaunay_graph_traits_2<K>,
          class SDG = CGAL::Segment_Delaunay_graph_2<Gt>,
          class AT  = CGAL::Segment_Delaunay_graph_adaptation_traits_2<SDG>>
bool same_points(const SDG& dg, const typename AT::Site_2& p, const typename AT::Site_2& q) {
	return dg.geom_traits().equal_2_object()(p, q);
}

template<class K,
          class Gt  = CGAL::Segment_Delaunay_graph_traits_2<K>,
          class SDG = CGAL::Segment_Delaunay_graph_2<Gt>,
          class AT  = CGAL::Segment_Delaunay_graph_adaptation_traits_2<SDG>>
bool is_endpoint_of_segment(const SDG& dg, typename AT::Site_2& p, typename AT::Site_2& s) {
	CGAL_precondition( p.is_point() && s.is_segment() );
	return ( same_points<K>(dg, p, s.source_site()) ||
	        same_points<K>(dg, p, s.target_site()) );
}

template <class Stream,
          class K,
          class Gt  = CGAL::Segment_Delaunay_graph_traits_2<K>,
          class SDG = CGAL::Segment_Delaunay_graph_2<Gt>,
          class ST = CGAL::Segment_Delaunay_graph_storage_traits_2<Gt>,
          class D_S = CGAL::Triangulation_data_structure_2<CGAL::Segment_Delaunay_graph_vertex_base_2<ST>, CGAL::Segment_Delaunay_graph_face_base_2<Gt>>>
Stream& draw_dual_edge(const SDG& dg, typename D_S::Edge e, Stream& str)
{
	CGAL_precondition( !is_infinite(e) );

	typename Gt::Line_2    l;
	typename Gt::Segment_2 s;
	typename Gt::Ray_2     r;
	CGAL::Parabola_segment_2<Gt> ps;

	CGAL::Object o = dg.primal(e);

	if (CGAL::assign(l, o))   str << l;
	if (CGAL::assign(s, o))   str << s;
	if (CGAL::assign(r, o))   str << r;
	if (CGAL::assign(ps, o))  str << ps;

	return str;
}

template <class Stream,
          class K,
          class Gt  = CGAL::Segment_Delaunay_graph_traits_2<K>,
          class SDG = CGAL::Segment_Delaunay_graph_2<Gt>
          >
Stream& draw_skeleton(const SDG& dg, Stream& str) {
	//	typedef Segment_Delaunay_graph_traits_2<K> Gt;
	//	typedef Segment_Delaunay_graph_2<Gt> SDG;
	//	typedef Segment_Delaunay_graph_adaptation_traits_2<SDG>	AT;
	//	typedef typename AT::Site_2 Site;
	//	typedef Segment_Delaunay_graph_degeneracy_removal_policy_2<SDG>	AP;
	//	typedef Segment_Delaunay_graph_storage_traits_2<Gt> ST;
	//	typedef Triangulation_data_structure_2<Segment_Delaunay_graph_vertex_base_2<ST>, Segment_Delaunay_graph_face_base_2<Gt>> D_S;
	//	typedef Segment_Delaunay_graph_traits_wrapper_2<Gt>	Modified_traits;
	//	typedef Triangulation_2<Modified_traits,D_S> DG;
	//	typedef typename DG::Finite_edges_iterator Finite_edges_iterator;

	auto eit = dg.finite_edges_begin();
	for (; eit != dg.finite_edges_end(); ++eit) {
		auto p = eit->first->vertex(  SDG::cw(eit->second) )->site();
		auto q = eit->first->vertex( SDG::ccw(eit->second) )->site();

		bool is_endpoint_of_seg =
		    ( p.is_segment() && q.is_point() &&
		     is_endpoint_of_segment<K>(dg, q, p) ) ||
		    ( p.is_point() && q.is_segment() &&
		     is_endpoint_of_segment<K>(dg, p, q) );

		if ( !is_endpoint_of_seg ) {
			draw_dual_edge<Stream, K>(dg, *eit, str);
		}
	}
	return str;
}

#endif //CARTOCROW_MEDIAL_AXIS_H
