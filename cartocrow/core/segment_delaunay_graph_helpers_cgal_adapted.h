// These functions are adapted from CGAL functions that fall under the following license.
// draw_dual_edge was modified so that it does not linearize parabolic segments.
// See https://github.com/CGAL/cgal/issues/7973.
// Copyright (c) 2003,2004,2005,2006  INRIA Sophia-Antipolis (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v5.4/Segment_Delaunay_graph_2/include/CGAL/Segment_Delaunay_graph_2.h $
// $Id: Segment_Delaunay_graph_2.h 98e4718 2021-08-26T11:33:39+02:00 Sébastien Loriot
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     : Menelaos Karavelas <mkaravel@iacm.forth.gr>

#pragma once

#include <CGAL/Segment_Delaunay_graph_2.h>
#include <CGAL/Segment_Delaunay_graph_adaptation_policies_2.h>
#include <CGAL/Segment_Delaunay_graph_adaptation_traits_2.h>
#include <CGAL/Segment_Delaunay_graph_hierarchy_2.h>
#include <CGAL/Segment_Delaunay_graph_traits_2.h>

namespace cartocrow {
template<class SDG>
bool same_points(const SDG& dg, const typename SDG::Site_2& p, const typename SDG::Site_2& q) {
	return dg.geom_traits().equal_2_object()(p, q);
}

template<class SDG>
bool is_endpoint_of_segment(const SDG& dg, typename SDG::Site_2& p, typename SDG::Site_2& s) {
	CGAL_precondition( p.is_point() && s.is_segment() );
	return ( same_points<SDG>(dg, p, s.source_site()) ||
	        same_points<SDG>(dg, p, s.target_site()) );
}

template <class Stream, class SDG>
Stream& draw_dual_edge(const SDG& dg, typename SDG::Edge e, Stream& str)
{
	typename SDG::Geom_traits::Line_2  l;
	typename SDG::Geom_traits::Segment_2 s;
	typename SDG::Geom_traits::Ray_2     r;
	CGAL::Parabola_segment_2<typename SDG::Geom_traits> ps;

	if (dg.is_infinite(e)) return str;
	CGAL::Object o = dg.primal(e);

	if (CGAL::assign(l, o))   str << l;
	if (CGAL::assign(s, o))   str << s;
	if (CGAL::assign(r, o))   str << r;
	if (CGAL::assign(ps, o))  str << ps;

	return str;
}

template <class Stream,
          class SDG>
Stream& draw_skeleton(const SDG& dg, Stream& str) {
	auto eit = dg.finite_edges_begin();
	for (; eit != dg.finite_edges_end(); ++eit) {
		auto p = eit->first->vertex(  SDG::cw(eit->second) )->site();
		auto q = eit->first->vertex( SDG::ccw(eit->second) )->site();

		bool is_endpoint_of_seg =
		    ( p.is_segment() && q.is_point() &&
		     is_endpoint_of_segment<SDG>(dg, q, p) ) ||
		    ( p.is_point() && q.is_segment() &&
		     is_endpoint_of_segment<SDG>(dg, p, q) );

		if ( !is_endpoint_of_seg ) {
			draw_dual_edge<Stream, SDG>(dg, *eit, str);
		}
	}
	return str;
}

template <class Stream,
          class SDG>
Stream& draw_dual(const SDG& dg, Stream& str) {
	auto eit = dg.finite_edges_begin();
	for (; eit != dg.finite_edges_end(); ++eit) {
		draw_dual_edge<Stream, SDG>(dg, *eit, str);
	}
	return str;
}
}