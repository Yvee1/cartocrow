//
// Created by steven on 1/8/24.
//

#ifndef CARTOCROW_POLYLINE_H
#define CARTOCROW_POLYLINE_H

#include <CGAL/Point_2.h>
#include <CGAL/Segment_2.h>

namespace cartocrow {
template <class K, class InputIterator> class SegmentIterator:
    public std::iterator<
        std::input_iterator_tag, 			     // iterator_category
        CGAL::Segment_2<K>, 				 	 // value_type
        typename InputIterator::difference_type, // difference_type
        CGAL::Segment_2<K>*,				 	 // pointer
        CGAL::Segment_2<K>& 				 	 // reference
        > {
  private:
	InputIterator m_first_vertex;

  public:
	typedef SegmentIterator<K, InputIterator> Self;

	SegmentIterator(InputIterator first_vertex): m_first_vertex(first_vertex) {};

	CGAL::Segment_2<K> operator*() const {
		auto second_vertex = m_first_vertex;
		++second_vertex;
		return { *m_first_vertex, *second_vertex };
	}

	Self& operator++() {
		++m_first_vertex;
		return *this;
	}

	Self operator++(int) {
		Self tmp = *this;
		++*this;
		return tmp;
	}

	bool operator==(const Self& other) const {
		return m_first_vertex == other.m_first_vertex;
	}

	bool operator!=(const Self& other) const {
		return m_first_vertex != other.m_first_vertex;
	}
};

template <class K> class Polyline {
  public:
	typedef std::vector<CGAL::Point_2<K>>::const_iterator Vertex_iterator;
	typedef SegmentIterator<K, typename std::vector<typename CGAL::Point_2<K>>::const_iterator> Edge_iterator;
	Polyline() = default;

	template <class InputIterator> Polyline(InputIterator begin, InputIterator end) {
		if (begin == end) {
			throw std::runtime_error("Polyline cannot be empty.");
		}
		std::copy(begin, end, std::back_inserter(m_points));
	}

	explicit Polyline(std::vector<CGAL::Point_2<K>> points): m_points(points) {};

	void push_back(const CGAL::Point_2<K>& p) {
		m_points.push_back(p);
	}

	void insert(Vertex_iterator i, const CGAL::Point_2<K>& p) {
		m_points.insert(i, p);
	}

	[[nodiscard]] Vertex_iterator vertices_begin() const { return m_points.begin(); }
	[[nodiscard]] Vertex_iterator vertices_end() const { return m_points.end(); }
	[[nodiscard]] Edge_iterator edges_begin() const { return { m_points.begin() }; }
	[[nodiscard]] Edge_iterator edges_end() const { return { --m_points.end() }; }
	[[nodiscard]] int size() const { return m_points.size(); }
	[[nodiscard]] CGAL::Point_2<K> vertex(int i) const { return m_points[i]; }
  private:
	std::vector<CGAL::Point_2<K>> m_points;
};
}

#endif //CARTOCROW_POLYLINE_H
