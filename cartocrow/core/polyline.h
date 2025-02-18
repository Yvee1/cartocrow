/*
The CartoCrow library implements algorithmic geo-visualization methods,
developed at TU Eindhoven.
Copyright (C) 2021  Netherlands eScience Center and TU Eindhoven

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef CARTOCROW_POLYLINE_H
#define CARTOCROW_POLYLINE_H

#include <CGAL/Point_2.h>
#include <CGAL/Segment_2.h>

namespace cartocrow {
template <class Segment_>
class Polyline_Segment_ptr
{
  public:
	typedef Segment_ Segment;
	Polyline_Segment_ptr(Segment const &seg) :m_seg(seg){}
	Segment* operator->() {return &m_seg;}
  private:
	Segment m_seg;
};

template <class K, class RandomAccessIterator> class SegmentIterator:
    public std::iterator<
        std::random_access_iterator_tag, 			    // iterator_category
        CGAL::Segment_2<K>, 				 	        // value_type
        typename RandomAccessIterator::difference_type, // difference_type
        CGAL::Segment_2<K>*,				 	        // pointer
        CGAL::Segment_2<K>& 				 	        // reference
        > {
  private:
	RandomAccessIterator m_first_vertex;
    using difference_type = typename RandomAccessIterator::difference_type;

  public:
	typedef SegmentIterator<K, RandomAccessIterator> Self;

	SegmentIterator(RandomAccessIterator first_vertex): m_first_vertex(first_vertex) {};

	CGAL::Segment_2<K> operator*() const {
		auto second_vertex = m_first_vertex;
		++second_vertex;
		return { *m_first_vertex, *second_vertex };
	}

	Polyline_Segment_ptr<CGAL::Segment_2<K>> operator->() const
	{return Polyline_Segment_ptr<CGAL::Segment_2<K>>(operator*());}

	Self& operator++() {
		++m_first_vertex;
		return *this;
	}

	Self operator++(int) {
		Self tmp = *this;
		++*this;
		return tmp;
	}

    Self& operator--() {
        --m_first_vertex;
        return *this;
    }

    Self operator--(int) {
        Self tmp = *this;
        --*this;
        return tmp;
    }

    Self& operator+=(difference_type rhs) { m_first_vertex += rhs; return *this; }
    Self& operator-=(difference_type rhs) { m_first_vertex -= rhs; return *this; }
    Self operator+(difference_type rhs) { return SegmentIterator(m_first_vertex + rhs); }
    Self operator-(difference_type rhs) { return SegmentIterator(m_first_vertex - rhs); }
    friend Self operator+(difference_type lhs, const Self& rhs) { return SegmentIterator(lhs + rhs); }
    friend Self operator-(difference_type lhs, const Self& rhs) { return SegmentIterator(lhs - rhs); }
    difference_type operator-(const Self& rhs) { return m_first_vertex - rhs.m_first_vertex; }

    CGAL::Segment_2<K> operator[](difference_type rhs) { return *(this + rhs); }

	bool operator==(const Self& other) const {
		return m_first_vertex == other.m_first_vertex;
	}

	bool operator!=(const Self& other) const {
		return m_first_vertex != other.m_first_vertex;
	}
};

template <class K> class Polyline {
  public:
	typedef typename std::vector<CGAL::Point_2<K>>::const_iterator Vertex_iterator;
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
	[[nodiscard]] int num_vertices() const { return m_points.size(); }
	[[nodiscard]] int num_edges() const { return m_points.size() - 1; }
	[[nodiscard]] int size() const { return num_vertices(); }
	[[nodiscard]] CGAL::Point_2<K> vertex(int i) const { return m_points[i]; }
	[[nodiscard]] CGAL::Segment_2<K> edge(int i) const { return *(std::next(edges_begin(), i)); }
    [[nodiscard]] CGAL::Point_2<K> source() const { return m_points.front(); }
    [[nodiscard]] CGAL::Point_2<K> target() const { return m_points.back(); }
    [[nodiscard]] CGAL::Point_2<K> start() const { return source(); }
    [[nodiscard]] CGAL::Point_2<K> end() const { return target(); }
  private:
	std::vector<CGAL::Point_2<K>> m_points;
};
}

#endif //CARTOCROW_POLYLINE_H
