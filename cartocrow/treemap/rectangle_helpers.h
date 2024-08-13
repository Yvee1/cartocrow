#ifndef CARTOCROW_RECTANGLE_HELPERS_H
#define CARTOCROW_RECTANGLE_HELPERS_H

#include "cartocrow/core/core.h"

namespace cartocrow {
template <class K> using Rectangle = CGAL::Iso_rectangle_2<K>;

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

bool is_horizontal(Side side);

Corner mirror_corner(Corner corner, bool vertical);

template <class K>
Number<K> width(const Rectangle<K>& rect) {
	return rect.xmax() - rect.xmin();
}

template <class K>
Number<K> height(const Rectangle<K>& rect) {
	return rect.ymax() - rect.ymin();
}

template <class K>
Point<K> centroid(const Rectangle<K>& rect) {
	return {(rect.xmin() + rect.xmax()) / 2, (rect.ymin() + rect.ymax()) / 2};
}

template <class K>
auto dimension(const Rectangle<K>& rect, int i) {
	if (i == 0) {
		return width(rect);
	} else if (i == 1) {
		return height(rect);
	} else {
		throw std::runtime_error("Dimension i is not 0 or 1");
	}
}

template <class K>
Corner corner(const Rectangle<K>& rect, const Side& side1, const Side& side2) {
	if (side1 > side2) return corner(rect, side2, side1);
	int dist = side2 - side1;
	if (dist == 1) {
		return static_cast<Corner>(side1);
	} else if (dist == 3) {
		return static_cast<Corner>(side2);
	} else {
		throw std::runtime_error("Sides are not adjacent");
	}
}

template <class K>
Side side(const Rectangle<K>& rect, Corner corner1, Corner corner2) {
	if (corner1 > corner2) return side(rect, corner2, corner1);
	int dist = corner2 - corner1;
	if (dist == 1) {
		return static_cast<Side>(corner2);
	} else if (dist == 3) {
		return static_cast<Side>(corner1);
	} else {
		throw std::runtime_error("Corners are not adjacent");
	}
}
}

#endif //CARTOCROW_RECTANGLE_HELPERS_H
