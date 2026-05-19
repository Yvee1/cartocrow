#include "../catch.hpp"

#include <cartocrow/core/polygon_helpers.h>

using namespace cartocrow;

TEST_CASE("Square centroid (CCW)") {
	Polygon<Exact> p;
	p.push_back(Point<Exact>(0, 0));
	p.push_back(Point<Exact>(10, 0));
	p.push_back(Point<Exact>(10, 10));
	p.push_back(Point<Exact>(0, 10));

	CHECK(p.is_counterclockwise_oriented());

	Point<Exact> c = centroid(p);

	CHECK(c == Point<Exact>(5, 5));
}

TEST_CASE("Square centroid (CW)") {
	Polygon<Exact> p;
	p.push_back(Point<Exact>(0, 0));
	p.push_back(Point<Exact>(0, 10));
	p.push_back(Point<Exact>(10, 10));
	p.push_back(Point<Exact>(10, 0));

	CHECK(p.is_clockwise_oriented());

	Point<Exact> c = centroid(p);

	CHECK(c == Point<Exact>(5, 5));
}

TEST_CASE("Nested centroid") {

	// 10x10 square
	Polygon<Exact> p1;
	p1.push_back(Point<Exact>(0, 0));
	p1.push_back(Point<Exact>(0, 10));
	p1.push_back(Point<Exact>(10, 10));
	p1.push_back(Point<Exact>(10, 0));
		
	// 5x5 square
	Polygon<Exact> p2;
	p2.push_back(Point<Exact>(0, 0));
	p2.push_back(Point<Exact>(0, 5));
	p2.push_back(Point<Exact>(5, 5));
	p2.push_back(Point<Exact>(5, 0));

	// L shape created by 10x10 minus 5x5
	PolygonWithHoles<Exact> p12(p1);
	p12.add_hole(p2);

	// same L shape as above, but now as single polygon
	Polygon<Exact> p3;
	p3.push_back(Point<Exact>(5, 0));
	p3.push_back(Point<Exact>(10, 0));
	p3.push_back(Point<Exact>(10, 10));
	p3.push_back(Point<Exact>(0, 10));
	p3.push_back(Point<Exact>(0, 5));
	p3.push_back(Point<Exact>(5, 5));

	Point<Exact> c12 = centroid(p12);
	Point<Exact> c3 = centroid(p3);

	CHECK(c12 == c3);
}