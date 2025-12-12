#include "../catch.hpp"

#include "cartocrow/necklace_map/circular_range.h"

using namespace cartocrow::necklace_map;

TEST_CASE("Creating and copying circular ranges") {
	CircularRange r1(0.5 * std::numbers::pi, 1.0 * std::numbers::pi);
	CHECK(r1.from() == 0.5 * std::numbers::pi);
	CHECK(r1.to() == 1.0 * std::numbers::pi);

	CircularRange r2(0.5 * std::numbers::pi, 2.0 * std::numbers::pi);
	CHECK(r2.from() == 0.5 * std::numbers::pi);
	CHECK(r2.to() == 2.0 * std::numbers::pi);

	CircularRange r3(0.5 * std::numbers::pi, 0.0 * std::numbers::pi);
	CHECK(r3.from() == 0.5 * std::numbers::pi);
	CHECK(r3.to() == Approx(2.0 * std::numbers::pi));

	CircularRange r4(0.5 * std::numbers::pi, 0.25 * std::numbers::pi);
	CHECK(r4.from() == 0.5 * std::numbers::pi);
	CHECK(r4.to() == Approx(2.25 * std::numbers::pi));

	CircularRange r5(0.5 * std::numbers::pi, 2.5 * std::numbers::pi);
	CHECK(r5.from() == 0);
	CHECK(r5.to() == 2 * std::numbers::pi);

	CircularRange r6(0.5 * std::numbers::pi, 0.5 * std::numbers::pi);
	CHECK(r6.from() == 0.5 * std::numbers::pi);
	CHECK(r6.to() == 0.5 * std::numbers::pi);

	CircularRange r7(5.0 * std::numbers::pi, 6.5 * std::numbers::pi);
	CHECK(r7.from() == Approx(1.0 * std::numbers::pi));
	CHECK(r7.to() == Approx(2.5 * std::numbers::pi));

	CircularRange r8(6.5 * std::numbers::pi, 5.0 * std::numbers::pi);
	CHECK(r8.from() == Approx(0.5 * std::numbers::pi));
	CHECK(r8.to() == Approx(1.0 * std::numbers::pi));

	CircularRange r9(1.5 * std::numbers::pi, 1.25 * std::numbers::pi);
	CHECK(r9.from() == 1.5 * std::numbers::pi);
	CHECK(r9.to() == Approx(3.25 * std::numbers::pi));
}

TEST_CASE("Checking if circular ranges are valid and degenerate") {
	CircularRange r1(0.5 * std::numbers::pi, 1.5 * std::numbers::pi);
	CHECK(r1.isValid());
	CHECK(!r1.isDegenerate());

	CircularRange r2(std::numbers::pi, std::numbers::pi);
	CHECK(r2.isValid());
	CHECK(r2.isDegenerate());

	CircularRange r3(1.5 * std::numbers::pi, 0.5 * std::numbers::pi);
	CHECK(r3.isValid());
	CHECK(!r3.isDegenerate());
}

TEST_CASE("Checking if a circular range contains a given value") {
	CircularRange r1(0.5 * std::numbers::pi, 1.5 * std::numbers::pi);

	CHECK(!r1.contains(0.0 * std::numbers::pi));
	CHECK(r1.contains(0.5 * std::numbers::pi));
	CHECK(r1.contains(1.0 * std::numbers::pi));
	CHECK(r1.contains(1.5 * std::numbers::pi));
	CHECK(!r1.contains(2.0 * std::numbers::pi));

	CHECK(!r1.containsInterior(0.0 * std::numbers::pi));
	CHECK(!r1.containsInterior(0.5 * std::numbers::pi));
	CHECK(r1.containsInterior(1.0 * std::numbers::pi));
	CHECK(!r1.containsInterior(1.5 * std::numbers::pi));
	CHECK(!r1.containsInterior(2.0 * std::numbers::pi));

	CircularRange r2(1.5 * std::numbers::pi, 0.5 * std::numbers::pi);

	CHECK(r2.contains(0.0 * std::numbers::pi));
	CHECK(r2.contains(0.5 * std::numbers::pi));
	CHECK(!r2.contains(1.0 * std::numbers::pi));
	CHECK(r2.contains(1.5 * std::numbers::pi));
	CHECK(r2.contains(2.0 * std::numbers::pi));

	CHECK(r2.containsInterior(0.0 * std::numbers::pi));
	CHECK(!r2.containsInterior(0.5 * std::numbers::pi));
	CHECK(!r2.containsInterior(1.0 * std::numbers::pi));
	CHECK(!r2.containsInterior(1.5 * std::numbers::pi));
	CHECK(r2.containsInterior(2.0 * std::numbers::pi));
}

TEST_CASE("Checking if circular ranges intersect") {
	CircularRange r1(0.5 * std::numbers::pi, 1.0 * std::numbers::pi);
	CircularRange r2(1.25 * std::numbers::pi, 1.5 * std::numbers::pi);
	CircularRange r3(1.0 * std::numbers::pi, 0.5 * std::numbers::pi);
	CircularRange r4(1.5 * std::numbers::pi, 1.25 * std::numbers::pi);

	CHECK(r1.intersects(r1));
	CHECK(!r1.intersects(r2));
	CHECK(r1.intersects(r3));
	CHECK(r1.intersects(r4));
	CHECK(!r2.intersects(r1));
	CHECK(r3.intersects(r1));
	CHECK(r4.intersects(r1));

	CHECK(r1.intersectsInterior(r1));
	CHECK(!r1.intersectsInterior(r2));
	CHECK(!r1.intersectsInterior(r3));
	CHECK(r1.intersectsInterior(r4));
	CHECK(!r2.intersectsInterior(r1));
	CHECK(!r3.intersectsInterior(r1));
	CHECK(r4.intersectsInterior(r1));
}
