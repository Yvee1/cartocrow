#include "treemap_painting.h"
#include "cartocrow/core/core.h"

namespace cartocrow::treemap {
Color color_at(double x, double y) {
	//r1 = lefttop, r2 = righttop, r3 = leftbottom, r4= rightbottom
	//linear interpolation between the four colors

	double r1 = 214;
	double g1 = 216;
	double b1 = 255;

	double r2 = 255;
	double g2 = 217;
	double b2 = 66;

	double r3 = 80;
	double g3 = 93;
	double b3 = 242;

	double r4 = 148;
	double g4 = 52;
	double b4 = 0;

	double rHor1 = (r2 - r1) * x + r1;
	double gHor1 = (g2 - g1) * x + g1;
	double bHor1 = (b2 - b1) * x + b1;

	double rHor2 = (r4 - r3) * x + r3;
	double gHor2 = (g4 - g3) * x + g3;
	double bHor2 = (b4 - b3) * x + b3;

	double r = (rHor2 - rHor1) * y + rHor1;
	double g = (gHor2 - gHor1) * y + gHor1;
	double b = (bHor2 - bHor1) * y + bHor1;

	return Color((int) r, (int) g, (int) b);
}
}
