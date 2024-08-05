#include "aspect_ratio.h"

namespace cartocrow::treemap {
Number<K> aspect_ratio_square_percentage(Polygon<K> poly) {
	auto w = poly.bbox().x_span();
	auto h = poly.bbox().y_span();
	auto d = std::max(w, h);
	auto ar = (d * d) / poly.area();
	return ar;
}
}