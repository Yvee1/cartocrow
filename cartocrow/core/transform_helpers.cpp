#include "transform_helpers.h"

namespace cartocrow {
CGAL::Aff_transformation_2<Inexact> fitInto(const Box& toFit, const Box& into) {
	return fitInto(Rectangle<Inexact>(toFit), Rectangle<Inexact>(into));
}
CGAL::Aff_transformation_2<Inexact> fitInto(const Rectangle<Inexact>& toFit, const Box& into) {
	return fitInto(toFit, Rectangle<Inexact>(into));
}
CGAL::Aff_transformation_2<Inexact> fitInto(const Box& toFit, const Rectangle<Inexact>& into) {
	return fitInto(Rectangle<Inexact>(toFit), into);
}
}