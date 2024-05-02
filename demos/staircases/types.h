//
// Created by steven on 5/2/24.
//

#ifndef CARTOCROW_TYPES_H
#define CARTOCROW_TYPES_H

#include "cartocrow/core/core.h"
#include "cartocrow/core/ipe_reader.h"
#include "cartocrow/renderer/geometry_painting.h"
#include "cartocrow/renderer/geometry_widget.h"

using namespace cartocrow;
using namespace cartocrow::renderer;

template <class K> using Rectangle = CGAL::Iso_rectangle_2<K>;
typedef Inexact K;

#endif //CARTOCROW_TYPES_H
