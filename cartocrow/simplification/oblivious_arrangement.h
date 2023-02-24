#pragma once

#include "../core/core.h"
#include "modifiable_arrangement.h"
#include "util.h"

namespace cartocrow::simplification {

/// Implements the \ref cartocrow::simplification::ModifiableArrangement 
/// "ModifiableArrangement" concept. It does not keep track of 
/// changes made to the arrangement: any change made is final.
template <MapType MT> class ObliviousArrangement {
  public:
	using Map = MT::Map;

	ObliviousArrangement(Map& inmap) : map(inmap){};

	~ObliviousArrangement() {}

	Map::Halfedge_handle mergeWithNext(Map::Halfedge_handle e, Number<Exact> cost) {
		return util::mergeWithNext(map, e);
	}

	Map& getMap() {
		return map;
	}

  private:
	Map& map;
};

} // namespace cartocrow::simplification