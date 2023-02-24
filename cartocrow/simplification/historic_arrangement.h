#pragma once

#include "../core/core.h"
#include "modifiable_arrangement.h"
#include "util.h"

namespace cartocrow::simplification {

/// Concept to express that a pointer to a class \f$D\f$ can be stored via the 
/// \ref cartocrow::simplification::MapType "MapType" \f$MT\f$, via the described 
/// functions.
template <class MT, class D>
concept EdgeStoredHistory = requires(typename MT::Map::Halfedge_handle e, D* d) {
	requires MapType<MT>;

	/// Sets the data for a \ref HistoricArrangement
	{MT::histSetData(e, d)};
	/// Retrieves the data for a \ref HistoricArrangement
	{ MT::histGetData(e) } -> std::same_as<D*>;
};

/// The history of an edge, including pointers to other steps in the history that
/// it may have overriden. This struct is used by a \ref HistoricArrangement.
template <MapType MT> struct EdgeHistory {

	using Map = MT::Map;

	EdgeHistory(Map::Halfedge_handle he, EdgeHistory<MT>* p, EdgeHistory<MT>* n,
	            EdgeHistory<MT>* pt, EdgeHistory<MT>* nt, int pc, Number<Exact> pmc, Point<Exact> pl)
	    : halfedge(he), prev(p), next(n), prev_twin(pt), next_twin(nt), post_complexity(pc),
	      post_maxcost(pmc), pre_loc(pl) {}

	// Pointer to the current halfedge
	Map::Halfedge_handle halfedge;

	EdgeHistory<MT>* prev = nullptr;
	EdgeHistory<MT>* next = nullptr;
	EdgeHistory<MT>* prev_twin = nullptr;
	EdgeHistory<MT>* next_twin = nullptr;

	int post_complexity;
	Number<Exact> post_maxcost;
	Point<Exact> pre_loc;
};

/// This historic arrangement keeps track of the operations performed, by storing 
/// this in the edges of the map. It implements the \ref 
/// cartocrow::simplification::ModifiableArrangementWithHistory 
/// "ModifiableArrangementWithHistory" concept, requiring \ref 
/// cartocrow::simplification::EdgeStoredHistory "EdgeStoredHistory" on the 
/// maptype.
template <MapType MT> requires(EdgeStoredHistory<MT, EdgeHistory<MT>>) class HistoricArrangement {
  public:
	using Map = MT::Map;

	HistoricArrangement(Map& inmap);

	~HistoricArrangement();

	Map& getMap();

	
	// Recovers the result as if the simplification algorithm was run once with 
	/// complexity parameter \f$c\f$.
	void recallComplexity(int c);
	// Recovers the result as if the simplification algorithm was run once with 
	/// threshold parameter \f$t\f$.
	void recallThreshold(Number<Exact> t);

	/// Tests whether any operations have been undone. Returns true iff no 
	/// operations were undone.
	bool atPresent();

	/// Undo one operation, if one exists.
	void backInTime();

	/// Redo one operation, if one exists.
	void forwardInTime();

	// From ModifiableArrangementWithHistory
	Map::Halfedge_handle mergeWithNext(Map::Halfedge_handle e, Number<Exact> cost);
	void goToPresent();

  private:
	Number<Exact> max_cost;
	Map& map;
	int in_complexity;
	std::vector<EdgeHistory<MT>*> history;
	std::vector<EdgeHistory<MT>*> undone;
};

} // namespace cartocrow::simplification

#include "historic_arrangement.hpp"