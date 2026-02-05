#ifndef ASTAR_CELL_H
#define ASTAR_CELL_H

/// represents a single pixel in A* grid
class AStarCell {
  public:
    int idx;     // index in the flattened grid
    float cost;  // cost of traversing this pixel

    /**
     * @brief Construct a cell with index and cost.
     * @param i Index of the cell in the flattened grid.
     * @param c Traversal cost for the cell.
     * @details Used by the A* open list to track candidates.
     */
    AStarCell(int i, float c) : idx(i),cost(c) {}
};

// the top of the priority queue is the greatest element by default,
// but we want the smallest, so flip the sign
/**
 * @brief Compare two cells for priority queue ordering.
 * @param n1 First cell.
 * @param n2 Second cell.
 * @return True if n1 should come after n2 (higher cost).
 * @details Inverts comparison so std::priority_queue pops lowest cost first.
 */
bool operator<(const AStarCell &n1, const AStarCell &n2) {
  return n1.cost > n2.cost;
}

/**
 * @brief Compare two cells for index equality.
 * @param n1 First cell.
 * @param n2 Second cell.
 * @return True if the cell indices match.
 * @details Used to detect revisits in A* bookkeeping.
 */
bool operator==(const AStarCell &n1, const AStarCell &n2) {
  return n1.idx == n2.idx;
}

#endif
