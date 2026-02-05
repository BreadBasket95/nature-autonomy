#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <nature/visualization/base_visualizer.h>
#include "nature/messaging/message_types.h"

namespace nature {
namespace planning{

/**
 * Astar map class with solve functions and map accessor methods.
 * The map holds obstacle values from 0 to 100. 0=no obstacle, 100=impassable.
 */
class Astar {
 public:
  /**
   * @brief Construct the A* planner with an optional visualizer.
   * @param visualizer Visualizer used for debug display; may be null.
   * @details Initializes internal buffers; call AllocateMap before solving.
   */
  Astar(std::shared_ptr<nature::visualization::VisualizerBase> visualizer);

  /**
   * @brief Destroy the A* planner instance.
   * @details Default cleanup of internal buffers.
   */
  ~Astar();

  /**
   * @brief Display the current map/path in the attached visualizer.
   * @details No-op if no visualizer is configured.
   */
  void Display();

  /**
   * @brief Save the current map visualization to disk.
   * @param ofname Output filename.
   * @details Delegates to the configured visualizer.
   */
  void SaveMap(std::string ofname);
  
	/**
	 * @brief Get the current path in world coordinates.
	 * @return Pointer to the path vector (list of [x,y] points).
	 * @details Used by global planning nodes to publish path results.
	 */
	std::vector<std::vector<float> > *GetCurrentPath() { return &path_world_; }

	/**
	 * @brief Get the current map cost grid.
	 * @return Pointer to the map vector.
	 * @details Exposes the current cost grid for visualization and debugging.
	 */
	std::vector<std::vector<float> > *GetCurrentMap() { return &map_; }

	/**
	 * @brief Get the current goal in world coordinates.
	 * @return Goal point [x,y] in world coordinates.
	 * @details Converts the internal grid index back to ENU coordinates.
	 */
	std::vector<float> GetCurrentGoal() {
		std::vector<int> gi = FoldIndex(goal_);
		std::vector<float> goal_world = IndexToPoint(gi);
		return goal_world;
	}

	/**
	 * @brief Get the minimum X (East) coordinate of the map.
	 * @return Lower-left X coordinate in world frame.
	 * @details Used to map between grid and world coordinates.
	 */
	float GetXMin() { return llx_; }

	/**
	 * @brief Get the minimum Y (North) coordinate of the map.
	 * @return Lower-left Y coordinate in world frame.
	 * @details Used to map between grid and world coordinates.
	 */
	float GetYMin() { return lly_; }

	/**
	 * @brief Get the grid resolution in meters.
	 * @return Map resolution in meters per cell.
	 * @details Used by planners and visualizers when scaling.
	 */
	float GetGridResolution() { return map_res_; }

	/**
	 * @brief Get the map width in cells.
	 * @return Number of horizontal cells.
	 * @details Reflects the allocated map size.
	 */
	int GetGridWidth() { return width_; }

	/**
	 * @brief Get the map height in cells.
	 * @return Number of vertical cells.
	 * @details Reflects the allocated map size.
	 */
	int GetGridHeight() { return height_; }

	/**
	 * @brief Plan a path using A*.
	 * @param grid Occupancy grid with obstacle costs.
	 * @param segmentation_grid Optional segmentation grid for context.
	 * @param goal Goal position [x,y] in world coordinates.
	 * @param position Current vehicle position [x,y] in world coordinates.
	 * @return Path as a list of [x,y] points in world coordinates.
	 * @details Converts inputs to grid indices, runs A*, and returns the path.
	 */
	std::vector<std::vector<float> > PlanPath(nature::msg::OccupancyGrid *grid, nature::msg::OccupancyGrid *segmentation_grid, std::vector<float> goal, std::vector<float> position);

  /**
   * @brief Allocate memory for the map and initialize values.
   * @param height Height of the map in cells.
   * @param width Width of the map in cells.
   * @param init_val Initial value for all cells, from 0 to 100.
   * @details Resizes internal buffers and fills with init_val.
   */
  void AllocateMap(int height, int width, int init_val);

  /**
   * @brief Set the value of cell (i,j).
   * @param i Vertical index of the cell to set.
   * @param j Horizontal index of the cell to set.
   * @param val_height Occupancy cost value to set, [0,100].
   * @param val_seg Segmentation value to set.
   * @details Updates both occupancy and segmentation weights.
   */
  void SetMapValue(int i, int j, int val_height, int val_seg);

  /**
   * @brief Get the occupancy value of cell (i,j).
   * @param i Vertical index of cell to get.
   * @param j Horizontal index of cell to get.
   * @return Occupancy weight value.
   * @details Reads from the flattened weight array.
   */
  int GetMapValue(int i, int j){return weights_[FlattenIndex(i,j)];}

  /**
   * @brief Set the goal cell.
   * @param i Vertical index of goal cell.
   * @param j Horizontal index of goal cell.
   * @details Stores the goal as a flattened index for A*.
   */
  void SetGoal(int i, int j){goal_ = FlattenIndex(i,j);}

  /**
   * @brief Set the start cell (current vehicle location).
   * @param i Vertical index of the vehicle location.
   * @param j Horizontal index of the vehicle location.
   * @details Stores the start as a flattened index for A*.
   */
  void SetStart(int i, int j){start_ = FlattenIndex(i,j);}

  /**
   * @brief Solve the A* search on the current map.
   * @return True if a path was found; false otherwise.
   * @details Populates internal path buffers for retrieval.
   */
  bool Solve();
 
  /**
   * @brief Get the current path as grid indices.
   * @return Path as a list of [i,j] indices.
   * @details Useful for debugging and visualization.
   */
  std::vector<std::vector<int> > GetPath(){return path_;}

  /**
   * @brief Set the ENU coordinates of the map origin.
   * @param x Easting of the lower-left corner.
   * @param y Northing of the lower-left corner.
   * @details Used for converting between grid and world coordinates.
   */
  void SetCornerCoords(float x, float y){
    llx_ = x;
    lly_ = y;
  }

  /**
   * @brief Set the resolution of the map cells in meters.
   * @param res Resolution in meters.
   * @details Affects conversions between grid indices and ENU coordinates.
   */
  void SetMapRes(float res){
    map_res_ = res;
  }

  /**
   * @brief Get the resolution of the map in meters.
   * @return Resolution in meters.
   * @details Used by downstream planners for scaling.
   */
  float GetRes(){return map_res_;}
  
  /**
   * @brief Convert a point in ENU to map indices.
   * @param x Easting coordinate.
   * @param y Northing coordinate.
   * @return Map indices [i,j].
   * @details Uses map origin and resolution to compute indices.
   */
  std::vector<int> PointToIndex(float x, float y){
    std::vector<int> c;
		c.resize(2);
    c[0] = (int)((x-llx_)/map_res_);
    c[1] = (int)((y-lly_)/map_res_);
    return c;
  }

  /**
   * @brief Convert a map index to ENU coordinates.
   * @param c Index of the map cell [i,j].
   * @return ENU coordinates [x,y] at the cell center.
   * @details Adds half-cell offset to return center coordinates.
   */
  std::vector<float> IndexToPoint(std::vector<int> c){
    std::vector<float> p;
    p.resize(2);
    p[0] = (c[0]+0.5f)*map_res_ + llx_;
    p[1] = (c[1]+0.5f)*map_res_ + lly_;
    return p;
  }

  /**
   * @brief Determine if a cell index is within map bounds.
   * @param c The index of the map cell [i,j].
   * @return True if inside the map; false otherwise.
   * @details Used to guard neighbor expansion during A*.
   */
  bool IsInMap(std::vector<int> c){
    bool isin = false;
    if (c[0]>=0 && c[0]<width_ && c[1]>=0 && c[1]<height_){
      isin = true;
    }
    return isin;
  }
  
  /**
   * @brief Set the dilation factor for obstacle expansion.
   * @param dfac The dilation factor in cells.
   * @details Expands obstacles to account for vehicle footprint.
   */
  void SetDilationFactor(int dfac){dfac_ = dfac;}

 private:
  /**
   * @brief Convert a flattened index to 2D indices.
   * @param n Flattened index.
   * @return Vector [i,j] of grid indices.
   * @details Inverse of FlattenIndex.
   */
  std::vector<int> FoldIndex(int n);
  
  /**
   * @brief Convert 2D indices to a flattened index.
   * @param i Row index.
   * @param j Column index.
   * @return Flattened index.
   * @details Used for compact storage of map weights.
   */
  int FlattenIndex(int i, int j){return j*width_+i;}
  
  /**
   * @brief Compute the A* heuristic between two cells.
   * @param i0 Start cell row.
   * @param j0 Start cell column.
   * @param i1 Goal cell row.
   * @param j1 Goal cell column.
   * @return Heuristic cost estimate.
   * @details Uses Euclidean distance in grid space.
   */
  float Heuristic(int i0, int j0, int i1, int j1);

  /// Flattened occupancy grid
  std::vector<int> weights_;

	/// unflattened occupancy grid
	std::vector<std::vector<float> > map_;


  ///height of the grid
  int height_;

  ///width of the grid
  int width_;

  ///flattened index of the goal point
  int goal_;

  ///flattened index of the start point
  int start_;

  /// map dilation factor
  int dfac_;

  /// calculated path
  std::vector<int> paths_;

  //std::vector<MapIndex> path_;
	std::vector<std::vector<int> > path_;
	std::vector<std::vector<float> > path_world_;

  /**
   * @brief Extract the path from the search tree.
   * @return True if a valid path was extracted.
   * @details Backtracks from goal to start using the predecessor list.
   */
  bool ExtractPath();
  /**
   * @brief Smooth the extracted path.
   * @details Applies line-of-sight checks to remove unnecessary waypoints.
   */
  void PostSmoothing();
  /**
   * @brief Check line of sight between two grid cells.
   * @param p0 Start cell index [i,j].
   * @param p1 End cell index [i,j].
   * @return True if line of sight exists without obstacles.
   * @details Used by path smoothing to shortcut segments.
   */
  bool LineOfSight(std::vector<int> p0, std::vector<int> p1);

  float llx_,lly_;
  float map_res_;

  std::shared_ptr<nature::visualization::VisualizerBase> visualizer_;
};

} // namespace planning
} // namespace nature

#endif
