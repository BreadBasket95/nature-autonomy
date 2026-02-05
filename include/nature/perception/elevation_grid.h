/**
 * \class ElevationGrid
 *
 * A slope-based obstacle detection algorithm. 
 * The world is divided into 2D cells. 
 * The highest and lowest point are used to calculate the slope in each cell.
 * Cells that exceed a slope threshold are flagged as obstcles.
 *
 * \author Chris Goodin
 *
 * \date 9/3/2020
 */
#include <vector>
#include <limits>
#include <string>
#include "nature/messaging/message_types.h"

namespace nature{
namespace perception{

struct Cell{
    float low = std::numeric_limits<float>::max();
    float high = std::numeric_limits<float>::lowest();
    float highest = std::numeric_limits<float>::lowest();
    float second_highest = std::numeric_limits<float>::lowest();
    float height = 0.0f;
    bool filled = false;
    //float slope_x = 0.0f;
    //float slope_y = 0.0f;
    float slope = 0.0f;
    bool obstacle = false;
    bool has_dilated = false;
    uint8_t dilated_val = 0;
    float terrain = 0.0f;
};

class ElevationGrid{
  public:
    /**
     * @brief Construct an elevation grid with default parameters.
     * @details Initializes internal sizing and thresholds to defaults; callers
     *          should configure resolution, size, and thresholds before use.
     */
    ElevationGrid();

    /**
     * @brief Destroy the elevation grid and release storage.
     * @details Uses default cleanup; provided for symmetry with ROS-era design.
     */
    ~ElevationGrid();

    /**
     * @brief Add points to the grid and classify obstacles.
     * @param point_cloud PointCloud message to process; may be modified in place.
     * @return Vector of surface points that remain after obstacle filtering.
     * @details Updates per-cell min/max elevation, computes slopes, and flags
     *          obstacles. The input cloud is trimmed to obstacle points while
     *          the returned vector represents non-obstacle surface points.
     *          This feeds mapping and local planning stages.
     */
    std::vector<nature::msg::Point32> AddPoints(nature::msg::PointCloud &point_cloud);

    /**
     * @brief Check whether segmentation channels are present in the last input.
     * @return True if segmentation data has been observed.
     * @details Used by downstream consumers to decide whether to use class labels.
     */
    bool has_segmentation() const { return has_segmentation_; }

    /**
     * @brief Set a square grid size and resize internal storage.
     * @param s Width and height in meters.
     * @details Updates both width and height and recalculates grid dimensions.
     */
    void SetSize(float s){
        width_ = s;
        height_ = s;
        ResizeGrid();
    }

    /**
     * @brief Set grid width and height and resize internal storage.
     * @param width Grid width in meters.
     * @param height Grid height in meters.
     * @details Recomputes grid dimensions to match the new extents.
     */
    void SetSize(float width,float height){
        width_ = width;
        height_ = height;
        ResizeGrid();
    }

    /**
     * @brief Set grid resolution and resize internal storage.
     * @param r Cell resolution in meters.
     * @details Recomputes grid dimensions; affects slope computation granularity.
     */
    void SetRes(float r){
        res_ = r;
        ResizeGrid();
    }

    /**
     * @brief Set the slope threshold used to mark obstacles.
     * @param tr Slope threshold.
     * @details Cells with slope above this value are flagged as obstacles.
     */
    void SetSlopeThreshold(float tr){
        thresh_ = tr;
    }

    /**
     * @brief Enable or disable persistence of obstacle markings.
     * @param persist True to keep obstacles across frames.
     * @details Useful for stabilizing maps in sparse sensor updates.
     */
    void SetPersistentObstacles(bool persist){ persistent_obstacles_ = persist; }

    /**
     * @brief Enable or disable stitching of incoming points.
     * @param stitch_points True to stitch, false to use only current frame.
     * @details Stitching blends successive point clouds for denser grids.
     */
    void SetStitchPoints(bool stitch_points){ stitch_points_ = stitch_points; }

    /**
     * @brief Enable or disable filtering of highest points per cell.
     * @param filter_high True to filter highest point outliers.
     * @details Helps remove spurious spikes from vegetation returns.
     */
    void SetFilterHighest(bool filter_high){ filter_highest_ = filter_high; }

    /**
     * @brief Enable or disable elevation usage for obstacles.
     * @param use_elevation True to use elevation differences in slope logic.
     * @details Allows the algorithm to focus on slope only or include absolute height.
     */
    void SetUseElevation(bool use_elevation){
        use_elevation_ = use_elevation;
    }

    /**
     * @brief Clear grid cell data and reset obstacle flags.
     * @details Used when resetting the map or on reconfiguration.
     */
    void ClearGrid();

    /**
     * @brief Enable or disable dilation of obstacle cells.
     * @param use_dil True to enable dilation.
     * @details Dilation expands obstacles to account for vehicle footprint.
     */
    void UseDilation(bool use_dil){
        dilate_ = use_dil;
    }

    /**
     * @brief Build an OccupancyGrid message from the current cell data.
     * @param row_major True to order data row-major; false for column-major.
     * @param is_segmentation True to emit segmentation values instead of obstacles.
     * @return OccupancyGrid message representing obstacles or segmentation.
     * @details Converts internal grid state to a message for planners and visualizers.
     */
    nature::msg::OccupancyGrid GetGrid(bool row_major=false, bool is_segmentation=false);

    /**
     * @brief Set the lower-left corner of the grid.
     * @param llx X coordinate of the lower-left corner.
     * @param lly Y coordinate of the lower-left corner.
     * @details Used to position the grid in a global map frame.
     */
    void SetCorner(float llx, float lly){
        llx_ = llx;
        lly_ = lly;
    }

    /**
     * @brief Configure obstacle dilation parameters.
     * @param grid_dilate True to enable dilation.
     * @param grid_dilate_x Dilation distance in X.
     * @param grid_dilate_y Dilation distance in Y.
     * @param grid_dilate_proportion Proportion of dilation to apply.
     * @details Used to buffer obstacles based on vehicle footprint assumptions.
     */
    void SetDilation(bool grid_dilate, float grid_dilate_x, float grid_dilate_y, float grid_dilate_proportion){
        dilate_ = grid_dilate;
        grid_dilate_x_ = grid_dilate_x;
        grid_dilate_y_ = grid_dilate_y;
        grid_dilate_proportion_ = grid_dilate_proportion;
    }


  private:
    /**
     * @brief Map a cell to an occupancy grid value.
     * @param cell Cell to convert.
     * @return Occupancy value in [0, GRID_MAX_VALUE].
     * @details Encodes obstacle/slope state into a single uint8 value.
     */
    uint8_t GetGridCellValue(const Cell & cell) const;
    /**
     * @brief Resize the internal grid storage to match current dimensions.
     * @details Recomputes nx_/ny_ from width/height/resolution and clears cells.
     */
    void ResizeGrid();
    /**
     * @brief Update any cached image/visualization buffers.
     * @details Called after grid updates to prepare visualization output.
     */
    void FillImage();
    std::vector< std::vector<Cell> > cells_;
    float width_;
    float height_;
    float res_;
    float thresh_;
    int nx_,ny_;
    bool first_display_;
    bool dilate_;
    float llx_;
    float lly_;
    float grid_dilate_x_;
    float grid_dilate_y_;
    float grid_dilate_proportion_;
    bool use_elevation_;
    bool stitch_points_;
    bool persistent_obstacles_;
    bool filter_highest_;
    const uint8_t GRID_MAX_VALUE = 100;
    const float GRID_SLOPE_MULT = 50.0f;
    bool has_segmentation_ = false;
};

} // namespace perception
} // namespace nature
