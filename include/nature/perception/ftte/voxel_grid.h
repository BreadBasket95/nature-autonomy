/*
Non-Commercial License - Mississippi State University Off-Road Traversability Algorithm

REPO: https://gitlab.com/cgoodin/off_road_traversability

CONTACT: cgoodin@cavs.msstate.edu

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

CITATION:
Goodin, C., Dabbiru, L., Hudson, C., Mason, G., Carruth, D., & Doude, M. (2021, April). 
Fast terrain traversability estimation with terrestrial lidar in off-road autonomous navigation. 
In Unmanned Systems Technology XXIII (Vol. 11758, p. 117580O). International Society for Optics and Photonics.

NOTICE:
Do not share or distribute. Software is authorized for use only by the approved recepient.

Copyright 2022 (C) Mississippi State University
*/
#ifndef VOXEL_GRID_H_
#define VOXEL_GRID_H_

// c++ includes
#include <limits>
#include <vector>
// messaging includes
#include "nature/messaging/message_types.h"
// project includes
#include "nature/perception/ftte/vehicle.h"
#include "nature/perception/ftte/traverse_cell.h"
#include "nature/perception/ftte/plane.h"
#include "nature/thirdparty/glm/glm.hpp"
#include "nature/thirdparty/CImg.h"


namespace traverselib {

class VoxelGrid {
public:

	/**
	 * @brief Construct an uninitialized voxel grid.
	 * @details Sets initialized_ false; caller must call Initialize or use
	 *          the parameterized constructor before adding points.
	 */
	VoxelGrid(){
		initialized_ = false;
	}

	/**
	 * @brief Construct and initialize a voxel grid.
	 * @param llc Lower-left corner of the grid (min x,y,z).
	 * @param urc Upper-right corner of the grid (max x,y,z).
	 * @param res Voxel resolution in meters.
	 * @details Allocates grid storage and prepares traversal metrics.
	 */
	VoxelGrid(glm::vec3 llc, glm::vec3 urc, float res);

	/**
	 * @brief Initialize the grid with bounds and resolution.
	 * @param llc Lower-left corner of the grid.
	 * @param urc Upper-right corner of the grid.
	 * @param res Voxel resolution in meters.
	 * @details Allocates and resets internal buffers; marks grid initialized.
	 */
	void Initialize(glm::vec3 llc, glm::vec3 urc, float res);

	/**
	 * @brief Move the grid to new bounds.
	 * @param new_llc New lower-left corner.
	 * @param new_urc New upper-right corner.
	 * @details Shifts grid origin and clears data outside the new window.
	 */
	void Move(glm::vec3 new_llc, glm::vec3 new_urc);

	/**
	 * @brief Plot the estimated ground surface.
	 * @details Generates a visualization for debugging traversability estimates.
	 */
	void PlotGround(); 

	/**
	 * @brief Save a ground surface plot to disk.
	 * @param fname Output filename.
	 * @details Renders the ground plot and writes it as an image file.
	 */
	void SaveGroundPlot(std::string fname);

	/**
	 * @brief Save a slice plot to disk.
	 * @param fname Output filename.
	 * @details Captures a planar slice of the voxel grid for debugging.
	 */
	void SaveSlicePlot(std::string fname);

	/**
	 * @brief Convert traversability data into an OccupancyGrid message.
	 * @param row_major True to output in row-major order.
	 * @return OccupancyGrid message of traversability values.
	 * @details Used to feed planners with a grid representation of terrain cost.
	 */
	nature::msg::OccupancyGrid GetTraversabilityAsOccupancyGrid(bool row_major);
	
	/**
	 * @brief Write grid statistics to a text file.
	 * @param fname Output filename.
	 * @details Dumps metrics like slope and roughness extrema for analysis.
	 */
	void WriteStats(std::string fname);

	/**
	 * @brief Get grid dimensions in voxels.
	 * @return Dimensions as glm::ivec3 (nx, ny, nz).
	 * @details Useful for iterating over grid cells.
	 */
	glm::ivec3 GetDim() { return dim_; }

	/**
	 * @brief Compute slope values for each grid cell.
	 * @details Uses local plane fits or height differences depending on settings.
	 */
	void CalculateSlope();
	/**
	 * @brief Compute roughness values for each grid cell.
	 * @details Calculates RMS roughness from local elevation statistics.
	 */
	void CalculateRoughness();

	

	/**
	 * @brief Plot the slope map.
	 * @details Visualizes per-cell slope for debugging and tuning.
	 */
	void PlotSlope();
	/**
	 * @brief Plot the vegetation density map.
	 * @details Visualizes vegetation density used in traversability scoring.
	 */
	void PlotVegDensity();
	/**
	 * @brief Plot the roughness map.
	 * @details Visualizes RMS roughness per cell.
	 */
	void PlotRoughness();
	/**
	 * @brief Plot the traversability map.
	 * @details Visualizes combined traversability score for each cell.
	 */
	void PlotTraversability();
	/**
	 * @brief Plot the confidence map.
	 * @details Visualizes confidence values associated with traversability.
	 */
	void PlotConfidence();
	/**
	 * @brief Plot the RCI map.
	 * @details Visualizes rolling resistance/soil metric per cell.
	 */
	void PlotRci();

	/**
	 * @brief Overlay traversability on an input image and plot.
	 * @param image Input image to overlay (copied).
	 * @param prefix Prefix for output window titles/files.
	 * @details Combines visualization with external imagery for debugging.
	 */
	void PlotTravAndImage(cimg_library::CImg<float> image, std::string prefix);

	/**
	 * @brief Save the RCI plot to disk.
	 * @param fname Output filename.
	 * @details Renders and writes the RCI visualization.
	 */
	void SaveRciPlot(std::string fname);
	/**
	 * @brief Save the confidence plot to disk.
	 * @param fname Output filename.
	 * @details Renders and writes confidence visualization.
	 */
	void SaveConfidencePlot(std::string fname);
	/**
	 * @brief Save the vegetation density plot to disk.
	 * @param fname Output filename.
	 * @details Renders and writes vegetation density visualization.
	 */
	void SaveVegDensityPlot(std::string fname);
	/**
	 * @brief Save the traversability plot to disk.
	 * @param fname Output filename.
	 * @details Renders and writes traversability visualization.
	 */
	void SaveTraversabilityPlot(std::string fname);
	/**
	 * @brief Save the slope plot to disk.
	 * @param fname Output filename.
	 * @details Renders and writes slope visualization.
	 */
	void SaveSlopePlot(std::string fname);
	/**
	 * @brief Save the roughness plot to disk.
	 * @param fname Output filename.
	 * @details Renders and writes roughness visualization.
	 */
	void SaveRoughPlot(std::string fname);

	/**
	 * @brief Configure the vehicle model used for scoring.
	 * @param vehicle Vehicle model to copy into the grid.
	 * @details Updates vegetation height thresholds and resolution-dependent data.
	 */
	void SetVehicle(traverselib::Vehicle vehicle) {
		vehicle_ = vehicle;
		veg_hi_lo_cutoff_.x = vehicle_.GetBumperHeight();
		veg_hi_lo_cutoff_.y = vehicle_.GetRoofHeight();
		vehicle_.SetGridResolution(res_);
	}

	/**
	 * @brief Get traversability at a world-space point.
	 * @param p Query point in world coordinates.
	 * @return Traversability score at that location.
	 * @details Converts the point to a grid index and samples the stored value.
	 */
	float GetTraversabilityAtPoint(glm::vec3 p);

	/**
	 * @brief Get traversability at a grid cell.
	 * @param i Cell index in X.
	 * @param j Cell index in Y.
	 * @return Traversability score for the cell.
	 * @details Used by planners that sample a local costmap.
	 */
	float GetTraversabilityAtCell(int i, int j);

	/**
	 * @brief Get traversability at a grid cell by vector index.
	 * @param idx Cell indices (x,y,z); z is ignored.
	 * @return Traversability score for the cell.
	 * @details Convenience overload that delegates to (i,j).
	 */
	float GetTraversabilityAtCell(glm::ivec3 idx) {
		return GetTraversabilityAtCell(idx.x, idx.y);
	}

	/**
	 * @brief Add registered lidar points to the grid.
	 * @param points Point cloud in world coordinates.
	 * @param position Vehicle position associated with the scan.
	 * @details Updates cell statistics, ground estimates, and traversability.
	 */
	void AddRegisteredPoints(std::vector<glm::vec3> &points, glm::vec3 position);

	/**
	 * @brief Compute local averages used for smoothing.
	 * @details Aggregates neighborhood statistics for roughness and slope smoothing.
	 */
	void CalculateLocalAvg();

	/**
	 * @brief Get the fallback traversability value.
	 * @return Default traversability score.
	 * @details Used when a cell has insufficient data.
	 */
	float GetDefaultTraversability() { return default_traversability_; }

	/**
	 * @brief Set the fallback traversability value.
	 * @param dft Default traversability score.
	 * @details Used to fill unknown cells for planning.
	 */
	void SetDefaultTraversability(float dft) { default_traversability_ = dft; }

	/**
	 * @brief Enable or disable plane fitting for slope estimation.
	 * @param use_planes True to use plane fitting.
	 * @details Plane fitting can improve slope accuracy on sparse points.
	 */
	void UsePlaneFitting(bool use_planes){ use_plane_fitting_ = use_planes; }

	/**
	 * @brief Set the averaging radius used for local smoothing.
	 * @param ar Radius in meters.
	 * @details Affects the size of the neighborhood for local average calculations.
	 */
	void SetAveragingRadius(float ar){averaging_radius_ = ar;}

	/**
	 * @brief Access the vehicle model used by this grid.
	 * @return Pointer to the internal vehicle model.
	 * @details Allows external configuration of vehicle parameters.
	 */
	traverselib::Vehicle *GetVehicle(){return &vehicle_;}

	/**
	 * @brief Enable or disable timing diagnostics.
	 * @param to_print True to print timing info.
	 * @details Used for profiling the FTTE pipeline.
	 */
	void SetPrintTimingInfo(bool to_print){ print_timing_info_ = to_print; }

	/**
	 * @brief Check whether the grid is initialized.
	 * @return True if Initialize has been called successfully.
	 * @details Helps guard against use before configuration.
	 */
	bool Initialized(){ return initialized_; }

protected:

	/**
	 * @brief Check intersection between two line segments in 2D.
	 * @param origin Start point of first segment.
	 * @param endpoint End point of first segment.
	 * @param line_p0 Start point of second segment.
	 * @param line_p1 End point of second segment.
	 * @return True if the segments intersect.
	 * @details Used by ray marching and visibility checks.
	 */
	bool LineLineIntersect(glm::vec2 origin, glm::vec2 endpoint, glm::vec2 line_p0, glm::vec2 line_p1);
	/**
	 * @brief Check intersection between a line segment and a grid cell box.
	 * @param origin Start point of the segment.
	 * @param endpoint End point of the segment.
	 * @param box_idx Grid cell index of the box.
	 * @return True if the line intersects the cell.
	 * @details Used by ray marching to mark traversed cells.
	 */
	bool LineBoxIntersect(glm::vec2 origin, glm::vec2 endpoint, glm::ivec2 box_idx);
	/**
	 * @brief Determine if a set of points are colinear.
	 * @param points Input points to test.
	 * @return True if points lie on a straight line.
	 * @details Used to detect degenerate plane fits.
	 */
	bool ArePointsColinear(std::vector<glm::vec3> points);
	/**
	 * @brief Ray-march between two points through the grid.
	 * @param start_point Ray start in world coordinates.
	 * @param end_point Ray end in world coordinates.
	 * @details Updates grid cells along the path for visibility/occupancy metrics.
	 */
	void RayMarching(glm::vec3 start_point, glm::vec3 end_point);
	/**
	 * @brief Draw the vehicle path onto an image.
	 * @param img Image to modify in place.
	 * @details Used to overlay trajectory on diagnostic plots.
	 */
	void AddVehiclePathToImage(cimg_library::CImg<float> &img);
	/**
	 * @brief Estimate the ground surface from accumulated points.
	 * @details Fits planes or uses min/max stats to establish ground height.
	 */
	void CalculateGround();
	
	// utility methods
	/**
	 * @brief Convert a 3D point to a grid index.
	 * @param p Point in world coordinates.
	 * @return Grid index (x,y,z).
	 * @details Uses grid bounds and resolution to compute indices.
	 */
	glm::ivec3 PointToIndex(glm::vec3 p);
	/**
	 * @brief Convert a 2D point to a grid index.
	 * @param p Point in world coordinates.
	 * @return Grid index (x,y).
	 * @details Used when operating on 2D grid projections.
	 */
	glm::ivec2 PointToIndex(glm::vec2 p);
	/**
	 * @brief Convert a grid index to a world-space point.
	 * @param idx Grid index (x,y,z).
	 * @return World coordinate corresponding to the cell center.
	 * @details Used for visualization and debugging.
	 */
	glm::vec3 IndexToPoint(glm::ivec3 idx);
	/**
	 * @brief Check whether a 3D index lies inside the grid bounds.
	 * @param idx Grid index to test.
	 * @return True if inside bounds.
	 * @details Guards array access when ray marching.
	 */
	bool InGrid(glm::ivec3 idx);
	/**
	 * @brief Check whether a 2D index lies inside the grid bounds.
	 * @param idx Grid index to test.
	 * @return True if inside bounds.
	 * @details Used for 2D projections and plotting.
	 */
	bool InGrid(glm::ivec2 idx);

	bool use_plane_fitting_;
	bool print_timing_info_;
	bool initialized_;

	// map data
	std::vector<std::vector<TraverseCell> > cells_;
	std::vector<std::vector<float> > ground_;
	std::vector<std::vector<float> > local_avg_;
	std::vector<std::vector<Plane> > fit_planes_;
	std::vector<std::vector<float> > slope_;
	std::vector<std::vector<float> > roughness_;
	std::vector<std::vector<float> > impermeability_; 
	std::vector<std::vector<float> > rci_;
	std::vector<glm::vec2> vehicle_path_;
	float max_slope_;
	float max_impermeability_; 
	float max_roughness_;
	float min_ground_;
	float max_ground_;
	float rci_max_;

	traverselib::Vehicle vehicle_;

	// parameters
	float res_;
	float res_squared_;
	glm::vec3 llc_;
	glm::vec3 urc_;

	glm::ivec3 dim_;

	float averaging_radius_;
	glm::vec2 veg_hi_lo_cutoff_;

	float default_traversability_;

	// plotting displays
	/**
	 * @brief Draw a slope visualization image.
	 * @return Image representing slope values.
	 * @details Used internally by PlotSlope/SaveSlopePlot.
	 */
	cimg_library::CImg<float> DrawSlope();
	/**
	 * @brief Draw a ground height visualization image.
	 * @return Image representing ground height values.
	 * @details Used internally by PlotGround/SaveGroundPlot.
	 */
	cimg_library::CImg<float> DrawGround();
	/**
	 * @brief Draw a vegetation density visualization image.
	 * @return Image representing vegetation density.
	 * @details Used internally by PlotVegDensity/SaveVegDensityPlot.
	 */
	cimg_library::CImg<float> DrawVegDensity();
	/**
	 * @brief Draw a roughness visualization image.
	 * @return Image representing roughness.
	 * @details Used internally by PlotRoughness/SaveRoughPlot.
	 */
	cimg_library::CImg<float> DrawRoughness();
	/**
	 * @brief Draw a traversability visualization image.
	 * @return Image representing traversability scores.
	 * @details Used internally by PlotTraversability/SaveTraversabilityPlot.
	 */
	cimg_library::CImg<float> DrawTraversability();
	/**
	 * @brief Draw a confidence visualization image.
	 * @return Image representing confidence values.
	 * @details Used internally by PlotConfidence/SaveConfidencePlot.
	 */
	cimg_library::CImg<float> DrawConfidence();
	/**
	 * @brief Draw an RCI visualization image.
	 * @return Image representing RCI values.
	 * @details Used internally by PlotRci/SaveRciPlot.
	 */
	cimg_library::CImg<float> DrawRci();

	cimg_library::CImgDisplay ground_disp_;
	cimg_library::CImgDisplay slice_disp_;
	cimg_library::CImgDisplay slope_disp_;
	cimg_library::CImgDisplay rough_disp_;
	cimg_library::CImgDisplay veg_disp_;
	cimg_library::CImgDisplay rci_disp_;
	cimg_library::CImgDisplay trav_disp_;
	cimg_library::CImgDisplay conf_disp_;

	std::vector<glm::ivec2> path_taken_;
	int max_num_points_;
	float elev_avg_;

	bool local_avg_complete_;
};

} // namespace traverselib

#endif
