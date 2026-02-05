/**
 * \class Plotter
 *
 * Class to plot the candidate paths, centerline, and map from the planner.
 * For debugging purposes.
 *
 * \author Chris Goodin
 *
 * \date 9/2/2020
 */
#ifndef SPLINE_PLOTTER_H
#define SPLINE_PLOTTER_H

#include "nature/nature_utils.h"
#include "nature/planning/local/candidate.h"
#include "nature/visualization/base_visualizer.h"

// messaging includes
#include "nature/messaging/message_types.h"


namespace nature {
namespace planning{

class Plotter {
public:
	/**
	 * @brief Construct a plotter for candidate path visualization.
	 * @param visualizer Visualizer backend for drawing.
	 * @details Stores the visualizer and initializes plot parameters.
	 */
	Plotter(std::shared_ptr<nature::visualization::VisualizerBase> visualizer);

	/**
	 * @brief Set the centerline to be plotted.
	 * @param path List of points representing the centerline.
	 * @details Used by local planner visualizations to show the reference path.
	 */
	void SetPath(std::vector<utils::vec2> path);

	/**
	 * @brief Add candidate paths to the plot.
	 * @param curves List of candidate paths to draw.
	 * @details Stored for rendering when Display is called.
	 */
	void AddCurves(std::vector<Candidate> curves);

	/**
	 * @brief Add the occupancy grid to be plotted.
	 * @param grid Occupancy grid to draw.
	 * @details Used for visualizing obstacle layout beneath candidate paths.
	 */
	void AddMap(nature::msg::OccupancyGrid grid);

	/**
	 * @brief Add global waypoints to be plotted.
	 * @param waypoints Waypoints path to draw.
	 * @details Draws the global path alongside local candidates.
	 */
	void AddWaypoints(nature::msg::Path waypoints);

	/**
	 * @brief Display the plot using default parameters.
	 * @details Convenience wrapper around the full Display method.
	 */
	void Display();


	/**
	 * @brief Display and optionally save the plot.
	 * @param save True to save, false to only display.
	 * @param ofname Output filename when saving.
	 * @param nx Output image width in pixels.
	 * @param ny Output image height in pixels.
	 * @details Uses the configured VisualizerBase implementation.
	 */
	virtual void Display(bool save, const std::string & ofname, int nx, int ny);

	/**
	 * @brief Get the current image dimensions for plotting.
	 * @return Dimensions as an integer vector (nx, ny).
	 * @details Useful when coordinating with external visualization outputs.
	 */
	utils::ivec2 GetDimensions(){
		utils::ivec2 dim(nx_, ny_);
		return dim;
	}

protected:
	std::vector<utils::vec2> path_;
	std::vector<utils::vec2> waypoints_;
	std::vector<Candidate> curves_;
  std::shared_ptr<nature::visualization::VisualizerBase> visualizer_;
  nature::msg::OccupancyGrid grid_;

	float x_lo_;
	float x_hi_;
	float y_lo_;
	float y_hi_;
	int nx_; 
	int ny_;
	float pixdim_;
	bool map_set_;
	/**
	 * @brief Convert Cartesian coordinates to pixel coordinates.
	 * @param x X coordinate in world frame.
	 * @param y Y coordinate in world frame.
	 * @return Pixel coordinates as integer vector.
	 * @details Uses plot bounds and pixel dimensions.
	 */
	utils::ivec2 CartesianToPixel(float x, float y);

};
} // namespace planning
} // namespace nature


#endif
