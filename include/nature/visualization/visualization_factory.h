#ifndef NATURE_VISUALIZATION_FACTORY_H
#define NATURE_VISUALIZATION_FACTORY_H

#include "nature/visualization/base_visualizer.h"
#include "nature/visualization/image_visualizer.h"
#include "nature/planning/local/marker_spline_plotter.h"

namespace nature{
  namespace visualization{

    const std::string default_display = "x11";    // x11, opencv, markers, none

    /**
     * @brief Create a visualizer backend based on a display type string.
     * @param display_type Display type name (e.g., "image", "markers", "none").
     * @return Shared pointer to a VisualizerBase implementation.
     * @details Returns an ImageVisualizer for "image" and a null visualizer otherwise.
     */
    inline std::shared_ptr<nature::visualization::VisualizerBase> create_visualizer(const std::string & display_type){
      return display_type == "image" ? std::make_shared<nature::visualization::ImageVisualizer>()
                                     : std::make_shared<nature::visualization::VisualizerBase>();
    }

    /**
     * @brief Create a plotter for local planner visualization.
     * @param display_type Display type name (e.g., "markers" or "image").
     * @param cost_vis Cost visualization mode string.
     * @param node NodeProxy used to publish markers when applicable.
     * @param w_c Comfortability weight for display annotations.
     * @param w_s Static safety weight for display annotations.
     * @param w_r Path adherence weight for display annotations.
     * @param w_d Dynamic safety weight for display annotations.
     * @param w_t Segmentation weight for display annotations.
     * @param cost_vis_text_size Text size for cost annotations.
     * @return Shared pointer to a Plotter implementation.
     * @details Returns MarkerPlotter when display_type is "markers"; otherwise
     *          returns a generic Plotter with the selected visualizer.
     */
    inline std::shared_ptr<nature::planning::Plotter> create_local_path_plotter(const std::string & display_type, const std::string & cost_vis,
                                                                                 std::shared_ptr<nature::node::NodeProxy> node, float w_c, float w_s, float w_r, float w_d, float w_t, float cost_vis_text_size){
      auto visualizer = create_visualizer(display_type);
      return display_type == "markers" ? std::make_shared<nature::planning::MarkerPlotter>(visualizer, cost_vis, node, w_c, w_s, w_r, w_d, w_t, cost_vis_text_size)
                                       : std::make_shared<nature::planning::Plotter>(visualizer);
    }

  }
}


#endif //NATURE_VISUALIZATION_FACTORY_H
