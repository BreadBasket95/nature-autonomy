//
// Created by stefan on 2021-08-19.
//

#ifndef NATURE_MARKER_SPLINE_PLOTTER_H
#define NATURE_MARKER_SPLINE_PLOTTER_H

#include "nature/planning/local/spline_plotter.h"
#include "nature/visualization/base_visualizer.h"
#include "nature/messaging/message_types.h"
#include "nature/node/node_proxy.h"

namespace nature {
  namespace planning{

    class MarkerPlotter : public Plotter {
    public:
      /**
       * @brief Construct a marker-based plotter for candidate paths.
       * @param visualizer Visualizer backend for plot rendering.
       * @param cos_vis Visualization mode string for cost display.
       * @param node NodeProxy used to publish marker arrays.
       * @param w_c Weight for comfortability cost.
       * @param w_s Weight for static safety cost.
       * @param w_r Weight for rho deviation cost.
       * @param w_d Weight for dynamic safety cost.
       * @param w_t Weight for segmentation/terrain cost.
       * @param cost_vis_text_size_ Text size for cost labels.
       * @details Used by the local planner node to publish marker overlays in a
       *          ROS-free visualization path.
       */
      MarkerPlotter(std::shared_ptr<nature::visualization::VisualizerBase> visualizer, const std::string & cos_vis,
                    std::shared_ptr<nature::node::NodeProxy> node, float w_c, float w_s, float w_r, float w_d, float w_t,
                    float cost_vis_text_size_);
      /**
       * @brief Display the plot and optionally save to disk.
       * @param save True to save image output.
       * @param ofname Output filename when saving.
       * @param nx Output image width.
       * @param ny Output image height.
       * @details Publishes marker arrays representing candidate paths and costs.
       */
      virtual void Display(bool save, const std::string & ofname, int nx, int ny) override;

    private:
      /**
       * @brief Build a marker message with common formatting.
       * @param type Marker type identifier.
       * @param id Marker id.
       * @param is_blocked True if the marker represents a blocked path.
       * @return Marker message.
       * @details Encapsulates consistent coloring and sizing for marker outputs.
       */
      nature::msg::Marker get_marker_msg(int type, int id, bool is_blocked = false) const;
      std::shared_ptr<nature::node::NodeProxy> node_;
      std::string cost_vis_;
      float cost_vis_text_size_;
      std::shared_ptr<nature::node::Publisher<nature::msg::MarkerArray>> candidate_paths_publisher;
      float w_c_;
      float w_d_;
      float w_r_;
      float w_s_;
      float w_t_;
    };
  } // namespace planning
} // namespace nature

#endif //NATURE_MARKER_SPLINE_PLOTTER_H
