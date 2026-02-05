#ifndef NATURE_VISUALIZER_BASE_H
#define NATURE_VISUALIZER_BASE_H

#include <nature/nature_utils.h>

namespace nature{
  namespace visualization{

    /**
    * Base null visualizer with no image display support.
    * Used when an external visualization backend is active and no image display is required.
    */
    class VisualizerBase{
    public:
      /**
       * @brief Construct a base visualizer.
       * @details Default implementation is a no-op visualizer.
       */
      VisualizerBase() = default;
      /**
       * @brief Initialize the display surface.
       * @param nx Width in pixels.
       * @param ny Height in pixels.
       * @return True if initialization succeeded.
       * @details Base implementation returns false to indicate no display.
       */
      virtual bool initialize_display(int nx, int ny){ return false; }
      /**
       * @brief Draw a point into the visualizer.
       * @param x0 X coordinate in pixels.
       * @param y0 Y coordinate in pixels.
       * @param color RGB color vector.
       * @details Base implementation is a no-op; override in concrete visualizers.
       */
      virtual void draw_point(const int x0, const int y0, const nature::utils::vec3 & color){}
      /**
       * @brief Draw a circle into the visualizer.
       * @param x0 Center X coordinate in pixels.
       * @param y0 Center Y coordinate in pixels.
       * @param radius Circle radius in pixels.
       * @param color RGB color vector.
       * @details Base implementation is a no-op.
       */
      virtual void draw_circle(const int x0, const int y0, int radius, const nature::utils::vec3 & color){}
      /**
       * @brief Draw a line into the visualizer.
       * @param x0 X coordinate of the start point.
       * @param y0 Y coordinate of the start point.
       * @param x1 X coordinate of the end point.
       * @param y1 Y coordinate of the end point.
       * @param color RGB color vector.
       * @details Base implementation is a no-op.
       */
      virtual void draw_line(const int x0, const int y0, const int x1, const int y1, const nature::utils::vec3 & color){}
      /**
       * @brief Present the current frame.
       * @details Base implementation does nothing.
       */
      virtual void display(){}
      /**
       * @brief Save the current frame to disk.
       * @param file_name Output filename.
       * @details Base implementation does nothing.
       */
      virtual void save(const std::string & file_name){}
      /**
       * @brief Save the current frame with a specific output size.
       * @param file_name Output filename.
       * @param nx Output width in pixels.
       * @param ny Output height in pixels.
       * @details Base implementation does nothing.
       */
      virtual void save(const std::string & file_name, int nx, int ny){}
    };

  }
}


#endif //NATURE_VISUALIZER_BASE_H
