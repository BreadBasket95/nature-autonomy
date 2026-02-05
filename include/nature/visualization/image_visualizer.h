#ifndef NATURE_IMAGE_VISUALIZER_H
#define NATURE_IMAGE_VISUALIZER_H

#include "nature/visualization/base_visualizer.h"
#include "nature/CImg.h"

namespace nature {
  namespace visualization {

    class ImageVisualizer : public VisualizerBase {

    public:
      /**
       * @brief Construct an image-based visualizer.
       * @details Initializes an empty CImg image; call initialize_display before drawing.
       */
      ImageVisualizer() = default;
      /**
       * @brief Initialize the image buffer and display window.
       * @param nx Width in pixels.
       * @param ny Height in pixels.
       * @return True if initialization succeeded.
       * @details Allocates an image buffer and display window via CImg.
       */
      bool initialize_display(int nx, int ny) override;
      /**
       * @brief Draw a circle into the image buffer.
       * @param x0 Center X coordinate in pixels.
       * @param y0 Center Y coordinate in pixels.
       * @param radius Circle radius in pixels.
       * @param color RGB color vector.
       * @details Uses CImg drawing primitives for visualization.
       */
      void draw_circle(const int x0, const int y0, int radius, const nature::utils::vec3 &color) override;
      /**
       * @brief Draw a point into the image buffer.
       * @param x0 X coordinate in pixels.
       * @param y0 Y coordinate in pixels.
       * @param color RGB color vector.
       * @details Uses CImg drawing primitives for visualization.
       */
      void draw_point(const int x0, const int y0, const nature::utils::vec3 &color) override;
      /**
       * @brief Draw a line into the image buffer.
       * @param x0 Start X coordinate.
       * @param y0 Start Y coordinate.
       * @param x1 End X coordinate.
       * @param y1 End Y coordinate.
       * @param color RGB color vector.
       * @details Uses CImg drawing primitives for visualization.
       */
      void draw_line(const int x0, const int y0, const int x1, const int y1, const nature::utils::vec3 &color) override;
      /**
       * @brief Present the current image buffer.
       * @details Displays the image in the CImg window.
       */
      void display() override;
      /**
       * @brief Save the current image buffer to disk.
       * @param file_name Output filename.
       * @details Writes the image using CImg image saving.
       */
      void save(const std::string &file_name) override;
      /**
       * @brief Save the current image buffer with a specific output size.
       * @param file_name Output filename.
       * @param nx Output width in pixels.
       * @param ny Output height in pixels.
       * @details Resizes the image before saving.
       */
      void save(const std::string &file_name, int nx, int ny) override;

    private:
      cimg_library::CImgDisplay disp_;
      cimg_library::CImg<float> image_;
    };

  }
}
#endif //NATURE_IMAGE_VISUALIZER_H
