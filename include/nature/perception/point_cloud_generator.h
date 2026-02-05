#ifndef NATURE_POINT_CLOUD_GENERATOR_H
#define NATURE_POINT_CLOUD_GENERATOR_H

#include "nature/nature_utils.h"

namespace  nature {
  namespace perception {
    class PointCloudGenerator {
      public:
        /**
         * @brief Build a PointCloud2 message from a list of points.
         * @param points Input point list in vehicle/world coordinates.
         * @param out_point_cloud Output PointCloud2 message to populate.
         * @details Fills out fields, sizes, and binary data for XYZ points. Used by
         *          simulation and perception test nodes to publish synthetic clouds.
         */
        static void toPointCloud2(const std::vector<nature::utils::vec3> & points, nature::msg::PointCloud2 & out_point_cloud);
        /**
         * @brief Build a PointCloud2 message with segmentation values.
         * @param points Input point list in vehicle/world coordinates.
         * @param seg_values Per-point segmentation/class IDs.
         * @param out_point_cloud Output PointCloud2 message to populate.
         * @details Adds a "segmentation" field alongside XYZ; used by perception
         *          nodes that consume class-labeled point clouds.
         */
        static void toPointCloud2(const std::vector<nature::utils::vec3> & points, const std::vector<int> & seg_values, nature::msg::PointCloud2 & out_point_cloud);
    };
  }
}

#endif //NATURE_POINT_CLOUD_GENERATOR_H
