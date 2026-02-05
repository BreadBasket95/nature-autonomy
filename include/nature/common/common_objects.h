//
// Created by Stefan on 2021-07-28.
//

#ifndef NATURE_COMMON_OBJECTS_H
#define NATURE_COMMON_OBJECTS_H

#include "nature/nature_utils.h"

namespace nature {
    namespace common {

        struct vec4_d {
            /**
             * @brief Default-constructs a 4D vector with uninitialized components.
             * @details This constructor leaves the data members untouched; callers are expected to
             *          assign x/y/z/w explicitly before use. The type is used as a lightweight math
             *          container throughout perception and planning.
             */
            vec4_d()= default;
            /**
             * @brief Construct a 4D vector with explicit components.
             * @param x X component to store.
             * @param y Y component to store.
             * @param z Z component to store.
             * @param w W component to store.
             * @details Stores the values directly without normalization or validation. Used to
             *          represent orientations and homogeneous coordinates in the stack.
             */
            vec4_d(double x, double y, double z, double w): x(x), y(y), z(z), w(w){ }
            double x;
            double y;
            double z;
            double w;
        };

        struct vec3_d {
            /**
             * @brief Default-constructs a 3D vector with uninitialized components.
             * @details The members are left as-is; callers must assign values before use. This
             *          struct provides a simple container used across navigation primitives.
             */
            vec3_d()= default;
            /**
             * @brief Construct a 3D vector with explicit components.
             * @param x X component to store.
             * @param y Y component to store.
             * @param z Z component to store.
             * @details Stores the values directly; no normalization or bounds checks are applied.
             */
            vec3_d(double x, double y, double z): x(x), y(y), z(z){ }
            double x;
            double y;
            double z;
        };

        struct Header{
            std::string frame_id;
            double seconds;
            int32_t sec;
            int32_t nanosec;
        };

        struct GridOrigin {
            vec3_d position;
            vec4_d orientation;
        };

        struct GridInfo{
            GridOrigin origin;
            float resolution;
            uint32_t width;
            uint32_t height;
        };

        struct OccupancyGrid{
            Header header;
            GridInfo info;
        };


        struct Pose{
            vec3_d position;
            vec4_d orientation;
        };

        struct PoseStamped{
            Pose pose;
            Header header;
        };

        struct Odometry {
            Pose pose;
        };

        struct Path {
            std::vector<PoseStamped> poses;
        };

        struct PointCloud{
            std::vector<utils::vec3> points;
            /**
             * @brief Return the number of points in the cloud.
             * @return Point count as a size_t.
             * @details Reads the size of the underlying vector; no filtering is applied. Used by
             *          perception and mapping stages to size loops and buffers.
             */
            size_t size() const{
                return points.size();
            }
        };

        struct Twist{
            vec3_d linear;
            vec3_d angular;
        };
    }
}

#endif //NATURE_COMMON_OBJECTS_H
