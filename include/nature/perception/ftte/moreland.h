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
#ifndef MORELAND_COLORMAP_H
#define MORELAND_COLORMAP_H


#include "nature/thirdparty/glm/glm.hpp"


namespace traverselib {

/**
 * @brief Map a scalar value to an RGB color using the Moreland colormap.
 * @param x Input value to map.
 * @param xmin Minimum of the input range.
 * @param xmax Maximum of the input range.
 * @return RGB color as glm::vec3.
 * @details Clamps and normalizes x to the range, then computes a perceptually
 *          balanced color. Used for traversability visualization.
 */
glm::vec3 MorelandColormap(float x, float xmin, float xmax);

/**
 * @brief Map a scalar value to a green-tinted Moreland-style color.
 * @param x Input value to map.
 * @param xmin Minimum of the input range.
 * @param xmax Maximum of the input range.
 * @return RGB color as glm::vec3.
 * @details Variant colormap used for vegetation or safe-terrain overlays.
 */
glm::vec3 MorelandGreen(float x, float xmin, float xmax);

} // namespace traverslib

#endif
