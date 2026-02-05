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
#ifndef PLANE_H
#define PLANE_H
// c++ includes
#include <vector>
#include "nature/perception/ftte/matrix.h"
#include "nature/thirdparty/glm/glm.hpp"

namespace traverselib {
	// equation of plane: ax + by + c = z

class Plane {
public:
	/**
	 * @brief Construct a plane with zero coefficients.
	 * @details Initializes a_, b_, c_ to 0 so the plane represents z=0.
	 *          Used by traversability estimation when seeding fits.
	 */
	Plane() {
		a_ = 0.0f;
		b_ = 0.0f;
		c_ = 0.0f;
	}

	/**
	 * @brief Copy-construct a plane.
	 * @param p Plane to copy.
	 * @details Copies plane coefficients directly; used when duplicating fits.
	 */
	Plane(const Plane &p) {
		a_ = p.a_;
		b_ = p.b_;
		c_ = p.c_;
	}

	/**
	 * @brief Set plane coefficients for z = a*x + b*y + c.
	 * @param a Plane coefficient for x.
	 * @param b Plane coefficient for y.
	 * @param c Plane offset.
	 * @details Directly assigns the coefficients without validation.
	 */
	void SetCoeffs(float a, float b, float c) {
		a_ = a;
		b_ = b;
		c_ = c;
	}

	/**
	 * @brief Fit a plane to a set of 3D points.
	 * @param points Input points (x,y,z) to fit.
	 * @details Uses least squares to solve for plane coefficients. For
	 *          degenerate cases with <=3 points, it defaults to a flat plane
	 *          at the mean z height. This is used by the FTTE voxel analysis.
	 */
	void FitToPoints(std::vector<glm::vec3> points) {
		int np = (int)points.size();
		if (np <= 0) {
			return;
		}
		if (np <= 3) {
			c_ = 0.0f;
			for (int i = 0; i < np; i++) {
				c_ = c_ + points[i].z;
			}
			c_ = c_ / (float)np;
			a_ = 0.0f;
			b_ = 0.0f;
			return;
		}
		Matrix A((int)points.size(), 3);
		Matrix B((int)points.size(), 1);
		for (int i = 0; i < np; i++) {
			A(i, 0) = points[i].x;
			A(i, 1) = points[i].y;
			A(i, 2) = 1.0f;
			B(i, 0) = points[i].z;
		}
		Matrix AT = A.Transpose();
		Matrix ATA = AT*A;
		Matrix ATAI = ATA.Inverse();
		Matrix x = ATAI*AT*B;
		a_ = (float)x(0,0);
		b_ = (float)x(1,0);
		c_ = (float)x(2,0);
	}

	/**
	 * @brief Compute the plane height at a given (x,y).
	 * @param x X coordinate.
	 * @param y Y coordinate.
	 * @return Z height on the plane.
	 * @details Evaluates z = a*x + b*y + c.
	 */
	float GetHeightAt(float x, float y) {
		float z = a_ * x + b_ * y + c_;
		return z;
	}

	/**
	 * @brief Get the plane coefficients as a vector.
	 * @return glm::vec3 containing (a,b,c).
	 * @details Useful for passing plane parameters to other algorithms.
	 */
	glm::vec3 GetCoeffs() {
		glm::vec3 coeffs(a_, b_, c_);
		return coeffs;
	}

	/**
	 * @brief Compute the plane slope magnitude.
	 * @return Slope magnitude sqrt(a^2 + b^2).
	 * @details The slope represents terrain steepness used in traversability.
	 */
	float GetSlope() {
		// magnitude of the gradient
		float s = sqrt(a_*a_ + b_*b_);
		return s;
	}

private:

	float a_;
	float b_;
	float c_;

};

} // namespace traverselib

#endif
