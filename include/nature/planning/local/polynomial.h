/**
 * \class Polynomial
 *
 * Polynomial class for defining polynomial with coefficients.
 *
 * \author Chris Goodin
 *
 * \date 8/31/2020
 */
#ifndef SPLINE_POLYNOMIAL_H
#define SPLINE_POLYNOMIAL_H
#include <vector>
#include <algorithm>

namespace nature {
namespace planning {
class Polynomial {
public:

	/**
	 * @brief Construct an empty polynomial.
	 * @details Leaves coefficient list empty; used as a placeholder before
	 *          initialization by the local planner.
	 */ 
	Polynomial() {}

	/**
	 * @brief Construct a polynomial from coefficients.
	 * @param coeffs Coefficients ordered highest degree to constant term.
	 * @details Reverses the input for internal low-to-high degree storage.
	 *          Represents p(x) = c0*x^n + ... + cn.
	 */
	Polynomial(std::vector<float> coeffs) {
		coeffs_ = coeffs;
		std::reverse(coeffs_.begin(), coeffs_.end());
	}

	/**
	 * @brief Compute the derivative polynomial.
	 * @return Polynomial representing the first derivative.
	 * @details Generates new coefficients by multiplying by the power index.
	 *          Used by candidate path curvature computations.
	 */
	Polynomial Derivative() {
		std::vector<float> coeffs;
		for (int i = 0; i < coeffs_.size(); i++) {
			float c = i * coeffs_[i];
			coeffs.push_back(c);
		}
		Polynomial poly(coeffs);
		return poly;
	}

	/**
	 * @brief Evaluate the polynomial at x.
	 * @param x Input value.
	 * @return Polynomial value at x.
	 * @details Uses power expansion with stored coefficients.
	 */ 
	float At(float x) {
		float y = 0.0f;
		for (int i = 0; i < coeffs_.size(); i++) {

			y += coeffs_[i] * (float)pow(x, i);
		}
		return y;
	}

private:
	std::vector<float> coeffs_;

};

} // namespace planning
} // namespace nature


#endif
