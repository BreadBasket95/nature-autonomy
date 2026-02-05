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
#ifndef VEHICLE_H_
#define VEHICLE_H_

namespace traverselib {

class Vehicle {
public:
	/**
	 * @brief Construct a vehicle model with default parameters.
	 * @details Initializes physical parameters used by FTTE traversability scoring.
	 */
	Vehicle();

	/**
	 * @brief Compute traversability score from terrain metrics.
	 * @param rms RMS roughness of the terrain.
	 * @param slope Estimated slope magnitude.
	 * @param veg_dens Vegetation density metric.
	 * @param rci Rolling resistance or soil metric (RCI/VCI proxy).
	 * @return Traversability score, higher is better.
	 * @details Combines slope, roughness, soil, and vegetation terms using vehicle
	 *          parameters; used by voxel grid classification.
	 */
	float GetTraversability(float rms, float slope, float veg_dens, float rci);

	/**
	 * @brief Get the soil deformation coefficient (beta).
	 * @return Beta coefficient used in soil/RCI calculations.
	 * @details Set during parameter configuration.
	 */
	float GetBeta() { return beta_; }

	/**
	 * @brief Get the soil deformation exponent (eta).
	 * @return Eta coefficient used in soil/RCI calculations.
	 * @details Set during parameter configuration.
	 */
	float GetEta() { return eta_; }

	/**
	 * @brief Compute gamma parameter from roughness.
	 * @param roughness RMS roughness value.
	 * @return Gamma value used in roughness penalty calculations.
	 * @details Used internally by GetTraversability to scale roughness effects.
	 */
	float GetGamma(float roughness);

	/**
	 * @brief Get the critical obstacle diameter.
	 * @return Critical diameter in meters.
	 * @details Derived from vehicle geometry and used to threshold obstacles.
	 */
	float GetCriticalDiameter() { return critical_diameter_; }

	/**
	 * @brief Get the bumper height.
	 * @return Bumper height in meters.
	 * @details Used to determine clearance against terrain features.
	 */
	float GetBumperHeight() { return bumper_height_; }

	/**
	 * @brief Get the roof height.
	 * @return Roof height in meters.
	 * @details Used for overhead clearance checks.
	 */
	float GetRoofHeight() { return roof_height_; }

	/**
	 * @brief Set physical vehicle parameters.
	 * @param mass Vehicle mass.
	 * @param bumper_height Bumper height in meters.
	 * @param tire_radius Tire radius in meters.
	 * @param vci1 Soil strength metric.
	 * @param max_slope Maximum climbable slope.
	 * @param roof_height Roof height in meters.
	 * @details Updates dependent coefficients used by traversability scoring.
	 */
	void SetParams(float mass, float bumper_height, float tire_radius, float vci1, float max_slope, float roof_height);

	/**
	 * @brief Set the grid resolution used for terrain calculations.
	 * @param h Resolution in meters.
	 * @details Updates cached values derived from resolution for performance.
	 */
	void SetGridResolution(float h);

	/**
	 * @brief Set the slope coefficient for traversability scoring.
	 * @param sc Slope coefficient.
	 * @details Adjusts sensitivity of slope term.
	 */
	void SetSlopeCoeff(float sc){ slope_coeff_ = sc; }

	/**
	 * @brief Set the slope exponent for traversability scoring.
	 * @param se Slope exponent.
	 * @details Adjusts nonlinearity of slope term.
	 */
	void SetSlopeExponent(float se){ slope_exp_ = se; }

	/**
	 * @brief Set the soil coefficient for traversability scoring.
	 * @param sc Soil coefficient.
	 * @details Adjusts sensitivity of soil term.
	 */
	void SetSoilCoeff(float sc){ soil_coeff_ = sc; }

	/**
	 * @brief Set the soil exponent for traversability scoring.
	 * @param se Soil exponent.
	 * @details Adjusts nonlinearity of soil term.
	 */
	void SetSoilExponent(float se){ soil_exp_ = se; }

	/**
	 * @brief Set the roughness coefficient for traversability scoring.
	 * @param rc Roughness coefficient.
	 * @details Adjusts sensitivity of roughness term.
	 */
	void SetRoughnessCoeff(float rc){ rough_coeff_ = rc; }

	/**
	 * @brief Set the roughness exponent for traversability scoring.
	 * @param re Roughness exponent.
	 * @details Adjusts nonlinearity of roughness term.
	 */
	void SetRoughnessExponent(float re){ rough_exp_ = re; }

	/**
	 * @brief Set the vegetation coefficient for traversability scoring.
	 * @param vc Vegetation coefficient.
	 * @details Adjusts sensitivity of vegetation term.
	 */
	void SetVegCoeff(float vc){ veg_coeff_ = vc; }

	/**
	 * @brief Set the vegetation exponent for traversability scoring.
	 * @param ve Vegetation exponent.
	 * @details Adjusts nonlinearity of vegetation term.
	 */
	void SetVegExponent(float ve){ veg_exp_ = ve; }

private:
	float res_;
	float res_squared_;
	float rv_;
	float mass_;
	float tire_radius_;
	float bumper_height_;
	float vci1_;
	float max_slope_;
	float roof_height_;
	float beta_;
	float eta_;
	float sigma_max_;
	float critical_diameter_;
	float slope_coeff_, slope_exp_;
	float soil_coeff_, soil_exp_;
	float veg_coeff_, veg_exp_;
	float rough_coeff_, rough_exp_;
};

} // namespace traverselib
#endif
