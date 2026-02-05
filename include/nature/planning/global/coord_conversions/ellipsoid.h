/**
 * \class Ellipsoid
 * Class that defines parameters of the reference ellipsoid being used.
 * Modified from from the code posted on 
 * "www.gpsy.com/gpsinfo/geotoutm", 4 Dec 2017 by Chuck Gantz
 *
 * \author Chris Goodin
 *
 * \date 12/13/2017
 */

#ifndef AVT_341_ELLIPSOID_H
#define AVT_341_ELLIPSOID_H

#include <vector>
#include <string>

namespace nature{
namespace coordinate_system{

/**
 * The Ellipsoid class defines 23 reference ellipsoids, with
 * corresponding reference numbers.
 * 1, "Airy"
 * 2, "Australian National"
 * 3, "Bessel 1841"
 * 4, "Bessel 1841 (Nambia)"
 * 5, "Clarke 1866"
 * 6, "Clarke 1880"
 * 7, "Everest"
 * 8, "Fischer 1960 (Mercury)"
 * 9, "Fischer 1968"
 * 10, "GRS 1967"
 * 11, "GRS 1980"
 * 12, "Helmert 1906"
 * 13, "Hough"
 * 14, "International"
 * 15, "Krassovsky"
 * 16, "Modified Airy"
 * 17, "Modified Everest"
 * 18, "Modified Fischer 1960"
 * 19, "South American 1969"
 * 20, "WGS 60"
 * 21, "WGS 66"
 * 22, "WGS-72"
 * 23 "WGS-84"
 */
class Ellipsoid{
 public:
  /**
   * @brief Construct an ellipsoid without initializing constants.
   * @details Leaves parameters unset until Init or the parameterized constructor
   *          is used. This mirrors legacy initialization paths.
   */
  Ellipsoid();

  /**
   * @brief Construct an ellipsoid with explicit constants.
   * @param id Unique id number of the ellipsoid.
   * @param name Reference name of the ellipsoid.
   * @param radius Equatorial radius of the reference, meters.
   * @param ecc Squared eccentricity of the ellipsoid.
   * @details Stores constants for later geodetic conversions.
   */
  Ellipsoid(int id, std::string name, double radius, double ecc);

  /**
   * @brief Get the equatorial radius of this ellipsoid.
   * @return Equatorial radius in meters.
   * @details Used in latitude/longitude to UTM conversions.
   */
  double GetEquatorialRadius(){return equatorial_radius_;}

  /**
   * @brief Get the squared eccentricity of this ellipsoid.
   * @return Squared eccentricity.
   * @details Used in geodetic calculations.
   */
  double GetEccentricitySquared(){return eccentricity_squared_;}

  /**
   * @brief Get the equatorial radius for a reference ellipsoid by id.
   * @param refnum Reference ellipsoid id (1-23).
   * @return Equatorial radius in meters.
   * @details Accesses built-in constants table.
   */
  double GetEquatorialRadius(int refnum );

  /**
   * @brief Get the squared eccentricity for a reference ellipsoid by id.
   * @param refnum Reference ellipsoid id (1-23).
   * @return Squared eccentricity.
   * @details Accesses built-in constants table.
   */
  double GetEccentrictySquared(int refnum);
 
 private: 
  /**
   * @brief Initialize the internal list of reference ellipsoids.
   * @details Populates constants for the 23 standard ellipsoids.
   */
  void Init(); 
  int id_;
  std::string ellipsoid_name_;
  double equatorial_radius_; 
  double eccentricity_squared_;  
};

} //namespace coordinate_system
} //namespace nature

#endif
