/**
 * \class CoordinateConverter
 * Class that converts between latitude/longitude/altitude (LLA), 
 * Universal Transerse Mercator (UTM), Earth-Centered Earth-Fixed, (ECEF), 
 * and East-North-Up (ENU) coordinate systems.
 * Some code modified from from the code posted on 
 * "www.gpsy.com/gpsinfo/geotoutm", 4 Dec 2017 by Chuck Gantz
 * Other code modified from MATLAB file exchange, where noted.
 *
 * \author Chris Goodin
 *
 * \date 12/13/2017
 */
#ifndef NATURE_COORD_CONVERSIONS_H
#define NATURE_COORD_CONVERSIONS_H
#include <math.h>

#include <vector> 
#include <string> 

#include <nature/planning/global/coord_conversions/ellipsoid.h>
#include <nature/planning/global/coord_conversions/matrix.h>

namespace nature{
  
namespace coordinate_system{
 
  /// Latitude-Longitude-Altitude coordinates.
  struct LLA{
    double latitude;
    double longitude;
    double altitude;
  };
  
  /// Earth-Centered, Earth-Fixed coordinates.
  struct ECEF{
    double x;
    double y;
    double z;
  };
  
  /// Local East-North-Up coordinates.
  struct ENU{
    double x;
    double y; 
    double z;
  };

  /// Universal Transverse Mercator coordinates.
  struct UTM{
    double x;
    double y;
    double altitude;
    char zone_char;
    int zone_num;
  };

///CoordinateConverter class. By default the reference ellipsoid is WGS84.
class CoordinateConverter{
 public:

  /**
   * @brief Construct a coordinate converter with WGS84 as default.
   * @details Initializes the reference ellipsoid and local origin transforms.
   */
  CoordinateConverter();

  /**
   * @brief Set the reference ellipsoid by numeric code.
   * @param ellips Reference ellipsoid id (see ellipsoid.h).
   * @details Updates ellipsoid constants and recomputes cached values.
   */
  void SetReferenceEllipsoid(int ellips);

  /**
   * @brief Convert latitude/longitude/altitude to ECEF.
   * @param lla Input LLA coordinates.
   * @return ECEF coordinates.
   * @details Uses the configured reference ellipsoid.
   */
  coordinate_system::ECEF LLA2ECEF(coordinate_system::LLA lla);
  
  /**
   * @brief Convert ECEF to latitude/longitude/altitude.
   * @param ecef Input ECEF coordinates.
   * @return LLA coordinates.
   * @details Uses the configured reference ellipsoid.
   */
  coordinate_system::LLA ECEF2LLA(coordinate_system::ECEF ecef);
  
  /**
   * @brief Convert UTM to latitude/longitude/altitude.
   * @param utm Input UTM coordinates.
   * @return LLA coordinates.
   * @details Uses the configured reference ellipsoid.
   */
  coordinate_system::LLA UTM2LLA(coordinate_system::UTM utm);
 
  /**
   * @brief Convert latitude/longitude/altitude to UTM.
   * @param lla Input LLA coordinates.
   * @return UTM coordinates with zone.
   * @details Uses the configured reference ellipsoid.
   */
  coordinate_system::UTM LLA2UTM(coordinate_system::LLA lla);

  /**
   * @brief Convert UTM to ECEF.
   * @param utm Input UTM coordinates.
   * @return ECEF coordinates.
   * @details Performs UTM->LLA->ECEF conversion.
   */
  coordinate_system::ECEF UTM2ECEF(coordinate_system::UTM utm);
  
  /**
   * @brief Convert ECEF to UTM.
   * @param ecef Input ECEF coordinates.
   * @return UTM coordinates with zone.
   * @details Performs ECEF->LLA->UTM conversion.
   */
  coordinate_system::UTM ECEF2UTM(coordinate_system::ECEF ecef);
  
  /**
   * @brief Convert ENU to ECEF.
   * @param enu Input ENU coordinates.
   * @return ECEF coordinates.
   * @details Uses the current local origin and rotation matrices.
   */
  coordinate_system::ECEF ENU2ECEF(coordinate_system::ENU enu);
  
  /**
   * @brief Convert ECEF to ENU.
   * @param ecef Input ECEF coordinates.
   * @return ENU coordinates.
   * @details Uses the current local origin and rotation matrices.
   */
  coordinate_system::ENU ECEF2ENU(coordinate_system::ECEF ecef);

  /**
   * @brief Convert ENU to LLA.
   * @param enu Input ENU coordinates.
   * @return LLA coordinates.
   * @details Performs ENU->ECEF->LLA conversion.
   */
  coordinate_system::LLA ENU2LLA(coordinate_system::ENU enu);

  /**
   * @brief Convert LLA to ENU.
   * @param lla Input LLA coordinates.
   * @return ENU coordinates.
   * @details Performs LLA->ECEF->ENU conversion relative to local origin.
   */
  coordinate_system::ENU LLA2ENU(coordinate_system::LLA lla);

  /**
   * @brief Convert ENU to UTM.
   * @param enu Input ENU coordinates.
   * @return UTM coordinates.
   * @details Performs ENU->LLA->UTM conversion.
   */
  coordinate_system::UTM ENU2UTM(coordinate_system::ENU enu);

  /**
   * @brief Convert UTM to ENU.
   * @param utm Input UTM coordinates.
   * @return ENU coordinates.
   * @details Performs UTM->LLA->ENU conversion relative to local origin.
   */
  coordinate_system::ENU UTM2ENU(coordinate_system::UTM utm);

  /**
   * @brief Set the local origin in UTM coordinates.
   * @param utm Local origin in UTM.
   * @details Updates internal origin and rotation matrices.
   */
  void SetLocalOrigin(coordinate_system::UTM utm);

  /**
   * @brief Set the local origin in LLA coordinates.
   * @param lla Local origin in LLA.
   * @details Updates internal origin and rotation matrices.
   */
  void SetLocalOrigin(coordinate_system::LLA lla);

  /**
   * @brief Set the local origin from scalar LLA values.
   * @param lat Latitude in degrees.
   * @param lon Longitude in degrees.
   * @param alt Altitude in meters.
   * @details Convenience overload for callers without LLA structs.
   */
  void SetLocalOrigin(double lat, double lon, double alt);

  /**
   * @brief Set the local origin in ECEF coordinates.
   * @param ecef Local origin in ECEF.
   * @details Updates internal origin and rotation matrices.
   */
  void SetLocalOrigin(coordinate_system::ECEF ecef);

 private:
  int ref_ellips_;
  coordinate_system::ECEF local_origin_ecef_;
  coordinate_system::LLA  local_origin_lla_;
  math::Matrix rot_transpose_;
  math::Matrix rot_transpose_inverse_;
  math::Matrix reference_position_;
  Ellipsoid ellipsoid_;
  double a_; 
  double e_; 
  double e2_; 
  double one_minus_e2; 

  /**
   * @brief Determine the UTM zone letter for a latitude.
   * @param lat Latitude in degrees.
   * @return UTM zone letter.
   * @details Used internally when computing UTM outputs.
   */
  char UTMLetterDesignator(double lat);
  /**
   * @brief Compute rotation matrices based on the local origin.
   * @details Precomputes ENU/ECEF transforms for faster conversions.
   */
  void SetMatrices();
};

} //namespace coordinate_system
} //namespace nature

#endif
