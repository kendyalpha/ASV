/*
***********************************************************************
* windcompensation.h: compute the wind load on USV using wind
* coefficient
* This header file can be read by C++ compilers
*
* by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef _WINDCOMPENSATION_H_
#define _WINDCOMPENSATION_H_

#include "estimatordata.h"
#include "lowpass.h"

namespace ASV::localization {

class windcompensation {
 public:
  windcompensation(const sealoadRTdata &_sealoadRTdata)
      : air_rho(1.184),
        cx(0.7),
        cy(0.825),
        KG(0.4),
        H(0.8),
        AFW(0.96),
        ALW(2.16),
        SealoadRTdata(_sealoadRTdata) {}

  ~windcompensation() {}

  windcompensation &computewindload(double raw_windspeed,
                                    double raw_windorientation) {
    double a_windorientation = orientation_lowpass.movingaverage(
        convertwinddirection(raw_windorientation));
    double a_windspeed = speed_lowpass.movingaverage(raw_windspeed);

    if (SealoadRTdata.windstatus == WINDCOMPENSATION::WINDON)
      computewindforce(a_windspeed, a_windorientation,
                       SealoadRTdata.windload(0), SealoadRTdata.windload(1),
                       SealoadRTdata.windload(2));
    else
      SealoadRTdata.windload.setZero();
    return *this;
  }

  // setvalue after the initialization
  void setvalue(double raw_windspeed, double raw_windorientation) {
    // low pass
    speed_lowpass.setaveragevector(raw_windspeed);
    orientation_lowpass.setaveragevector(
        convertwinddirection(raw_windorientation));
  }

  void setwindstatus(int _windstatus) {
    switch (_windstatus) {
      case 0:
        SealoadRTdata.windstatus = WINDCOMPENSATION::WINDOFF;
        break;
      case 1:
        SealoadRTdata.windstatus = WINDCOMPENSATION::WINDON;
        break;
      default:
        break;
    }
  }
  void setwindstatus(WINDCOMPENSATION _windstatus) {
    SealoadRTdata.windstatus = _windstatus;
  }

  auto getsealoadRTdata() const noexcept { return SealoadRTdata; }

 private:
  const double air_rho;  // air density
  const double cx;       // 系数0.5~0.9
  const double cy;       // 系数0.7~0.95
  const double KG;       // 重心高
  const double H;        // 风速仪高度
  const double AFW;      // 船舶横向投影面积
  const double ALW;      // 船舶纵向投影面积
  // Fx, Fy, Mz (wind force) in the body coordinate
  sealoadRTdata SealoadRTdata;
  // low pass filter for speed and orientation
  lowpass<1> speed_lowpass;
  lowpass<1> orientation_lowpass;
  double convertwinddirection(double wind_orientation) const noexcept {
    return wind_orientation - M_PI;
  }  // convertwinddirection
  // compute wind load on vessel using relative wind speed and orientation
  void computewindforce(double wind_speed, double wind_orientation, double &Fx,
                        double &Fy, double &Mz) {
    double Cx = cx * std::cos(wind_orientation);  // x方向的风力系数
    double Cy = cy * std::sin(wind_orientation);  // y方向的风力系数
    double v_h =
        std::pow((KG / H), 0.143) * wind_speed;  //转化后船实际的相对风速
    double temp_force = 0.5 * air_rho * std::pow(v_h, 2);
    Fx = temp_force * Cx * AFW;  // Fx
    Fy = temp_force * Cy * ALW;  // Fy
    Mz = 0;

  }  // computewindload
}; // end windcompensation
}  // end namespace ASV


#endif /*_WINDCOMPENSATION_H_*/

