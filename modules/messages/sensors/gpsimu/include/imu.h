/**************************************************
 * imu.h
 * function to receive data from IMU, using XDA library.
 * the header file can be read by C++ compilers
 *
 * by ZH.Hu (CrossOcean.ai)
 *************************************************
 */

#ifndef _IMU_H_
#define _IMU_H_

#include <xscommon/xsens_mutex.h>
#include <xsensdeviceapi.h>
#include <xstypes/xstime.h>

#include "common/logging/include/easylogging++.h"
#include "gpsdata.h"

#include <iomanip>
#include <iostream>
#include <list>

namespace ASV::messages {

class CallbackHandler : public XsCallback {
 public:
  CallbackHandler(std::size_t maxBufferSize = 5)
      : m_maxNumberOfPacketsInBuffer(maxBufferSize),
        m_numberOfPacketsInBuffer(0) {}

  virtual ~CallbackHandler() throw() {}

  bool packetAvailable() const {
    xsens::Lock locky(&m_mutex);
    return m_numberOfPacketsInBuffer > 0;
  }

  XsDataPacket getNextPacket() {
    assert(packetAvailable());
    xsens::Lock locky(&m_mutex);
    XsDataPacket oldestPacket(m_packetBuffer.front());
    m_packetBuffer.pop_front();
    --m_numberOfPacketsInBuffer;
    return oldestPacket;
  }

 protected:
  virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) {
    xsens::Lock locky(&m_mutex);
    assert(packet != nullptr);
    while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
      (void)getNextPacket();

    m_packetBuffer.push_back(*packet);
    ++m_numberOfPacketsInBuffer;
    assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
  }

 private:
  mutable xsens::Mutex m_mutex;

  std::size_t m_maxNumberOfPacketsInBuffer;
  std::size_t m_numberOfPacketsInBuffer;
  std::list<XsDataPacket> m_packetBuffer;
};  // end class CallbackHandler

class imu {
 public:
  imu(uint16_t m_frequency = 100)
      : imu_frequency(m_frequency),
        imu_RTdata({
            0,  // acc_X
            0,  // acc_Y
            0,  // acc_Z
            0,  // roll
            0,  // pitch
            0,  // yaw
            0   // roti
        }) {}
  virtual ~imu() = default;

  int initializeIMU() {
    control = XsControl::construct();
    assert(control != nullptr);

    XsPortInfoArray portInfoArray = XsScanner::scanPorts();
    // Find an MTi device
    for (auto const& portInfo : portInfoArray) {
      if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig()) {
        mtPort = portInfo;
        break;
      }
    }

    if (mtPort.empty()) {
      CLOG(ERROR, "IMU") << "No MTi device found!";
      return -1;
    }

    if (!control->openPort(mtPort.portName().toStdString(),
                           mtPort.baudrate())) {
      CLOG(ERROR, "IMU") << "Could not open port!";
      return -1;
    }

    // Get the device object
    XsDevice* device = control->device(mtPort.deviceId());
    assert(device != nullptr);

    // Create and attach callback handler to device
    device->addCallbackHandler(&callback);

    // Put the device into configuration mode before configuring the device
    if (!device->gotoConfig()) {
      CLOG(ERROR, "IMU") << "Could not put device into configuration mode!";
      return -1;
    }

    XsOutputConfigurationArray configArray;

    configArray.push_back(
        XsOutputConfiguration(XDI_SubFormatDouble, imu_frequency));
    configArray.push_back(
        XsOutputConfiguration(XDI_SampleTimeFine, imu_frequency));
    configArray.push_back(
        XsOutputConfiguration(XDI_EulerAngles, imu_frequency));
    configArray.push_back(
        XsOutputConfiguration(XDI_Acceleration, imu_frequency));
    configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, imu_frequency));
    configArray.push_back(XsOutputConfiguration(XDI_StatusWord, imu_frequency));

    if (!device->setOutputConfiguration(configArray)) {
      CLOG(ERROR, "IMU") << "Could not configure MTi device!";
      return -1;
    }

    // Putting device into measurement mode...
    if (!device->gotoMeasurement()) {
      CLOG(ERROR, "IMU") << "Could not put device into measurement mode!";
      return -1;
    }

    CLOG(INFO, "IMU") << "Initialization Successful!";
    return 0;
  }  // initializeIMU

  imu& receivedata() {
    if (callback.packetAvailable()) {
      std::cout << std::setw(5) << std::fixed << std::setprecision(2);

      // Retrieve a packet
      XsDataPacket packet = callback.getNextPacket();

      if (packet.containsCalibratedData()) {
        XsVector acc = packet.calibratedAcceleration();
        imu_RTdata.acc_X = acc[0];
        imu_RTdata.acc_Y = acc[1];
        imu_RTdata.acc_Z = acc[2];

        // XsVector gyr = packet.calibratedGyroscopeData();
        // std::cout << " |Gyr X:" << gyr[0] << ", Gyr Y:" << gyr[1]
        //           << ", Gyr Z:" << gyr[2];

        // XsVector mag = packet.calibratedMagneticField();
        // std::cout << " |Mag X:" << mag[0] << ", Mag Y:" << mag[1]
        //           << ", Mag Z:" << mag[2];
      }

      if (packet.containsOrientation()) {
        XsEuler euler = packet.orientationEuler();
        imu_RTdata.roll = euler.roll();
        imu_RTdata.pitch = euler.pitch();
        imu_RTdata.yaw = euler.yaw();
      }
    }
    return *this;

  }  // receivedata

  imuRTdata getimuRTdata() const noexcept { return imu_RTdata; }

  void releaseIMU() {
    control->closePort(mtPort.portName().toStdString());
    control->destruct();
  }

 private:
  uint16_t imu_frequency;
  imuRTdata imu_RTdata;
  XsControl* control;
  CallbackHandler callback;
  XsPortInfo mtPort;

};  // end class imu

}  // namespace ASV::messages

#endif /* _IMU_H_ */