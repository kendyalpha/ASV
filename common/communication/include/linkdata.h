/*
***********************************************************************
* linkdata.h:
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _LINKDATA_H_
#define _LINKDATA_H_

namespace ASV::common {

enum class LINKSTATUS {
  DISCONNECTED = 0,  //
  CONNECTING = 1,
  CONNECTED
};

enum class LINKPROTOCOL {
  SERIAL_RS232 = 0,  //
  TCPCLIENT,
  TCPSERVER

};

}  // namespace ASV::common

#endif /* _LINKDATA_H_ */