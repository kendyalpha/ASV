// Example of using the GeographicLib::UTMUPS class

#include <GeographicLib/UTMUPS.hpp>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    // See also example-GeoCoords.cpp
    {
      // Sample forward calculation
      double lat = -33.3, lon = 44.4;  // Baghdad
      int zone;
      bool northp;
      double x, y;
      UTMUPS::Forward(lat, lon, zone, northp, x, y);
      string zonestr = UTMUPS::EncodeZone(zone, northp);
      cout << fixed << setprecision(2) << zonestr << " " << x << " " << y
           << "\n";
    }
    {
      // Sample reverse calculation
      string zonestr = "38n";
      int zone;
      bool northp;
      UTMUPS::DecodeZone(zonestr, zone, northp);
      double x = 444e3, y = 3688e3;
      double lat, lon;
      UTMUPS::Reverse(zone, northp, x, y, lat, lon);
      cout << lat << " " << lon << "\n";
    }

    {
      double lat = 31.2303904, lon = 121.4737021;  // Shanghai
      int _zone = UTMUPS::StandardZone(lat, lon);
      std::cout << "zone is: " << _zone << std::endl;
    }

    {
      double x = 350926.048;
      double y = 3433826.766;
      // Shanghai Inverse
      string zonestr = "51n";
      int zone;
      bool northp;
      UTMUPS::DecodeZone(zonestr, zone, northp);
      double lat, lon;
      UTMUPS::Reverse(zone, northp, x, y, lat, lon);
      cout << fixed << setprecision(7) << lat << " " << lon << "\n";
    }
  } catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
