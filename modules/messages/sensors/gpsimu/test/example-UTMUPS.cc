// Example of using the GeographicLib::UTMUPS class

#include <GeographicLib/UTMUPS.hpp>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

using namespace GeographicLib;

int main() {
  try {
    // See also example-GeoCoords.cpp
    {
      std::cout << "Sample forward calculation\n";
      double lat = -33.3, lon = 44.4;  // Baghdad
      int zone;
      bool northp;
      double x, y;
      UTMUPS::Forward(lat, lon, zone, northp, x, y);
      std::string zonestr = UTMUPS::EncodeZone(zone, northp);
      std::cout << std::fixed << std::setprecision(2) << zonestr << " " << x
                << " " << y << "\n";
    }
    {
      std::cout << "Sample reverse calculation\n";
      std::string zonestr = "51n";
      int zone;
      bool northp;
      UTMUPS::DecodeZone(zonestr, zone, northp);
      double x = 350999;
      double y = 3433777;
      double lat, lon;
      UTMUPS::Reverse(zone, northp, x, y, lat, lon);
      std::cout << std::fixed << std::setprecision(7) << lat << " " << lon
                << "\n";
    }

    {
      std::cout << "Sample decode zone\n";
      double lat = 31.2303904, lon = 121.4737021;  // Shanghai
      int _zone = UTMUPS::StandardZone(lat, lon);
      std::cout << "zone is: " << _zone << std::endl;
    }

    {
      std::cout << "Sample zone transfer \n";

      double lat_51 = 31.2303904, lon_51 = 125.99999;  // 51n
      double lat_52 = 31.2303904, lon_52 = 126.00011;  // 52n
      int zone_51, zone_52;
      bool northp_51, northp_52;
      double x_51, y_51, x_52, y_52;
      UTMUPS::Forward(lat_51, lon_51, zone_51, northp_51, x_51, y_51);
      UTMUPS::Forward(lat_52, lon_52, zone_52, northp_52, x_52, y_52);

      // transfer one zone to another
      double x_51_to_52, y_51_to_52;
      int zone_51_to_52;
      UTMUPS::Transfer(zone_51, northp_51, x_51, y_51, zone_52, northp_52,
                       x_51_to_52, y_51_to_52, zone_51_to_52);

      std::cout << zone_51 << " " << x_51 << " " << y_51 << std::endl;

      std::cout << zone_51_to_52 << " " << x_51_to_52 << " " << y_51_to_52
                << std::endl;

      std::cout << zone_52 << " " << x_52 << " " << y_52 << std::endl;
    }

    {
      std::cout << "Sample zone transfer \n";

      double lat_n = 00.002000, lon_n = 120;   // 51n
      double lat_s = -00.102000, lon_s = 120;  // 51s
      int zone_n, zone_s;
      bool northp_n, northp_s;
      double x_n, y_n, x_s, y_s;
      UTMUPS::Forward(lat_n, lon_n, zone_n, northp_n, x_n, y_n);
      UTMUPS::Forward(lat_s, lon_s, zone_s, northp_s, x_s, y_s);

      // transfer one zone to another
      double x_n_to_s, y_n_to_s;
      int zone_n_to_s;
      UTMUPS::Transfer(zone_n, northp_n, x_n, y_n, zone_s, northp_s, x_n_to_s,
                       y_n_to_s, zone_n_to_s);

      std::cout << zone_n << " " << x_n << " " << y_n << std::endl;

      std::cout << zone_n_to_s << " " << x_n_to_s << " " << y_n_to_s
                << std::endl;

      std::cout << zone_s << " " << x_s << " " << y_s << std::endl;
    }

  } catch (const std::exception& e) {
    std::cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}