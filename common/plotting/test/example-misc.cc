/*
Copyright (c) 2013 Daniel Stahlke

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <map>
#include <vector>

// Warn about use of deprecated functions.
#define GNUPLOT_DEPRECATE_WARN
#include "gnuplot-iostream.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// http://stackoverflow.com/a/1658429
#ifdef _WIN32
#include <windows.h>
inline void mysleep(unsigned millis) { ::Sleep(millis); }
#else
#include <unistd.h>
inline void mysleep(unsigned millis) { ::usleep(millis * 1000); }
#endif

void pause_if_needed() {
#ifdef _WIN32
  // For Windows, prompt for a keystroke before the Gnuplot object goes out of
  // scope so that the gnuplot window doesn't get closed.
  std::cout << "Press enter to exit." << std::endl;
  std::cin.get();
#endif
}

// Tell MSVC to not warn about using fopen.
// http://stackoverflow.com/a/4805353/1048959
#if defined(_MSC_VER) && _MSC_VER >= 1400
#pragma warning(disable : 4996)
#endif

void demo_basic() {
  Gnuplot gp;
  // For debugging or manual editing of commands:
  // Gnuplot gp(std::fopen("plot.gnu"));
  // or
  // Gnuplot gp("tee plot.gnu | gnuplot -persist");

  std::vector<std::pair<double, double>> xy_pts_A;
  for (double x = -2; x < 2; x += 0.01) {
    double y = x * x * x;
    xy_pts_A.push_back(std::make_pair(x, y));
  }

  std::vector<std::pair<double, double>> xy_pts_B;
  for (double alpha = 0; alpha < 1; alpha += 1.0 / 24.0) {
    double theta = alpha * 2.0 * 3.14159;
    xy_pts_B.push_back(std::make_pair(cos(theta), sin(theta)));
  }

  gp << "set xrange [-2:2]\nset yrange [-2:2]\n";
  gp << "plot '-' with lines title 'cubic', '-' with points title 'circle'\n";
  gp.send1d(xy_pts_A);
  gp.send1d(xy_pts_B);

  pause_if_needed();
}

void demo_binary() {
  Gnuplot gp;

  std::vector<std::pair<double, double>> xy_pts_A;
  for (double x = -2; x < 2; x += 0.01) {
    double y = x * x * x;
    xy_pts_A.push_back(std::make_pair(x, y));
  }

  std::vector<std::pair<double, double>> xy_pts_B;
  for (double alpha = 0; alpha < 1; alpha += 1.0 / 24.0) {
    double theta = alpha * 2.0 * 3.14159;
    xy_pts_B.push_back(std::make_pair(cos(theta), sin(theta)));
  }

  gp << "set xrange [-2:2]\nset yrange [-2:2]\n";
  gp << "plot '-' binary" << gp.binFmt1d(xy_pts_A, "record")
     << "with lines title 'cubic',"
     << "'-' binary" << gp.binFmt1d(xy_pts_B, "record")
     << "with points title 'circle'\n";
  gp.sendBinary1d(xy_pts_A);
  gp.sendBinary1d(xy_pts_B);

  pause_if_needed();
}

void demo_2Dpolygon() {
  Gnuplot gp;

  double position_x = 1;
  double position_y = -2;
  double heading = -M_PI / 2;

  std::vector<double> vessel_profile_x({2.0, 1.1, -1.0, -1.0, 1.1});
  std::vector<double> vessel_profile_y({0.0, 0.8, 0.8, -0.8, -0.8});
  double cvalue = std::cos(heading);
  double svalue = std::sin(heading);

  gp << "reset\n";
  gp << "set xrange [-4:4]\n";
  gp << "set yrange [-4:4]\n";
  gp << "set size ratio -1\n";
  gp << "set object 1 polygon from";
  for (std::size_t i = 0; i != vessel_profile_x.size(); ++i) {
    double x = position_x + cvalue * vessel_profile_x[i] -
               svalue * vessel_profile_y[i];
    double y = position_y + svalue * vessel_profile_x[i] +
               cvalue * vessel_profile_y[i];

    gp << " " << y << ", " << x << " to";
  }
  gp << " "
     << position_y + svalue * vessel_profile_x[0] + cvalue * vessel_profile_y[0]
     << ", "
     << position_x + cvalue * vessel_profile_x[0] - svalue * vessel_profile_y[0]
     << "\n";
  gp << "set object 1 fc rgb 'blue' fillstyle solid 0.25 noborder\n";
  gp << "set xlabel 'Y'\n";
  gp << "set ylabel 'X'\n";

  std::vector<std::pair<double, double>> xy_pts_A;
  xy_pts_A.push_back(std::make_pair(position_y, position_x));
  gp << "plot " << gp.file1d(xy_pts_A) << " with points title 'CoG'\n";

  pause_if_needed();
}

void demo_tmpfile() {
  Gnuplot gp;

  std::vector<std::pair<double, double>> xy_pts_A;
  for (double x = -2; x < 2; x += 0.01) {
    double y = x * x * x;
    xy_pts_A.push_back(std::make_pair(x, y));
  }

  std::vector<std::pair<double, double>> xy_pts_B;
  for (double alpha = 0; alpha < 1; alpha += 1.0 / 24.0) {
    double theta = alpha * 2.0 * 3.14159;
    xy_pts_B.push_back(std::make_pair(cos(theta), sin(theta)));
  }

  gp << "set xrange [-2:2]\nset yrange [-2:2]\n";
  // Data will be sent via a temporary file.  These are erased when you call
  // gp.clearTmpfiles() or when gp goes out of scope.  If you pass a filename
  // (i.e. `gp.file1d(pts, "mydata.dat")`), then the named file will be created
  // and won't be deleted.
  //
  // Note: you need std::endl here in order to flush the buffer.  The send1d()
  // function flushes automatically, but we're not using that here.
  gp << "plot" << gp.file1d(xy_pts_A) << "with lines title 'cubic',"
     << gp.file1d(xy_pts_B) << "with points title 'circle'" << std::endl;

  pause_if_needed();
}

void demo_png() {
  Gnuplot gp;

  gp << "set terminal png\n";

  std::vector<double> y_pts;
  for (int i = 0; i < 1000; i++) {
    double y = (i / 500.0 - 1) * (i / 500.0 - 1);
    y_pts.push_back(y);
  }

  std::cout << "Creating my_graph_1.png" << std::endl;
  gp << "set output 'my_graph_1.png'\n";
  gp << "plot '-' with lines, sin(x/200) with lines\n";
  gp.send1d(y_pts);

  std::vector<std::pair<double, double>> xy_pts_A;
  for (double x = -2; x < 2; x += 0.01) {
    double y = x * x * x;
    xy_pts_A.push_back(std::make_pair(x, y));
  }

  std::vector<std::pair<double, double>> xy_pts_B;
  for (double alpha = 0; alpha < 1; alpha += 1.0 / 24.0) {
    double theta = alpha * 2.0 * 3.14159;
    xy_pts_B.push_back(std::make_pair(cos(theta), sin(theta)));
  }

  std::cout << "Creating my_graph_2.png" << std::endl;
  gp << "set output 'my_graph_2.png'\n";
  gp << "set xrange [-2:2]\nset yrange [-2:2]\n";
  gp << "plot '-' with lines title 'cubic', '-' with points title 'circle'\n";
  gp.send1d(xy_pts_A);
  gp.send1d(xy_pts_B);
}

void demo_vectors() {
  Gnuplot gp;

  std::vector<boost::tuple<double, double, double, double>> vecs;
  for (double alpha = 0; alpha < 1; alpha += 1.0 / 24.0) {
    double theta = alpha * 2.0 * 3.14159;
    vecs.push_back(boost::make_tuple(cos(theta), sin(theta), -cos(theta) * 0.1,
                                     -sin(theta) * 0.1));
  }

  gp << "set xrange [-2:2]\nset yrange [-2:2]\n";
  gp << "plot '-' with vectors title 'circle'\n";
  gp.send1d(vecs);

  pause_if_needed();
}

std::vector<boost::tuple<double, double, double>> get_trefoil() {
  std::vector<boost::tuple<double, double, double>> vecs;
  for (double alpha = 0; alpha < 1; alpha += 1.0 / 120.0) {
    double theta = alpha * 2.0 * 3.14159;
    vecs.push_back(boost::make_tuple((2 + cos(3 * theta)) * cos(2 * theta),
                                     (2 + cos(3 * theta)) * sin(2 * theta),
                                     sin(3 * theta)));
  }
  return vecs;
}

void demo_inline_text() {
  std::cout << "Creating inline_text.gnu" << std::endl;
  // This file handle will be closed automatically when gp goes out of scope.
  Gnuplot gp(std::fopen("inline_text.gnu", "w"));

  std::vector<boost::tuple<double, double, double>> vecs = get_trefoil();

  gp << "splot '-' with lines notitle\n";
  gp.send1d(vecs);

  std::cout << "Now run 'gnuplot -persist inline_text.gnu'.\n";
}

void demo_inline_binary() {
  std::cout << "Creating inline_binary.gnu" << std::endl;
  // This file handle will be closed automatically when gp goes out of scope.
  Gnuplot gp(std::fopen("inline_binary.gnu", "wb"));

  std::vector<boost::tuple<double, double, double>> vecs = get_trefoil();

  gp << "splot '-' binary" << gp.binFmt1d(vecs, "record")
     << "with lines notitle\n";
  gp.sendBinary1d(vecs);

  std::cout << "Now run 'gnuplot -persist inline_binary.gnu'.\n";
}

void demo_external_text() {
  std::cout << "Creating external_text.gnu" << std::endl;
  // This file handle will be closed automatically when gp goes out of scope.
  Gnuplot gp(std::fopen("external_text.gnu", "w"));

  std::vector<boost::tuple<double, double, double>> vecs = get_trefoil();

  std::cout << "Creating external_text.dat" << std::endl;
  gp << "splot" << gp.file1d(vecs, "external_text.dat")
     << "with lines notitle\n";

  std::cout << "Now run 'gnuplot -persist external_text.gnu'.\n";
}

void demo_external_binary() {
  std::cout << "Creating external_binary.gnu" << std::endl;
  // This file handle will be closed automatically when gp goes out of scope.
  Gnuplot gp(std::fopen("external_binary.gnu", "w"));

  std::vector<boost::tuple<double, double, double>> vecs = get_trefoil();

  std::cout << "Creating external_binary.dat" << std::endl;
  gp << "splot" << gp.binFile1d(vecs, "record", "external_binary.dat")
     << "with lines notitle\n";

  std::cout << "Now run 'gnuplot -persist external_binary.gnu'.\n";
}

void demo_animation() {
#ifdef _WIN32
  // No animation demo for Windows.  The problem is that every time the plot
  // is updated, the gnuplot window grabs focus.  So you can't ever focus the
  // terminal window to press Ctrl-C.  The only way to quit is to right-click
  // the terminal window on the task bar and close it from there.  Other than
  // that, it seems to work.
  std::cout << "Sorry, the animation demo doesn't work in Windows."
            << std::endl;
  return;
#endif

  Gnuplot gp;

  std::cout << "Press Ctrl-C to quit (closing gnuplot window doesn't quit)."
            << std::endl;

  gp << "set yrange [-1:1]\n";

  const int N = 1000;
  std::vector<double> pts(N);

  double theta = 0;
  while (1) {
    for (int i = 0; i < N; i++) {
      double alpha = (double(i) / N - 0.5) * 10;
      pts[i] = sin(alpha * 8.0 + theta) * exp(-alpha * alpha / 2.0);
    }

    gp << "plot '-' binary" << gp.binFmt1d(pts, "array")
       << "with lines notitle\n";
    gp.sendBinary1d(pts);
    gp.flush();

    theta += 0.2;
    mysleep(100);
  }
}

void demo_NaN() {
  // Demo of NaN (not-a-number) usage.  Plot a circle that has half the
  // coordinates replaced by NaN values.

  double nan = std::numeric_limits<double>::quiet_NaN();

  Gnuplot gp;

  std::vector<std::pair<double, double>> xy_pts;
  for (int i = 0; i < 100; i++) {
    double theta = double(i) / 100 * 2 * M_PI;
    if ((i / 5) % 2) {
      xy_pts.push_back(std::make_pair(std::cos(theta), std::sin(theta)));
    } else {
      xy_pts.push_back(std::make_pair(nan, nan));
    }
  }

  // You need to tell gnuplot that 'nan' should be treated as missing data
  // (otherwise it just gives an error).
  gp << "set datafile missing 'nan'\n";
  gp << "plot '-' with linespoints\n";
  gp.send1d(xy_pts);

  // This works too.  But the strange thing is that with text data the segments
  // are joined by lines and with binary data the segments are not joined.
  // gp << "plot '-' binary" << gp.binFmt1d(xy_pts, "record") << "with
  // linespoints\n"; gp.sendBinary1d(xy_pts);

  pause_if_needed();
}

void demo_segments() {
  // Demo of disconnected segments.  Plot a circle with some pieces missing.

  Gnuplot gp;

  std::vector<std::vector<std::pair<double, double>>> all_segments;
  for (int j = 0; j < 10; j++) {
    std::vector<std::pair<double, double>> segment;
    for (int i = 0; i < 5; i++) {
      double theta = double(j * 10 + i) / 100 * 2 * M_PI;
      segment.push_back(std::make_pair(std::cos(theta), std::sin(theta)));
    }
    all_segments.push_back(segment);
  }

  gp << "plot '-' with linespoints\n";
  // NOTE: send2d is used here, rather than send1d.  This puts a blank line
  // between segments.
  gp.send2d(all_segments);

  pause_if_needed();
}

void demo_image() {
  // Example of plotting an image.  Of course you are free (and encouraged) to
  // use Blitz or Armadillo rather than std::vector in these situations.

  Gnuplot gp;

  std::vector<std::vector<double>> image;
  for (int j = 0; j < 100; j++) {
    std::vector<double> row;
    for (int i = 0; i < 100; i++) {
      double x = (i - 50.0) / 5.0;
      double y = (j - 50.0) / 5.0;
      double z = std::cos(sqrt(x * x + y * y));
      row.push_back(z);
    }
    image.push_back(row);
  }

  // It may seem counterintuitive that send1d should be used rather than
  // send2d.  The explanation is as follows.  The "send2d" method puts each
  // value on its own line, with blank lines between rows.  This is what is
  // expected by the splot command.  The two "dimensions" here are the lines
  // and the blank-line-delimited blocks.  The "send1d" method doesn't group
  // things into blocks.  So the elements of each row are printed as columns,
  // as expected by Gnuplot's "matrix with image" command.  But images
  // typically have lots of pixels, so sending as text is not the most
  // efficient (although, it's not really that bad in the case of this
  // example).  See the binary version below.
  //
  // gp << "plot '-' matrix with image\n";
  // gp.send1d(image);

  // To be honest, Gnuplot's documentation for "binary" and for "image" are
  // both unclear to me.  The following example comes by trial-and-error.
  gp << "plot '-' binary" << gp.binFmt2d(image, "array") << "with image\n";
  gp.sendBinary2d(image);

  pause_if_needed();
}

void demo_heatmap() {
  Gnuplot gp;

  gp << "set tic scale 0\n";
  gp << "set palette rgbformula 33,13,10\n";
  gp << "set cbrange [0:5]\n";
  gp << "set cblabel 'Score'\n";
  // gp << "unset cbtics\n";
  gp << "set title 'Heat Map generated from a stream of XYZ values'\n";
  std::vector<std::tuple<double, double, double>> x_y_z = {
      {0, 0, 5}, {0, 1, 4}, {0, 2, 3}, {0, 3, 1}, {0, 4, 0},
      {1, 0, 2}, {1, 1, 2}, {1, 2, 0}, {1, 3, 0}, {1, 4, 1},
      {2, 0, 0}, {2, 1, 0}, {2, 2, 0}, {2, 3, 1}, {2, 4, 0},
      {3, 0, 0}, {3, 1, 0}, {3, 2, 0}, {3, 3, 2}, {3, 4, 3},
      {4, 0, 0}, {4, 1, 1}, {4, 2, 2}, {4, 3, 3}, {4, 4, 3}};

  gp << "plot " << gp.file1d(x_y_z) << " with image\n";

  pause_if_needed();
}  // demo_heatmap

void demo_multiwindows() {
  Gnuplot gp;

  // the first window
  gp << "set terminal x11 size 600, 600 0\n";
  gp << "set multiplot layout 3, 1 title 'Multiplot layout 3, 1' font ',14'\n";
  gp << "set tmargin 2\n";
  gp << "set title 'Plot 1'\n";
  gp << "unset key\n";
  gp << "plot sin(x)/x\n";

  gp << "set title 'Plot 2'\n";
  gp << "unset key\n";
  gp << "plot log(x)\n";

  gp << "set style histogram columns\n";
  gp << "set style fill solid\n";
  gp << "set key autotitle column\n";
  gp << " set boxwidth 0.8\n";
  gp << "set tics scale 0\n";
  gp << "set title 'Plot 3'\n";
  gp << "plot sin(x)\n";
  gp << "unset multiplot\n";

  // the second window
  gp << "set terminal x11 size 600, 600 1\n";
  gp << "plot cos(x)\n";

  pause_if_needed();
}

void demo_2dcircle() {
  Gnuplot gp;

  std::vector<std::tuple<double, double, double>> x_y_radius;
  x_y_radius.push_back({1, 2, 2});
  gp << "set title 'Trace of unconstrained optimization with trust-region "
        "method'\n";
  gp << "unset key\n";
  gp << "set size ratio -1\n";
  gp << "set xrange [-3:3]\n";
  gp << "set yrange [-3:3]\n";
  gp << "plot '-' with circles lc rgb 'blue' fs transparent solid 0.15 "
        "noborder\n";
  gp.send1d(x_y_radius);

  pause_if_needed();
}  // demo_2dcircle

void demo_multiplots() {
  // multiple plot in one graphs

  Gnuplot gp;
  gp << "reset\n";
  gp << "set style function lines \n";
  gp << "set size 1.0, 1.0 \n";
  gp << "set origin 0.0, 0.0 \n";
  gp << "set multiplot \n";
  gp << "set size 0.5,0.5 \n";
  gp << "set origin 0.0,0.5 \n";
  gp << "set grid \n";
  gp << "unset key\n ";
  gp << "set angles radians\n ";
  gp << "set samples 250\n ";

  // Plot Magnitude Response
  gp << "set title 'Second Order System Transfer Function - Magnitude'\n ";
  gp << "mag(w) = -10*log10( (1-w**2)**2 + 4*(zeta*w)**2)\n";
  gp << "set dummy w\n ";
  gp << "set logscale x\n ";
  gp << "set xlabel 'Frequency (w/wn)'\n ";
  gp << "set ylabel 'Magnitude (dB)' offset 1,0\n ";
  gp << "set label 1 'Damping =.1,.2,.3,.4,.5,.707,1.0,2.0' at .14,17 \n ";
  gp << "set xrange [.1:10]\n ";
  gp << "set yrange [-40:20]\n ";
  gp << "plot zeta=.1,mag(w), zeta=.2,mag(w), zeta=.3,mag(w), "
        "zeta=.4,mag(w), zeta=.5,mag(w), zeta=.707,mag(w), zeta=1.0,mag(w), "
        "zeta=2.0,mag(w),-6\n ";
  // Plot Phase Response
  gp << "set size 0.5,0.5\n";
  gp << "set origin 0.0,0.0\n";
  gp << "set title 'Second Order System Transfer Function - Phase'\n";
  gp << "set label 1 ''\n";
  gp << "set ylabel ' Phase(deg) ' offset 1,0\n";
  gp << "set ytics -180, 30, 0 \n";
  gp << "set yrange [-180:0]\n";
  gp << "tmp(w) = (-180/pi)*atan( 2*zeta*w/(1-w**2) )\n ";

  // Fix for atan function wrap problem
  gp << "tmp1(w)= w<1?tmp(w):(tmp(w)-180)\n";
  gp << "phi(w)=zeta==1?(-2*(180/pi)*atan(w)):tmp1(w)\n";
  gp << "plot zeta=.1,phi(w), zeta=.2,phi(w), zeta=.3,phi(w), zeta=.4,phi(w), "
        "zeta=.5,phi(w),  zeta=.707,phi(w), zeta=1,phi(w), zeta=2.0,phi(w), "
        "-90\n";

  // Plot Step Response
  gp << "set size 0.5,0.5 \n";
  gp << "set origin 0.5,0.5 \n";
  gp << "set dummy wnt \n";
  gp << "unset logscale x\n ";
  gp << "set title 'Second Order System - Unit Step Response' \n";
  gp << "set ylabel ' Amplitude y(wnt) ' offset 1,0  \n";
  gp << "set xlabel 'Normalized Time (wnt)' \n";
  gp << "set xrange [0:20] \n";
  gp << "set xtics 0,5,20 \n";
  gp << "set yrange [0:2.0] \n";
  gp << "set ytics 0, .5, 2.0 \n";
  gp << "set mytics 5 \n";
  gp << "set mxtics 10 \n";
  gp << "wdwn(zeta)=sqrt(1-zeta**2) \n";
  gp << "shift(zeta) = atan(wdwn(zeta)/zeta) \n";
  gp << "alpha(zeta)=zeta>1?sqrt(zeta**2-1.0):0 \n";
  gp << "tau1(zeta)=1/(zeta-alpha(zeta)) \n";
  gp << "tau2(zeta)=1/(zeta+alpha(zeta)) \n";
  gp << "c1(zeta)=(zeta + alpha(zeta))/(2*alpha(zeta)) \n";
  gp << "c2(zeta)=c1(zeta)-1 \n";
  gp << "y1(wnt)=zeta==1?1 - exp(-wnt)*(wnt + 1):0 \n";
  gp << "y2(wnt)=zeta<1?(1 - (exp(-zeta*wnt)/wdwn(zeta))*sin(wdwn(zeta)*wnt + "
        "shift(zeta))):y1(wnt) \n";
  gp << "y(wnt)=zeta>1?1-c1(zeta)*exp(-wnt/tau1(zeta))+c2(zeta)*exp(-wnt/"
        "tau2(zeta)):y2(wnt) \n";
  gp << "plot zeta=.1,y(wnt), zeta=.2,y(wnt), zeta=.3,y(wnt), zeta=.4,y(wnt), "
        "zeta=.5,y(wnt), zeta=.707,y(wnt), zeta=1,y(wnt), zeta=2,y(wnt)\n";

  // Plot Impulse Response
  gp << "set origin .5,0.\n";
  gp << "set title 'Second Order System - Unit Impulse Response'\n";
  gp << "y(wnt)=exp(-zeta*wnt) * sin(wdwn(zeta)*wnt) / wdwn(zeta)\n";
  gp << "set yrange [-1. :1.]\n";
  gp << "set ytics -1,.5,1.\n";
  gp << "plot zeta=.1,y(wnt), zeta=.2,y(wnt), zeta=.3,y(wnt), "
        "zeta=.4,y(wnt), zeta=.5,y(wnt), zeta=.707,y(wnt), zeta=1,y(wnt),  "
        "zeta=2,y(wnt)\n";

  gp << "unset multiplot\n";
  pause_if_needed();
}

int main(int argc, char **argv) {
  std::map<std::string, void (*)(void)> demos;

  demos["basic"] = demo_basic;
  demos["binary"] = demo_binary;
  demos["tmpfile"] = demo_tmpfile;
  demos["png"] = demo_png;
  demos["vectors"] = demo_vectors;
  demos["script_inline_text"] = demo_inline_text;
  demos["script_inline_binary"] = demo_inline_binary;
  demos["script_external_text"] = demo_external_text;
  demos["script_external_binary"] = demo_external_binary;
  demos["animation"] = demo_animation;
  demos["nan"] = demo_NaN;
  demos["segments"] = demo_segments;
  demos["image"] = demo_image;
  demos["multiplot"] = demo_multiplots;
  demos["multiwindows"] = demo_multiwindows;
  demos["2Dpolygon"] = demo_2Dpolygon;
  demos["2Dcircle"] = demo_2dcircle;
  demos["heatmap"] = demo_heatmap;

  if (argc < 2) {
    printf("Usage: %s <demo_name>\n", argv[0]);
    printf("Choose one of the following demos:\n");
    typedef std::pair<std::string, void (*)(void)> demo_pair;
    BOOST_FOREACH (const demo_pair &pair, demos) {
      printf("    %s\n", pair.first.c_str());
    }
    return 0;
  }

  std::string arg(argv[1]);
  if (!demos.count(arg)) {
    printf("No such demo '%s'\n", arg.c_str());
    return 1;
  }

  demos[arg]();
}
