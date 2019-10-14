/*
***********************************************************************
* reeds_shepp_path.h:
* reeds shepp path
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#pragma once

#include <omp.h>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/math/math_utils.h"
#include "modules/planning/open_space/coarse_trajectory_generator/node3d.h"
#include "modules/planning/proto/planner_open_space_config.pb.h"

#include "modules/planning/common/planning_gflags.h"

#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>
#include <iostream>
#include "common/math/miscellaneous/include/math_utils.h"
#include "planner/openspace/include/Node3D.h"
#include "planner/openspace/include/openspacedata.h"

namespace ASV::planning {

struct ReedSheppPath {
  std::vector<double> segs_lengths;
  std::string segs_types;
  double total_length = 0.0;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  // true for driving forward and false for driving backward
  std::vector<bool> gear;
};

struct RSPParam {
  bool flag = false;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
};

class ReedShepp {
 public:
  ReedShepp(const common::VehicleParam& vehicle_param,
            const PlannerOpenSpaceConfig& open_space_conf)
      : _vehicle_param(vehicle_param),
        planner_open_space_config_(open_space_conf) {
    if (FLAGS_enable_parallel_hybrid_a) std::cout << "parallel REEDShepp\n";
  }
  virtual ~ReedShepp() = default;
  // Pick the shortest path from all possible combination of movement primitives
  // by Reed Shepp
  bool ShortestRSP(const std::shared_ptr<Node3d> start_node,
                   const std::shared_ptr<Node3d> end_node,
                   ReedSheppPath& _optimal_path) {
    std::vector<ReedSheppPath> all_possible_paths;
    if (!GenerateRSPs(start_node, end_node, all_possible_paths)) {
      std::cout << "Fail to generate different combination of Reed Shepp "
                   "paths";
      return false;
    }

    double optimal_path_length = std::numeric_limits<double>::infinity();
    std::size_t optimal_path_index = 0;
    for (std::size_t i = 0; i < all_possible_paths.size(); ++i) {
      if (all_possible_paths.at(i).total_length > 0 &&
          all_possible_paths.at(i).total_length < optimal_path_length) {
        optimal_path_index = i;
        optimal_path_length = all_possible_paths.at(i).total_length;
      }
    }

    if (!GenerateLocalConfigurations(start_node, end_node,
                                     all_possible_paths[optimal_path_index])) {
      std::cout << "Fail to generate local configurations(x, y, phi) in SetRSP";
      return false;
    }

    if (std::abs(all_possible_paths[optimal_path_index].x.back() -
                 end_node->GetX()) > 1e-3 ||
        std::abs(all_possible_paths[optimal_path_index].y.back() -
                 end_node->GetY()) > 1e-3 ||
        std::abs(all_possible_paths[optimal_path_index].phi.back() -
                 end_node->GetPhi()) > 1e-3) {
      std::cout << "RSP end position not right";
      for (size_t i = 0;
           i < all_possible_paths[optimal_path_index].segs_types.size(); ++i) {
        std::cout << "types are "
                  << all_possible_paths[optimal_path_index].segs_types[i];
      }
      std::cout << "x, y, phi are: "
                << all_possible_paths[optimal_path_index].x.back() << ", "
                << all_possible_paths[optimal_path_index].y.back() << ", "
                << all_possible_paths[optimal_path_index].phi.back();
      std::cout << "end x, y, phi are: " << end_node->GetX() << ", "
                << end_node->GetY() << ", " << end_node->GetPhi();
      return false;
    }
    _optimal_path.x = all_possible_paths[optimal_path_index].x;
    _optimal_path.y = all_possible_paths[optimal_path_index].y;
    _optimal_path.phi = all_possible_paths[optimal_path_index].phi;
    _optimal_path.gear = all_possible_paths[optimal_path_index].gear;
    _optimal_path.total_length =
        all_possible_paths[optimal_path_index].total_length;
    _optimal_path.segs_types =
        all_possible_paths[optimal_path_index].segs_types;
    _optimal_path.segs_lengths =
        all_possible_paths[optimal_path_index].segs_lengths;
    return true;
  }  // ShortestRSP

 protected:
  // Generate all possible combination of movement primitives by Reed Shepp and
  // interpolate them
  bool GenerateRSPs(const std::shared_ptr<Node3d> start_node,
                    const std::shared_ptr<Node3d> end_node,
                    std::vector<ReedSheppPath>& all_possible_paths) {
    if (FLAGS_enable_parallel_hybrid_a) {
      // AINFO << "parallel hybrid a*";
      if (!GenerateRSPPar(start_node, end_node, all_possible_paths)) {
        std::cout << "Fail to generate general profile of different RSPs";
        return false;
      }
    } else {
      if (!GenerateRSP(start_node, end_node, all_possible_paths)) {
        std::cout << "Fail to generate general profile of different RSPs";
        return false;
      }
    }
    return true;
  }  // GenerateRSPs

  // Set the general profile of the movement primitives
  bool GenerateRSP(const std::shared_ptr<Node3d> start_node,
                   const std::shared_ptr<Node3d> end_node,
                   std::vector<ReedSheppPath>& all_possible_paths) {
    double dx = end_node->GetX() - start_node->GetX();
    double dy = end_node->GetY() - start_node->GetY();
    double dphi = end_node->GetPhi() - start_node->GetPhi();
    double c = std::cos(start_node->GetPhi());
    double s = std::sin(start_node->GetPhi());
    // normalize the initial point to (0,0,0)
    double x = (c * dx + s * dy) * _vehicle_param.max_kappa;
    double y = (-s * dx + c * dy) * _vehicle_param.max_kappa;
    if (!SCS(x, y, dphi, all_possible_paths)) {
      std::cout << "Fail at SCS";
    }
    if (!CSC(x, y, dphi, all_possible_paths)) {
      std::cout << "Fail at CSC";
    }
    if (!CCC(x, y, dphi, all_possible_paths)) {
      std::cout << "Fail at CCC";
    }
    if (!CCCC(x, y, dphi, all_possible_paths)) {
      std::cout << "Fail at CCCC";
    }
    if (!CCSC(x, y, dphi, all_possible_paths)) {
      std::cout << "Fail at CCSC";
    }
    if (!CCSCC(x, y, dphi, all_possible_paths)) {
      std::cout << "Fail at CCSCC";
    }
    if (all_possible_paths.empty()) {
      std::cout << "No path generated by certain two configurations";
      return false;
    }
    return true;
  }  // GenerateRSP

  // Set the general profile of the movement primitives, parallel implementation
  bool GenerateRSPPar(const std::shared_ptr<Node3d> start_node,
                      const std::shared_ptr<Node3d> end_node,
                      std::vector<ReedSheppPath>& all_possible_paths) {
    double dx = end_node->GetX() - start_node->GetX();
    double dy = end_node->GetY() - start_node->GetY();
    double dphi = end_node->GetPhi() - start_node->GetPhi();
    double c = std::cos(start_node->GetPhi());
    double s = std::sin(start_node->GetPhi());
    // normalize the initial point to (0,0,0)
    double x = (c * dx + s * dy) * _vehicle_param.max_kappa;
    double y = (-s * dx + c * dy) * _vehicle_param.max_kappa;
    // backward
    double xb = x * std::cos(dphi) + y * std::sin(dphi);
    double yb = x * std::sin(dphi) - y * std::cos(dphi);

    int RSP_nums = 46;  // there are 46 possibilities for RS paths
    all_possible_paths.resize(RSP_nums);
    bool succ = true;
    // for loop using 2 threads
#pragma omp parallel for num_threads(2)
    for (int i = 0; i < RSP_nums; ++i) {
      RSPParam RSP_param;
      double RSP_lengths[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
      std::vector<double> RSP_lengths;
      std::string rd_type;
      double x_param = 1.0;
      double y_param = 1.0;

      if (i > 2 && i % 2) {
        x_param = -1.0;
      }
      if (i > 2 && i % 4 < 2) {
        y_param = -1.0;
      }

      if (i < 2) {  // SCS case
        if (i == 1) {
          y_param = -1.0;
          rd_type = "SRS";
        } else {
          rd_type = "SLS";
        }
        SLS(x, y_param * y, y_param * dphi, RSP_param);
        RSP_lengths.resize(3);
        RSP_lengths[0] = RSP_param.t;
        RSP_lengths[1] = RSP_param.u;
        RSP_lengths[2] = RSP_param.v;
      } else if (i < 6) {  // CSC, LSL case
        LSL(x_param * x, y_param * y, x_param * y_param * dphi, RSP_param);
        if (y_param > 0) {
          rd_type = "LSL";
        } else {
          rd_type = "RSR";
        }
        RSP_lengths.resize(3);
        RSP_lengths[0] = x_param * RSP_param.t;
        RSP_lengths[1] = x_param * RSP_param.u;
        RSP_lengths[2] = x_param * RSP_param.v;
      } else if (i < 10) {  // CSC, LSR case
        LSR(x_param * x, y_param * y, x_param * y_param * dphi, RSP_param);
        if (y_param > 0) {
          rd_type = "LSR";
        } else {
          rd_type = "RSL";
        }
        RSP_lengths.resize(3);
        RSP_lengths[0] = x_param * RSP_param.t;
        RSP_lengths[1] = x_param * RSP_param.u;
        RSP_lengths[2] = x_param * RSP_param.v;
      } else if (i < 14) {  // CCC, LRL case
        LRL(x_param * x, y_param * y, x_param * y_param * dphi, RSP_param);
        if (y_param > 0) {
          rd_type = "LRL";
        } else {
          rd_type = "RLR";
        }
        RSP_lengths.resize(3);
        RSP_lengths[0] = x_param * RSP_param.t;
        RSP_lengths[1] = x_param * RSP_param.u;
        RSP_lengths[2] = x_param * RSP_param.v;
      } else if (i < 18) {  // CCC, LRL case, backward
        LRL(x_param * xb, y_param * yb, x_param * y_param * dphi, RSP_param);
        if (y_param > 0) {
          rd_type = "LRL";
        } else {
          rd_type = "RLR";
        }
        RSP_lengths.resize(3);
        RSP_lengths[0] = x_param * RSP_param.v;
        RSP_lengths[1] = x_param * RSP_param.u;
        RSP_lengths[2] = x_param * RSP_param.t;
      } else if (i < 22) {  // CCCC, LRLRn
        LRLRn(x_param * x, y_param * y, x_param * y_param * dphi, RSP_param);
        if (y_param > 0.0) {
          rd_type = "LRLR";
        } else {
          rd_type = "RLRL";
        }

        RSP_lengths.resize(4);
        RSP_lengths[0] = x_param * RSP_param.t;
        RSP_lengths[1] = x_param * RSP_param.u;
        RSP_lengths[2] = -x_param * RSP_param.u;
        RSP_lengths[3] = x_param * RSP_param.v;
      } else if (i < 26) {  // CCCC, LRLRp
        LRLRp(x_param * x, y_param * y, x_param * y_param * dphi, RSP_param);
        if (y_param > 0.0) {
          rd_type = "LRLR";
        } else {
          rd_type = "RLRL";
        }
        RSP_lengths.resize(4);
        RSP_lengths[0] = x_param * RSP_param.t;
        RSP_lengths[1] = x_param * RSP_param.u;
        RSP_lengths[2] = -x_param * RSP_param.u;
        RSP_lengths[3] = x_param * RSP_param.v;
      } else if (i < 30) {  // CCSC, LRLRn
        RSP_lengths.resize(4);

        LRLRn(x_param * x, y_param * y, x_param * y_param * dphi, RSP_param);
        if (y_param > 0.0) {
          rd_type = "LRSL";
        } else {
          rd_type = "RLSR";
        }
        RSP_lengths[0] = x_param * RSP_param.t;
        if (x_param < 0 && y_param > 0) {
          RSP_lengths[1] = 0.5 * M_PI;
        } else {
          RSP_lengths[1] = -0.5 * M_PI;
        }
        if (x_param > 0 && y_param < 0) {
          RSP_lengths[2] = RSP_param.u;
        } else {
          RSP_lengths[2] = -RSP_param.u;
        }
        RSP_lengths[3] = x_param * RSP_param.v;
      } else if (i < 34) {  // CCSC, LRLRp
        RSP_lengths.resize(4);
        LRLRp(x_param * x, y_param * y, x_param * y_param * dphi, RSP_param);
        if (y_param) {
          rd_type = "LRSR";
        } else {
          rd_type = "RLSL";
        }
        RSP_lengths[0] = x_param * RSP_param.t;
        if (x_param < 0 && y_param > 0) {
          RSP_lengths[1] = 0.5 * M_PI;
        } else {
          RSP_lengths[1] = -0.5 * M_PI;
        }
        RSP_lengths[2] = x_param * RSP_param.u;
        RSP_lengths[3] = x_param * RSP_param.v;
      } else if (i < 38) {  // CCSC, LRLRn, backward
        RSP_lengths.resize(4);
        LRLRn(x_param * xb, y_param * yb, x_param * y_param * dphi, RSP_param);
        if (y_param > 0) {
          rd_type = "LSRL";
        } else {
          rd_type = "RSLR";
        }
        RSP_lengths[0] = x_param * RSP_param.v;
        RSP_lengths[1] = x_param * RSP_param.u;
        RSP_lengths[2] = -x_param * 0.5 * M_PI;
        RSP_lengths[3] = x_param * RSP_param.t;
      } else if (i < 42) {  // CCSC, LRLRp, backward
        RSP_lengths.resize(4);
        LRLRp(x_param * xb, y_param * yb, x_param * y_param * dphi, RSP_param);
        if (y_param > 0) {
          rd_type = "RSRL";
        } else {
          rd_type = "LSLR";
        }
        RSP_lengths[0] = x_param * RSP_param.v;
        RSP_lengths[1] = x_param * RSP_param.u;
        RSP_lengths[2] = -x_param * M_PI * 0.5;
        RSP_lengths[3] = x_param * RSP_param.t;
      } else {  // CCSCC, LRSLR
        RSP_lengths.resize(5);
        LRSLR(x_param * x, y_param * y, x_param * y_param * dphi, RSP_param);
        if (y_param > 0.0) {
          rd_type = "LRSLR";
        } else {
          rd_type = "RLSRL";
        }
        RSP_lengths[0] = x_param * RSP_param.t;
        RSP_lengths[1] = -x_param * 0.5 * M_PI;
        RSP_lengths[2] = x_param * RSP_param.u;
        RSP_lengths[3] = -x_param * 0.5 * M_PI;
        RSP_lengths[4] = x_param * RSP_param.v;
      }

      if (RSP_param.flag &&
          !SetRSPPar(RSP_lengths, rd_type, all_possible_paths, i)) {
        std::cout << "Fail at SetRSP, idx#: " << i;
        succ = false;
      }
    }  // end for loop

    if (!succ) {
      std::cout << "RSP parallel fails";
      return false;
    }
    if (all_possible_paths.size() == 0) {
      std::cout << "No path generated by certain two configurations";
      return false;
    }
    return true;
  }  // GenerateRSPPar

  // Set local exact configurations profile of each movement primitive
  bool GenerateLocalConfigurations(const std::shared_ptr<Node3d> start_node,
                                   const std::shared_ptr<Node3d> end_node,
                                   ReedSheppPath& shortest_path) {
    double step_scaled =
        planner_open_space_config_.warm_start_config().step_size() *
        _vehicle_param.max_kappa;

    size_t point_num = static_cast<size_t>(
        std::floor(shortest_path.total_length / step_scaled +
                   static_cast<double>(shortest_path.segs_lengths.size()) + 4));
    std::vector<double> px(point_num, 0.0);
    std::vector<double> py(point_num, 0.0);
    std::vector<double> pphi(point_num, 0.0);
    std::vector<bool> pgear(point_num, true);
    int index = 1;
    double d = 0.0;
    double pd = 0.0;
    double ll = 0.0;

    if (shortest_path.segs_lengths.at(0) > 0.0) {
      pgear.at(0) = true;
      d = step_scaled;
    } else {
      pgear.at(0) = false;
      d = -step_scaled;
    }
    pd = d;
    for (std::size_t i = 0; i < shortest_path.segs_types.size(); ++i) {
      char m = shortest_path.segs_types.at(i);
      double l = shortest_path.segs_lengths.at(i);
      if (l > 0.0) {
        d = step_scaled;
      } else {
        d = -step_scaled;
      }
      double ox = px.at(index);
      double oy = py.at(index);
      double ophi = pphi.at(index);
      index--;
      if (i >= 1 && shortest_path.segs_lengths.at(i - 1) *
                            shortest_path.segs_lengths.at(i) >
                        0) {
        pd = -d - ll;
      } else {
        pd = d - ll;
      }
      while (std::abs(pd) <= std::abs(l)) {
        index++;
        Interpolation(index, pd, m, ox, oy, ophi, px, py, pphi, pgear);
        pd += d;
      }
      ll = l - pd - d;
      index++;
      Interpolation(index, l, m, ox, oy, ophi, px, py, pphi, pgear);
    }
    double epsilon = 1e-15;
    while (std::fabs(px.back()) < epsilon && std::fabs(py.back()) < epsilon &&
           std::fabs(pphi.back()) < epsilon && pgear.back()) {
      px.pop_back();
      py.pop_back();
      pphi.pop_back();
      pgear.pop_back();
    }

    for (size_t i = 0; i < px.size(); ++i) {
      shortest_path.x.emplace_back(std::cos(-start_node->GetPhi()) * px.at(i) +
                                   std::sin(-start_node->GetPhi()) * py.at(i) +
                                   start_node->GetX());
      shortest_path.y.emplace_back(-std::sin(-start_node->GetPhi()) * px.at(i) +
                                   std::cos(-start_node->GetPhi()) * py.at(i) +
                                   start_node->GetY());
      shortest_path.phi.emplace_back(common::math::Normalizeheadingangle(
          pphi.at(i) + start_node->GetPhi()));
    }
    shortest_path.gear = pgear;
    for (std::size_t i = 0; i < shortest_path.segs_lengths.size(); ++i) {
      shortest_path.segs_lengths.at(i) =
          shortest_path.segs_lengths.at(i) / _vehicle_param.max_kappa;
    }
    shortest_path.total_length =
        shortest_path.total_length / _vehicle_param.max_kappa;
    return true;
  }  // GenerateLocalConfigurations

  // Interpolation usde in GenetateLocalConfiguration
  void Interpolation(const int index, const double pd, const char m,
                     const double ox, const double oy, const double ophi,
                     std::vector<double>& px, std::vector<double>& py,
                     std::vector<double>& pphi, std::vector<bool>& pgear) {
    double ldx = 0.0;
    double ldy = 0.0;
    double gdx = 0.0;
    double gdy = 0.0;
    if (m == 'S') {
      px.at(index) = ox + pd / _vehicle_param.max_kappa * std::cos(ophi);
      py.at(index) = oy + pd / _vehicle_param.max_kappa * std::sin(ophi);
      pphi.at(index) = ophi;
    } else {
      ldx = std::sin(pd) / _vehicle_param.max_kappa;
      if (m == 'L') {
        ldy = (1.0 - std::cos(pd)) / _vehicle_param.max_kappa;
      } else if (m == 'R') {
        ldy = (1.0 - std::cos(pd)) / -_vehicle_param.max_kappa;
      }
      gdx = std::cos(-ophi) * ldx + std::sin(-ophi) * ldy;
      gdy = -std::sin(-ophi) * ldx + std::cos(-ophi) * ldy;
      px.at(index) = ox + gdx;
      py.at(index) = oy + gdy;
    }

    if (pd > 0.0) {
      pgear.at(index) = true;
    } else {
      pgear.at(index) = false;
    }

    if (m == 'L') {
      pphi.at(index) = ophi + pd;
    } else if (m == 'R') {
      pphi.at(index) = ophi - pd;
    }
  }  // Interpolation

  // motion primitives combination setup function
  bool SetRSP(const std::vector<double>& _lengths, const std::string& _types,
              std::vector<ReedSheppPath>& _all_possible_paths) {
    ReedSheppPath path;
    path.segs_lengths = _lengths;
    path.segs_types = _types;
    double sum = 0.0;
    for (std::size_t i = 0; i != _lengths.size(); ++i) {
      sum += std::abs(_lengths[i]);
    }
    path.total_length = sum;
    if (path.total_length <= 0.0) {
      std::cout << "total length smaller than 0\n";
      return false;
    }
    _all_possible_paths.emplace_back(path);
    return true;
  }

  // setRSP parallel version
  bool SetRSPPar(const std::vector<double>& _lengths, const std::string& _types,
                 std::vector<ReedSheppPath>& all_possible_paths,
                 const int idx) {
    ReedSheppPath path;
    path.segs_lengths = _lengths;
    path.segs_types = _types;
    double sum = 0.0;
    for (std::size_t i = 0; i != _lengths.size(); ++i) {
      sum += std::abs(lengths[i]);
    }
    path.total_length = sum;
    if (path.total_length <= 0.0) {
      std::cout << "total length smaller than 0";
      return false;
    }

    all_possible_paths[idx] = path;
    return true;
  }

  // Six different combination of motion primitive in Reed Shepp path used in
  // GenerateRSP()
  bool SCS(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>& all_possible_paths) {
    RSPParam SLS_param;
    SLS(x, y, phi, SLS_param);
    std::vector<double> SLS_lengths = {SLS_param.t, SLS_param.u, SLS_param.v};
    if (SLS_param.flag && !SetRSP(SLS_lengths, "SLS", all_possible_paths)) {
      std::cout << "Fail at SetRSP with SLS_param";
      return false;
    }

    RSPParam SRS_param;
    SLS(x, -y, -phi, SRS_param);
    std::vector<double> SRS_lengths = {SRS_param.t, SRS_param.u, SRS_param.v};
    if (SRS_param.flag && !SetRSP(SRS_lengths, "SRS", all_possible_paths)) {
      std::cout << "Fail at SetRSP with SRS_param";
      return false;
    }
    return true;
  }  // SCS

  bool CSC(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>& all_possible_paths) {
    RSPParam LSL1_param;
    LSL(x, y, phi, LSL1_param);
    std::vector<double> LSL1_lengths = {LSL1_param.t, LSL1_param.u,
                                        LSL1_param.v};
    if (LSL1_param.flag && !SetRSP(LSL1_lengths, "LSL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LSL_param";
      return false;
    }

    RSPParam LSL2_param;
    LSL(-x, y, -phi, LSL2_param);
    std::vector<double> LSL2_lengths = {-LSL2_param.t, -LSL2_param.u,
                                        -LSL2_param.v};
    if (LSL2_param.flag && !SetRSP(LSL2_lengths, "LSL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LSL2_param";
      return false;
    }

    RSPParam LSL3_param;
    LSL(x, -y, -phi, LSL3_param);
    std::vector<double> LSL3_lengths = {LSL3_param.t, LSL3_param.u,
                                        LSL3_param.v};
    if (LSL3_param.flag && !SetRSP(LSL3_lengths, "RSR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LSL3_param";
      return false;
    }

    RSPParam LSL4_param;
    LSL(-x, -y, phi, LSL4_param);
    std::vector<double> LSL4_lengths = {-LSL4_param.t, -LSL4_param.u,
                                        -LSL4_param.v};
    if (LSL4_param.flag && !SetRSP(LSL4_lengths, "RSR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LSL4_param";
      return false;
    }

    RSPParam LSR1_param;
    LSR(x, y, phi, LSR1_param);
    std::vector<double> LSR1_lengths = {LSR1_param.t, LSR1_param.u,
                                        LSR1_param.v};
    if (LSR1_param.flag && !SetRSP(LSR1_lengths, "LSR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LSR1_param";
      return false;
    }

    RSPParam LSR2_param;
    LSR(-x, y, -phi, LSR2_param);
    std::vector<double> LSR2_lengths = {-LSR2_param.t, -LSR2_param.u,
                                        -LSR2_param.v};
    if (LSR2_param.flag && !SetRSP(LSR2_lengths, "LSR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LSR2_param";
      return false;
    }

    RSPParam LSR3_param;
    LSR(x, -y, -phi, LSR3_param);
    std::vector<double> LSR3_lengths = {LSR3_param.t, LSR3_param.u,
                                        LSR3_param.v};
    if (LSR3_param.flag && !SetRSP(LSR3_lengths, "RSL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LSR3_param";
      return false;
    }

    RSPParam LSR4_param;
    LSR(-x, -y, phi, LSR4_param);
    std::vector<double> LSR4_lengths = {-LSR4_param.t, -LSR4_param.u,
                                        -LSR4_param.v};
    if (LSR4_param.flag && !SetRSP(LSR4_lengths, "RSL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LSR4_param";
      return false;
    }
    return true;
  }  // CSC

  bool CCC(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>& all_possible_paths) {
    RSPParam LRL1_param;
    LRL(x, y, phi, LRL1_param);
    std::vector<double> LRL1_lengths = {LRL1_param.t, LRL1_param.u,
                                        LRL1_param.v};
    if (LRL1_param.flag && !SetRSP(LRL1_lengths, "LRL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRL_param";
      return false;
    }

    RSPParam LRL2_param;
    LRL(-x, y, -phi, LRL2_param);
    std::vector<double> LRL2_lengths = {-LRL2_param.t, -LRL2_param.u,
                                        -LRL2_param.v};
    if (LRL2_param.flag && !SetRSP(LRL2_lengths, "LRL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRL2_param";
      return false;
    }

    RSPParam LRL3_param;
    LRL(x, -y, -phi, LRL3_param);
    std::vector<double> LRL3_lengths = {LRL3_param.t, LRL3_param.u,
                                        LRL3_param.v};
    if (LRL3_param.flag && !SetRSP(LRL3_lengths, "RLR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRL3_param";
      return false;
    }

    RSPParam LRL4_param;
    LRL(-x, -y, phi, LRL4_param);
    std::vector<double> LRL4_lengths = {-LRL4_param.t, -LRL4_param.u,
                                        -LRL4_param.v};
    if (LRL4_param.flag && !SetRSP(LRL4_lengths, "RLR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRL4_param";
      return false;
    }

    // backward
    double xb = x * std::cos(phi) + y * std::sin(phi);
    double yb = x * std::sin(phi) - y * std::cos(phi);

    RSPParam LRL5_param;
    LRL(xb, yb, phi, LRL5_param);
    std::vector<double> LRL5_lengths = {LRL5_param.v, LRL5_param.u,
                                        LRL5_param.t};
    if (LRL5_param.flag && !SetRSP(LRL5_lengths, "LRL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRL5_param";
      return false;
    }

    RSPParam LRL6_param;
    LRL(-xb, yb, -phi, LRL6_param);
    std::vector<double> LRL6_lengths = {-LRL6_param.v, -LRL6_param.u,
                                        -LRL6_param.t};
    if (LRL6_param.flag && !SetRSP(LRL6_lengths, "LRL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRL6_param";
      return false;
    }

    RSPParam LRL7_param;
    LRL(xb, -yb, -phi, LRL7_param);
    std::vector<double> LRL7_lengths = {LRL7_param.v, LRL7_param.u,
                                        LRL7_param.t};
    if (LRL7_param.flag && !SetRSP(LRL7_lengths, "RLR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRL7_param";
      return false;
    }

    RSPParam LRL8_param;
    LRL(-xb, -yb, phi, LRL8_param);
    std::vector<double> LRL8_lengths = {-LRL8_param.v, -LRL8_param.u,
                                        -LRL8_param.t};
    if (LRL8_param.flag && !SetRSP(LRL8_lengths, "RLR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRL8_param";
      return false;
    }
    return true;
  }  // CCC

  bool CCCC(const double x, const double y, const double phi,
            std::vector<ReedSheppPath>& all_possible_paths) {
    RSPParam LRLRn1_param;
    LRLRn(x, y, phi, LRLRn1_param);
    std::vector<double> LRLRn1_lengths = {LRLRn1_param.t, LRLRn1_param.u,
                                          -LRLRn1_param.u, LRLRn1_param.v};
    if (LRLRn1_param.flag &&
        !SetRSP(LRLRn1_lengths, "LRLR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRLRn_param";
      return false;
    }

    RSPParam LRLRn2_param;
    LRLRn(-x, y, -phi, LRLRn2_param);
    std::vector<double> LRLRn2_lengths = {-LRLRn2_param.t, -LRLRn2_param.u,
                                          LRLRn2_param.u, -LRLRn2_param.v};
    if (LRLRn2_param.flag &&
        !SetRSP(LRLRn2_lengths, "LRLR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRLRn2_param";
      return false;
    }

    RSPParam LRLRn3_param;
    LRLRn(x, -y, -phi, LRLRn3_param);
    std::vector<double> LRLRn3_lengths = {LRLRn3_param.t, LRLRn3_param.u,
                                          -LRLRn3_param.u, LRLRn3_param.v};
    if (LRLRn3_param.flag &&
        !SetRSP(LRLRn3_lengths, "RLRL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRLRn3_param";
      return false;
    }

    RSPParam LRLRn4_param;
    LRLRn(-x, -y, phi, LRLRn4_param);
    std::vector<double> LRLRn4_lengths = {-LRLRn4_param.t, -LRLRn4_param.u,
                                          LRLRn4_param.u, -LRLRn4_param.v};
    if (LRLRn4_param.flag &&
        !SetRSP(LRLRn4_lengths, "RLRL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRLRn4_param";
      return false;
    }

    RSPParam LRLRp1_param;
    LRLRp(x, y, phi, LRLRp1_param);
    std::vector<double> LRLRp1_lengths = {LRLRp1_param.t, LRLRp1_param.u,
                                          LRLRp1_param.u, LRLRp1_param.v};
    if (LRLRp1_param.flag &&
        !SetRSP(LRLRp1_lengths, "LRLR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRLRp1_param";
      return false;
    }

    RSPParam LRLRp2_param;
    LRLRp(-x, y, -phi, LRLRp2_param);
    std::vector<double> LRLRp2_lengths = {-LRLRp2_param.t, -LRLRp2_param.u,
                                          -LRLRp2_param.u, -LRLRp2_param.v};
    if (LRLRp2_param.flag &&
        !SetRSP(LRLRp2_lengths, "LRLR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRLRp2_param";
      return false;
    }

    RSPParam LRLRp3_param;
    LRLRp(x, -y, -phi, LRLRp3_param);
    std::vector<double> LRLRp3_lengths = {LRLRp3_param.t, LRLRp3_param.u,
                                          LRLRp3_param.u, LRLRp3_param.v};
    if (LRLRp3_param.flag &&
        !SetRSP(LRLRp3_lengths, "RLRL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRLRp3_param";
      return false;
    }

    RSPParam LRLRp4_param;
    LRLRp(-x, -y, phi, LRLRp4_param);
    std::vector<double> LRLRp4_lengths = {-LRLRp4_param.t, -LRLRp4_param.u,
                                          -LRLRp4_param.u, -LRLRp4_param.v};
    if (LRLRp4_param.flag &&
        !SetRSP(LRLRp4_lengths, "RLRL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRLRp4_param";
      return false;
    }
    return true;
  }  // CCCC

  bool CCSC(const double x, const double y, const double phi,
            std::vector<ReedSheppPath>& all_possible_paths) {
    RSPParam LRSL1_param;
    LRLRn(x, y, phi, LRSL1_param);
    std::vector<double> LRSL1_lengths = {LRSL1_param.t, -0.5 * M_PI,
                                         -LRSL1_param.u, LRSL1_param.v};
    if (LRSL1_param.flag &&
        !SetRSP(LRSL1_lengths, "LRSL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRSL1_param";
      return false;
    }

    RSPParam LRSL2_param;
    LRLRn(-x, y, -phi, LRSL2_param);
    std::vector<double> LRSL2_lengths = {-LRSL2_param.t, 0.5 * M_PI,
                                         -LRSL2_param.u, -LRSL2_param.v};
    if (LRSL2_param.flag &&
        !SetRSP(LRSL2_lengths, "LRSL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRSL2_param";
      return false;
    }

    RSPParam LRSL3_param;
    LRLRn(x, -y, -phi, LRSL3_param);
    std::vector<double> LRSL3_lengths = {LRSL3_param.t, -0.5 * M_PI,
                                         LRSL3_param.u, LRSL3_param.v};
    if (LRSL3_param.flag &&
        !SetRSP(LRSL3_lengths, "RLSR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRSL3_param";
      return false;
    }

    RSPParam LRSL4_param;
    LRLRn(-x, -y, phi, LRSL4_param);
    std::vector<double> LRSL4_lengths = {-LRSL4_param.t, -0.5 * M_PI,
                                         -LRSL4_param.u, -LRSL4_param.v};
    if (LRSL4_param.flag &&
        !SetRSP(LRSL4_lengths, "RLSR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRSL4_param";
      return false;
    }

    RSPParam LRSR1_param;
    LRLRp(x, y, phi, LRSR1_param);
    std::vector<double> LRSR1_lengths = {LRSR1_param.t, -0.5 * M_PI,
                                         LRSR1_param.u, LRSR1_param.v};
    if (LRSR1_param.flag &&
        !SetRSP(LRSR1_lengths, "LRSR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRSR1_param";
      return false;
    }

    RSPParam LRSR2_param;
    LRLRp(-x, y, -phi, LRSR2_param);
    std::vector<double> LRSR2_lengths = {-LRSR2_param.t, 0.5 * M_PI,
                                         -LRSR2_param.u, -LRSR2_param.v};
    if (LRSR2_param.flag &&
        !SetRSP(LRSR2_lengths, "LRSR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRSR2_param";
      return false;
    }

    RSPParam LRSR3_param;
    LRLRp(x, -y, -phi, LRSR3_param);
    std::vector<double> LRSR3_lengths = {LRSR3_param.t, -0.5 * M_PI,
                                         LRSR3_param.u, LRSR3_param.v};
    if (LRSR3_param.flag &&
        !SetRSP(LRSR3_lengths, "RLSL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRSR3_param";
      return false;
    }

    RSPParam LRSR4_param;
    LRLRp(-x, -y, phi, LRSR4_param);
    std::vector<double> LRSR4_lengths = {-LRSR4_param.t, 0.5 * M_PI,
                                         -LRSR4_param.u, -LRSR4_param.v};
    if (LRSR4_param.flag &&
        !SetRSP(LRSR4_lengths, "RLSL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRSR4_param";
      return false;
    }

    // backward
    double xb = x * std::cos(phi) + y * std::sin(phi);
    double yb = x * std::sin(phi) - y * std::cos(phi);

    RSPParam LRSL5_param;
    LRLRn(xb, yb, phi, LRSL5_param);
    std::vector<double> LRSL5_lengths = {LRSL5_param.v, LRSL5_param.u,
                                         -0.5 * M_PI, LRSL5_param.t};
    if (LRSL5_param.flag &&
        !SetRSP(LRSL5_lengths, "LSRL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRLRn_param";
      return false;
    }

    RSPParam LRSL6_param;
    LRLRn(-xb, yb, -phi, LRSL6_param);
    std::vector<double> LRSL6_lengths = {-LRSL6_param.v, -LRSL6_param.u,
                                         0.5 * M_PI, -LRSL6_param.t};
    if (LRSL6_param.flag &&
        !SetRSP(LRSL6_lengths, "LSRL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRSL6_param";
      return false;
    }

    RSPParam LRSL7_param;
    LRLRn(xb, -yb, -phi, LRSL7_param);
    std::vector<double> LRSL7_lengths = {LRSL7_param.v, LRSL7_param.u,
                                         -0.5 * M_PI, LRSL7_param.t};
    if (LRSL7_param.flag &&
        !SetRSP(LRSL7_lengths, "RSLR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRSL7_param";
      return false;
    }

    RSPParam LRSL8_param;
    LRLRn(-xb, -yb, phi, LRSL8_param);
    std::vector<double> LRSL8_lengths = {-LRSL8_param.v, -LRSL8_param.u,
                                         0.5 * M_PI, -LRSL8_param.t};
    if (LRSL8_param.flag &&
        !SetRSP(LRSL8_lengths, "RSLR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRSL8_param";
      return false;
    }

    RSPParam LRSR5_param;
    LRLRp(xb, yb, phi, LRSR5_param);
    std::vector<double> LRSR5_lengths = {LRSR5_param.v, LRSR5_param.u,
                                         -0.5 * M_PI, LRSR5_param.t};
    if (LRSR5_param.flag &&
        !SetRSP(LRSR5_lengths, "RSRL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRSR5_param";
      return false;
    }

    RSPParam LRSR6_param;
    LRLRp(-xb, yb, -phi, LRSR6_param);
    std::vector<double> LRSR6_lengths = {-LRSR6_param.v, -LRSR6_param.u,
                                         0.5 * M_PI, -LRSR6_param.t};
    if (LRSR6_param.flag &&
        !SetRSP(LRSR6_lengths, "RSRL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRSR6_param";
      return false;
    }

    RSPParam LRSR7_param;
    LRLRp(xb, -yb, -phi, LRSR7_param);
    std::vector<double> LRSR7_lengths = {LRSR7_param.v, LRSR7_param.u,
                                         -0.5 * M_PI, LRSR7_param.t};
    if (LRSR7_param.flag &&
        !SetRSP(LRSR7_lengths, "LSLR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRSR7_param";
      return false;
    }

    RSPParam LRSR8_param;
    LRLRp(-xb, -yb, phi, LRSR8_param);
    std::vector<double> LRSR8_lengths = {-LRSR8_param.v, -LRSR8_param.u,
                                         0.5 * M_PI, -LRSR8_param.t};
    if (LRSR8_param.flag &&
        !SetRSP(LRSR8_lengths, "LSLR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRSR8_param";
      return false;
    }
    return true;
  }  // CCSC

  bool CCSCC(const double x, const double y, const double phi,
             std::vector<ReedSheppPath>& all_possible_paths) {
    RSPParam LRSLR1_param;
    LRSLR(x, y, phi, LRSLR1_param);
    std::vector<double> LRSLR1_lengths = {LRSLR1_param.t, -0.5 * M_PI,
                                          LRSLR1_param.u, -0.5 * M_PI,
                                          LRSLR1_param.v};
    if (LRSLR1_param.flag &&
        !SetRSP(LRSLR1_lengths, "LRSLR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRSLR1_param";
      return false;
    }

    RSPParam LRSLR2_param;
    LRSLR(-x, y, -phi, LRSLR2_param);
    std::vector<double> LRSLR2_lengths = {-LRSLR2_param.t, 0.5 * M_PI,
                                          -LRSLR2_param.u, 0.5 * M_PI,
                                          -LRSLR2_param.v};
    if (LRSLR2_param.flag &&
        !SetRSP(LRSLR2_lengths, "LRSLR", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRSLR2_param";
      return false;
    }

    RSPParam LRSLR3_param;
    LRSLR(x, -y, -phi, LRSLR3_param);
    std::vector<double> LRSLR3_lengths = {LRSLR3_param.t, -0.5 * M_PI,
                                          LRSLR3_param.u, -0.5 * M_PI,
                                          LRSLR3_param.v};
    if (LRSLR3_param.flag &&
        !SetRSP(LRSLR3_lengths, "RLSRL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRSLR3_param";
      return false;
    }

    RSPParam LRSLR4_param;
    LRSLR(-x, -y, phi, LRSLR4_param);
    std::vector<double> LRSLR4_lengths = {-LRSLR4_param.t, 0.5 * M_PI,
                                          -LRSLR4_param.u, 0.5 * M_PI,
                                          -LRSLR4_param.v};
    if (LRSLR4_param.flag &&
        !SetRSP(LRSLR4_lengths, "RLSRL", all_possible_paths)) {
      ADEBUG << "Fail at SetRSP with LRSLR4_param";
      return false;
    }
    return true;
  }  // CCSCC

  // different options for different combination of motion primitives
  void LSL(const double x, const double y, const double phi, RSPParam& param) {
    auto [rho, theta] = common::math::Cartesian2Polar(x - std::sin(phi),
                                                      y - 1.0 + std::cos(phi));
    double u = rho;
    double t = theta;
    double v = 0.0;
    if (t >= 0.0) {
      v = common::math::Normalizeheadingangle(phi - t);
      if (v >= 0.0) {
        param.flag = true;
        param.u = u;
        param.t = t;
        param.v = v;
      }
    }
  }  // LSL

  void LSR(const double x, const double y, const double phi, RSPParam& param) {
    auto [rho, theta] = common::math::Cartesian2Polar(x + std::sin(phi),
                                                      y - 1.0 - std::cos(phi));
    double u1 = rho * rho;
    double t1 = theta;
    double u = 0.0;
    double theta_ = 0.0;
    double t = 0.0;
    double v = 0.0;
    if (u1 >= 4.0) {
      u = std::sqrt(u1 - 4.0);
      theta_ = std::atan2(2.0, u);
      t = common::math::Normalizeheadingangle(t1 + theta_);
      v = common::math::Normalizeheadingangle(t - phi);
      if (t >= 0.0 && v >= 0.0) {
        param.flag = true;
        param.u = u;
        param.t = t;
        param.v = v;
      }
    }
  }  // LSR

  // TODO: return RSPParam
  void LRL(const double x, const double y, const double phi, RSPParam& param) {
    auto [rho, theta] = common::math::Cartesian2Polar(x - std::sin(phi),
                                                      y - 1.0 + std::cos(phi));
    double u1 = rho;
    double t1 = theta;
    double u = 0.0;
    double t = 0.0;
    double v = 0.0;
    if (u1 <= 4.0) {
      u = -2.0 * std::asin(0.25 * u1);
      t = common::math::Normalizeheadingangle(t1 + 0.5 * u + M_PI);
      v = common::math::Normalizeheadingangle(phi - t + u);
      if (t >= 0.0 && u <= 0.0) {
        param.flag = true;
        param.u = u;
        param.t = t;
        param.v = v;
      }
    }
  }  // LRL

  void SLS(const double x, const double y, const double phi, RSPParam& param) {
    double phi_mod = common::math::Normalizeheadingangle(phi);
    double xd = 0.0;
    double u = 0.0;
    double t = 0.0;
    double v = 0.0;
    double epsilon = 1e-1;
    if (y > 0.0 && phi_mod > epsilon && phi_mod < M_PI) {
      xd = -y / std::tan(phi_mod) + x;
      t = xd - std::tan(phi_mod / 2.0);
      u = phi_mod;
      v = std::sqrt((x - xd) * (x - xd) + y * y) - std::tan(phi_mod / 2.0);
      param.flag = true;
      param.u = u;
      param.t = t;
      param.v = v;
    } else if (y < 0.0 && phi_mod > epsilon && phi_mod < M_PI) {
      xd = -y / std::tan(phi_mod) + x;
      t = xd - std::tan(phi_mod / 2.0);
      u = phi_mod;
      v = -std::sqrt((x - xd) * (x - xd) + y * y) - std::tan(phi_mod / 2.0);
      param.flag = true;
      param.u = u;
      param.t = t;
      param.v = v;
    }
  }  // SLS

  void LRLRn(const double x, const double y, const double phi,
             RSPParam& param) {
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));
    double u = 0.0;
    if (rho <= 1.0 && rho >= 0.0) {
      u = std::acos(rho);
      if (u >= 0 && u <= 0.5 * M_PI) {
        double tau = 0.0;
        double omega = 0.0;
        calc_tau_omega(u, -u, xi, eta, phi, tau, omega);
        if (tau >= 0.0 && omega <= 0.0) {
          param.flag = true;
          param.u = u;
          param.t = tau;
          param.v = omega;
        }
      }
    }
  }  // LRLRn

  void LRLRp(const double x, const double y, const double phi,
             RSPParam& param) {
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    double rho = (20.0 - xi * xi - eta * eta) / 16.0;
    double u = 0.0;
    if (rho <= 1.0 && rho >= 0.0) {
      u = -std::acos(rho);
      if (u >= 0 && u <= 0.5 * M_PI) {
        calc_tau_omega(u, u, xi, eta, phi, tau, omega);
        if (tau >= 0.0 && omega >= 0.0) {
          param.flag = true;
          param.u = u;
          param.t = tau;
          param.v = omega;
        }
      }
    }
  }  // LRLRp

  void LRSR(const double x, const double y, const double phi, RSPParam& param) {
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    auto [rho, theta] = common::math::Cartesian2Polar(-eta, xi);
    double t = 0.0;
    double u = 0.0;
    double v = 0.0;
    if (rho >= 2.0) {
      t = theta;
      u = 2.0 - rho;
      v = common::math::Normalizeheadingangle(t + 0.5 * M_PI - phi);
      if (t >= 0.0 && u <= 0.0 && v <= 0.0) {
        param.flag = true;
        param.u = u;
        param.t = t;
        param.v = v;
      }
    }
  }  // LRSR

  void LRSL(const double x, const double y, const double phi, RSPParam& param) {
    double xi = x - std::sin(phi);
    double eta = y - 1.0 + std::cos(phi);
    auto [rho, theta] = common::math::Cartesian2Polar(xi, eta);
    double r = 0.0;
    double t = 0.0;
    double u = 0.0;
    double v = 0.0;

    if (rho >= 2.0) {
      r = std::sqrt(rho * rho - 4.0);
      u = 2.0 - r;
      t = common::math::Normalizeheadingangle(theta + std::atan2(r, -2.0));
      v = common::math::Normalizeheadingangle(phi - 0.5 * M_PI - t);
      if (t >= 0.0 && u <= 0.0 && v <= 0.0) {
        param.flag = true;
        param.u = u;
        param.t = t;
        param.v = v;
      }
    }
  }  // LRSL

  void LRSLR(const double x, const double y, const double phi,
             RSPParam& param) {
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    auto [rho, theta] = common::math::Cartesian2Polar(xi, eta);
    double t = 0.0;
    double u = 0.0;
    double v = 0.0;
    if (rho >= 2.0) {
      u = 4.0 - std::sqrt(rho * rho - 4.0);
      if (u <= 0.0) {
        t = common::math::Normalizeheadingangle(std::atan2(
            (4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
        v = common::math::Normalizeheadingangle(t - phi);

        if (t >= 0.0 && v >= 0.0) {
          param.flag = true;
          param.u = u;
          param.t = t;
          param.v = v;
        }
      }
    }
  }  // LRSLR

  void calc_tau_omega(const double u, const double v, const double xi,
                      const double eta, const double phi, double& tau,
                      double& omega) {
    double delta = common::math::Normalizeheadingangle(u - v);
    double A = std::sin(u) - std::sin(delta);
    double B = std::cos(u) - std::cos(delta) - 1.0;

    double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
    double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
    tau = 0.0;
    if (t2 < 0) {
      tau = common::math::Normalizeheadingangle(t1 + M_PI);
    } else {
      tau = common::math::Normalizeheadingangle(t1);
    }
    omega = common::math::Normalizeheadingangle(tau - u + v - phi);
  }  // calc_tau_omega

 protected:
  VehicleParam _vehicle_param;
  PlannerOpenSpaceConfig planner_open_space_config_;
};  // class ReedShepp

}  // namespace ASV::planning
