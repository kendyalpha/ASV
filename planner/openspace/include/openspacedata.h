/*
***********************************************************************
* openspacedata.h:
* constant data using in open space path planning
*
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _OPENSPACE_DATA_H_
#define _OPENSPACE_DATA_H_

#include <cmath>

namespace ASV {
namespace planning {

// _________________
// GENERAL CONSTANTS

/// [#] --- Limits the maximum search depth of the algorithm, possibly
/// terminating without the solution
static const int iterations = 30000;
/// [m] --- Uniformly adds a padding around the vehicle
static const double bloating = 0;
/// [m] --- The width of the vehicle
static const double width = 1.75 + 2 * bloating;
/// [m] --- The length of the vehicle
static const double length = 2.65 + 2 * bloating;
/// [m] --- The minimum turning radius of the vehicle
static const float r = 6;
/// # of discretizations in heading
constexpr int N_HEADINGS = 72;
/// The discretization value of the heading (goal condition)
constexpr double deltaHeadingDeg = 360.0 / static_cast<double>(N_HEADINGS);
/// The discretization value of heading (goal condition)
constexpr double DELTA_HEADING_RAD = 2 * M_PI / static_cast<double>(N_HEADINGS);
/// [m] --- The cell size of the 2D grid of the world
static const float cellSize = 1;

/*!
  \brief [m] --- The tie breaker breaks ties between nodes expanded in the same
  cell


  As the cost-so-far are bigger than the cost-to-come it is reasonbale to
  believe that the algorithm would prefer the predecessor rather than the
  successor. This would lead to the fact that the successor would never be
  placed and the the one cell could only expand one node. The tieBreaker
  artificially increases the cost of the predecessor to allow the successor
  being placed in the same cell.
*/
static const float tieBreaker = 0.01;

// ___________________
// HEURISTIC CONSTANTS

/// [#] --- A factor to ensure admissibility of the holonomic with obstacles
/// heuristic
constexpr double factor2D = sqrt(5) / sqrt(2) + 1;
/// [#] --- A movement cost penalty for turning (choosing non straight motion
/// primitives)
constexpr double PENALTY_TURNING = 1.05;
/// [#] --- A movement cost penalty for reversing (choosing motion primitives >
/// 2)
constexpr double PENALTY_REVERSING = 2.0;
/// [#] --- A movement cost penalty for change of direction (changing from
/// primitives < 3 to primitives > 2)
constexpr double PENALTY_COD = 2.0;
/// [m] --- The distance to the goal when the analytical solution (Dubin's shot)
/// first triggers
constexpr double DUBINS_SHOT_DISTANCE = 100;
/// [m] --- The step size for the analytical solution (Dubin's shot) primarily
/// relevant for collision checking
constexpr double dubinsStepSize = 1;

}  // namespace planning
}  // namespace ASV
#endif /* _OPENSPACE_DATA_H_ */