/* This is an auto-generated file made from optimization engine: https://crates.io/crates/optimization_engine */

#pragma once



#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

/// Number of decision variables
static const uintptr_t ROSENBROCK_ROS2_NUM_DECISION_VARIABLES = 5;

/// Number of parameters
static const uintptr_t ROSENBROCK_ROS2_NUM_PARAMETERS = 2;

/// Number of parameters associated with augmented Lagrangian
static const uintptr_t ROSENBROCK_ROS2_N1 = 0;

/// Number of penalty constraints
static const uintptr_t ROSENBROCK_ROS2_N2 = 2;

/// rosenbrock_ros2 version of ExitStatus
/// Structure: `rosenbrock_ros2ExitStatus`
enum class rosenbrock_ros2ExitStatus {
  /// The algorithm has converged
  ///
  /// All termination criteria are satisfied and the algorithm
  /// converged within the available time and number of iterations
  rosenbrock_ros2Converged,
  /// Failed to converge because the maximum number of iterations was reached
  rosenbrock_ros2NotConvergedIterations,
  /// Failed to converge because the maximum execution time was reached
  rosenbrock_ros2NotConvergedOutOfTime,
  /// If the gradient or cost function cannot be evaluated internally
  rosenbrock_ros2NotConvergedCost,
  /// Computation failed and NaN/Infinite value was obtained
  rosenbrock_ros2NotConvergedNotFiniteComputation,
};

/// Solver cache (structure `rosenbrock_ros2Cache`)
///
struct rosenbrock_ros2Cache;

/// rosenbrock_ros2 version of AlmOptimizerStatus
/// Structure: `rosenbrock_ros2SolverStatus`
///
struct rosenbrock_ros2SolverStatus {
  /// Exit status
  rosenbrock_ros2ExitStatus exit_status;
  /// Number of outer iterations
  unsigned long num_outer_iterations;
  /// Total number of inner iterations
  ///
  /// This is the sum of the numbers of iterations of
  /// inner solvers
  unsigned long num_inner_iterations;
  /// Norm of the fixed-point residual of the the problem
  double last_problem_norm_fpr;
  /// Total solve time
  unsigned long long solve_time_ns;
  /// Penalty value
  double penalty;
  /// Norm of delta y divided by the penalty parameter
  double delta_y_norm_over_c;
  /// Norm of F2(u)
  double f2_norm;
  /// Value of cost function at solution
  double cost;
  /// Lagrange multipliers
  const double *lagrange;
};

extern "C" {

/// Allocate memory and setup the solver
rosenbrock_ros2Cache *rosenbrock_ros2_new();

/// Solve the parametric optimization problem for a given parameter
///
///
/// # Arguments:
/// - `instance`: re-useable instance of AlmCache, which should be created using
///   `rosenbrock_ros2_new` (and should be destroyed once it is not
///   needed using `rosenbrock_ros2_free`
/// - `u`: (on entry) initial guess of solution, (on exit) solution
///   (length: `ROSENBROCK_ROS2_NUM_DECISION_VARIABLES`)
/// - `params`:  static parameters of the optimizer
///   (length: `ROSENBROCK_ROS2_NUM_PARAMETERS`)
/// - `y0`: Initial guess of Lagrange multipliers (if `0`, the default will
///   be used; length: `ROSENBROCK_ROS2_N1`)
/// - `c0`: Initial penalty parameter (provide `0` to use the default initial
///   penalty parameter
///
///
/// # Returns:
/// Instance of `rosenbrock_ros2SolverStatus`, with the solver status
/// (e.g., number of inner/outer iterations, measures of accuracy, solver time,
/// and the array of Lagrange multipliers at the solution).
///
///
///
/// # Safety
/// All arguments must have been properly initialised
rosenbrock_ros2SolverStatus rosenbrock_ros2_solve(rosenbrock_ros2Cache *instance,
                                                  double *u,
                                                  const double *params,
                                                  const double *y0,
                                                  const double *c0);

/// Deallocate the solver's memory, which has been previously allocated
/// using `rosenbrock_ros2_new`
///
///
/// # Safety
/// All arguments must have been properly initialised
void rosenbrock_ros2_free(rosenbrock_ros2Cache *instance);

} // extern "C"
