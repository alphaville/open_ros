/* This is an auto-generated file made from optimization engine: https://crates.io/crates/optimization_engine */

#pragma once



#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <new>

static const uintptr_t ROSENBROCK_N1 = 0;

static const uintptr_t ROSENBROCK_N2 = 2;

static const uintptr_t ROSENBROCK_NUM_DECISION_VARIABLES = 5;

static const uintptr_t ROSENBROCK_NUM_PARAMETERS = 2;

/// rosenbrock version of ExitStatus
/// Structure: `rosenbrockExitStatus`
enum class rosenbrockExitStatus {
  /// The algorithm has converged
  /// All termination criteria are satisfied and the algorithm
  /// converged within the available time and number of iterations
  rosenbrockConverged,
  /// Failed to converge because the maximum number of iterations was reached
  rosenbrockNotConvergedIterations,
  /// Failed to converge because the maximum execution time was reached
  rosenbrockNotConvergedOutOfTime,
  /// If the gradient or cost function cannot be evaluated internally
  rosenbrockNotConvergedCost,
  /// Computation failed and NaN/Infinite value was obtained
  rosenbrockNotConvergedNotFiniteComputation,
};

/// Solver cache (structure `rosenbrockCache`)
struct rosenbrockCache;

/// rosenbrock version of AlmOptimizerStatus
/// Structure: `rosenbrockSolverStatus`
struct rosenbrockSolverStatus {
  /// Exit status
  rosenbrockExitStatus exit_status;
  /// Number of outer iterations
  unsigned long num_outer_iterations;
  /// Total number of inner iterations
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
  /// Lagrange multipliers
  const double *lagrange;
};

extern "C" {

/// Deallocate the solver's memory, which has been previously allocated
/// using `rosenbrock_new`
void rosenbrock_free(rosenbrockCache *instance);

/// Allocate memory and setup the solver
rosenbrockCache *rosenbrock_new();

/// Solve the parametric optimization problem for a given parameter
/// .
/// .
/// Arguments:
/// - `instance`: re-useable instance of AlmCache, which should be created using
/// `rosenbrock_new` (and should be destroyed once it is not
/// needed using `rosenbrock_free`
/// - `u`: (on entry) initial guess of solution, (on exit) solution
/// (length: `ROSENBROCK_NUM_DECISION_VARIABLES`)
/// - `params`:  static parameters of the optimizer
/// (length: `ROSENBROCK_NUM_PARAMETERS`)
/// - `y0`: Initial guess of Lagrange multipliers (if `0`, the default will
/// be used; length: `ROSENBROCK_N1`)
/// - `c0`: Initial penalty parameter (provide `0` to use the default initial
/// penalty parameter
/// .
/// .
/// Returns:
/// Instance of `rosenbrockSolverStatus`, with the solver status
/// (e.g., number of inner/outer iterations, measures of accuracy, solver time,
/// and the array of Lagrange multipliers at the solution).
rosenbrockSolverStatus rosenbrock_solve(rosenbrockCache *instance,
                                        double *u,
                                        const double *params,
                                        const double *y0,
                                        const double *c0);

} // extern "C"
