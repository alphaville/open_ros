float64[]    solution              # optimizer
bool         has_converged         # whether the algorithm has converged
uint8        inner_iterations
uint16       outer_iterations      # number of outer iterations
uint8        status                # status code
float64      norm_fpr              # norm of FPR of last inner problem
float64      penalty
float64[]    lagrange_multipliers
# etc

