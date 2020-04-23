# ROS + OpEn

Work in progress...

This will be incorporated into [OpEn][1] once completed.

### Instructions

Compile with `catkin_make`. 

Run with:

```
rosrun mpc_dummy mpc_dummy_optimizer
```

Once you run the node, you can post your request to `mpc_dummy/open_parameters` as follows:

```
rostopic pub /mpc_dummy/open_parameters        \
  mpc_dummy/OptimizationParameters             \
  "{'parameter':[1.0,200.0],                   \
   'initial_guess':[0.5, 0.1, 0.3, 0.1, -0.6], \
   'initial_penalty': 100.0}" --once
```

The result will be announced on `/mpc_dummy/open_solution`:

```
rostopic echo /mpc/open_solution
```

[1]: https://alphaville.github.io/optimization-engine
