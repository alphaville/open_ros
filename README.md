# ROS + OpEn

Work in progress...

This will be incorporated into [OpEn][1] once completed.

### Instructions

Compile with `catkin_make`. 

Run with:

```
rosrun mpc_dummy mpc_dummy_talker 
```

Once you run the node, you can post your request to `mpc/open_parameters` as follows:

```
rostopic pub /mpc/open_parameters              \
  mpc_dummy/OptimizationParameters             \
  "{'parameter':[1.0,200.0],                   \
   'initial_guess':[0.5, 0.1, 0.3, 0.1, -0.6], \
   'initial_penalty': 100.0}" --once
```

The result will be announced on `/mpc/open_solution`:

```
rostopic echo /mpc/open_solution
```

[1]: https://alphaville.github.io/optimization-engine
