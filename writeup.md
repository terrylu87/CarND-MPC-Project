---

# MPC Project

## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### The Model
#### 1.Student describes their model in detail. This includes the state, actuators and update equations.
The model have 6 variables: x, y, velocity, orientation, cte(cross-track error), epsi(orientation error)

And there are two actuators: delta (the steering angle actuator), alpha (the acceleration actuator)

The update equations are:

* x = x + v * cos(psi) * dt

* y = y + v * sin(psi) * dt

* psi = psi - v * delta * dt / Lf

* v = v + a * dt

I use the cost fuctions described in class with additional factors to address the importance of the specific cost.

My goal is to finish the lap as fast as possible without crashing itself. After watching the car crash again and again in the simulator, I give very large factors (4096) to the cost of cte and epsi, to make sure the car can safely running in the lap. After a few iterations, the car was able to run safely with reference speed equals to 120. 
The car would break very hard before enter the coner, maybe too hard. So I adjust the factors to 3072 and 2048. Those factors may not be the fastest choice, but are the best balance between speed and safe.
```cpp
    unsigned int t;
    for (t = 0; t < N; t++) {
      fg[0] += 3072*CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 2048*CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += 2*CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (t = 0; t < N - 1; t++) {
      fg[0] += 256*CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 2*CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (t = 0; t < N - 2; t++) {
      fg[0] += 256*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```

### Timestep Length and Elapsed Duration (N & dt)
#### 1.Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.
Since the car is running at over 90 mph, so the car can run pretty far distance in 1 second. So I decide to predict the trajectory for 1 second future. Assign 0.1 to dt will produce stable perfomance. So the N is 1 / 0.1 = 10.

### Polynomial Fitting and MPC Preprocessing
#### 1.A polynomial is fitted to waypoints.
The waypoints is fitted to a 3 order polynomial.

#### 2.If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.
The global waypoints is transformed into car coordinate before fitting the function.

### Model Predictive Control with Latency
#### 1.The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.
Before pass the states vector to the mpc.Solve function. The effect of 100 ms delay is caculated based on the updating equations.

---