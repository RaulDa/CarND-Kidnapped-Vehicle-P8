# **Kidnapped Vehicle**

---

**Kidnapped Vehicle Project**

The goals / steps of this project are the following:

* Implementation of a particle filter in C++.
* Localization of a vehicle within a determined accuracy.

[//]: # (Image References)

[image1]: ./output/Simulator.png "Simulator"

## Rubric Points

Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/747/view) individually and describe how I addressed each point in my implementation.  

---


### Algorithm structure and processing flow

The algorithm is divided in the following source code files located in `/src`:

`main.cpp` -> Main routine to read the sensor data and call the particle filter functions implemented in the `ParticleFilter` class.

`particle_filter.cpp` -> Particle initialization, prediction according with estimated velocity and yaw rate and update according with observations and map data via weight calculation and resampling.

Additionally the `helper_functions.h` header file is defined, where helper inline routines for Euclidean distance or error calculation are specified.

#### 1. Initialization

The `init` routine of the `ParticleFilter` class is called by `main` during the first iteration. It defines the number of particles to be implemented and performs the parameter initialization.

The number of particles determines the accuracy and execution velocity. A low number will estimate the position with low accuracy and a high number will make the execution too slow. It was found that `100` particles work well regarding these two factors, so this was the final chosen number.

For the particle coordinates initialization, the first GPS measurement is used after applying Gaussian noise. The selected standard deviations where `2m` for the `x` coordinate, `2m` for `y` and `0.2rad` for `theta` (values introduced directly inside `init`). Regarding the weights, they were initially set to `1`.

#### 2. Prediction

The `prediction` routine of the `ParticleFilter` class is called by `main` during all iterations. Here the estimated velocity and yaw rate are used together with the bicycle motion model equations to obtain the new predicted position for all particles. The step represents a practical implementation of the law of total probability.

A Gaussian noise of `0.3m` for the `x` coordinate, `0.3m` for `y` and `0.01rad` for `theta` is introduced (represents velocity and yaw rate noise).

In order to avoid a division by zero, it should be prevented a yaw rate value equal to zero.

#### 3. Weights update

In this case the update is performed by the function `updateWeights` of the `ParticleFilter` class, also called by `main` during all iterations. This step together with the resampling implements the Bayes' theorem. The functionality is divided in three steps: transformation of observations from vehicle to map coordinates, association of observations with map landmarks and application of the multivariate Gaussian probability density function.

The transformation from vehicle to map coordinates is done for each observation respect to each particle. The corresponding x, y and theta particle coordinates are used to perform both rotation and translation.

For the association, the distance from an observation to each map landmark is computed and the landmark with lower distance is associated with the observation. This is also done for all particles.

Finally, once all observations have an associated landmark, the multivariate Gaussian probabililty density function is applied. Here a measurement landmark noise is introduced (`0.3m` for the `x` coordinate, `0.3m` for `y`).

#### 4. Resampling

Finally, the resampling is performed by the function `resample` of the `ParticleFilter` class. Here the resampling wheel algorithm is applied and as a result a new set of particles (final posterior distribution) is generated in accordance with the weights. It is important to mention that a weight normalization is done as previous step to the resampling.

### 5. Accuracy and results

The particle filter achieve an approximate error of .115 for `x`, 0.095 for `y` and 0.03 for `theta` during the simulation, as shown in the [video](https://www.youtube.com/watch?v=J-JSZ5nrHpA).
