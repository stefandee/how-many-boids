# How Many Boids Can You Simulate in Unity?

This project aims to simulate [boids](https://en.wikipedia.org/wiki/Boids) using Unity features like the scene graph, parallel job system (w/wo Burst) and compute shaders. The goal was to learn the job system and compute shaders.

The boids algorithms used are slightly different between each method: some use global center mass and velocity, while others use local center mass and velocity.

Technically, all methods except the compute shader, are scene graph based, as they create a GameObject for each boid. What is named as "scene graph" is a simulation in the Unity main thread.

## Requirements

Requires Unity 6, but code should work in previous versions as well.

## Media
![Boids simulated in a compute shader](.media/boids-compute-shader.gif)

## TODO

Profile and do micro-optimizations where appropriate (especially the compute shader).

Implement a spatial locator algorithm (oct tree) to help with neighbor detection.

## License

https://opensource.org/licenses/MIT

