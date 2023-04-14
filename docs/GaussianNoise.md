# Gaussian noise

Robotec GPU Lidar (RGL) allows the introduction of Gaussian noise, which models the naturally present noise of the physical devices.

The following formula describes Gaussian noise:

![Gausian_noise_formula](image/gaussian_noise.gif)

It is parametrized by these two values:
- μ - mean value
- σ - standard deviation

## Supported noise types
    
The following table presents all kinds of Gaussian noise available in RGL concisely.
The subsequent chapters describe them in greater detail.

| Noise type | Visualization |
|------------|---------------|
| **Ray-based angular noise:** <li> Rotate rays around the selected axis of the lidar <br> <li> This kind of noise intends to model the noise in the lidar's rotational speed |![Angular ray 2](image/Angular_ray_2.png)  |
| **Hitpoint-based angular noise** <br> <li> Rotate hitpoints around the selected axis of the lidar <br> <li> This is different from rotating rays! Think of the effect on hitpoints on surfaces near-parallel to the ray <br> <li> This kind of noise could model, e.g., an error in the reading position of the rotated laser or sensor | ![Angular hitpoint 3](image/Angular_hitpoint_3.png) |
| **Distance noise** <br> <li> This kind of noise can model, e.g., an error in measuring the time-of-flight | ![Distance noise 3](image/Distance_3.png) |

## Ray-based angular noise

This type of noise applies angular noise to the rays **before raycasting**.
When this type of noise is applied, points stay on the obstacle surface

| Step | Visualization |
|--------------------------|---------------------------------------------|
|Rays are provided by the user via RGL API| ![Angular ray](image/Angular_ray_1.png)|
|Angular noise is applied to rays | ![Angular ray](image/Angular_ray_2.png)|
|Raytracing is performed using noised rays. Hitpoints are produced | ![Angular ray](image/Angular_ray_3.png)|
|Hitpoints with ray-based noise are sent for further processing| ![Angular ray](image/Angular_ray_4.png)|

### Usage

Create `GaussianNoiseAngularRaysNode` using API call `rgl_node_gaussian_noise_angular_ray`. Next, add this node to the RGL pipeline before `RaytraceNode`.

| Parameter name | Type | Description |
|----------------|------|-------------|
| `mean` | `float` | Angular noise mean in radians |
| `st_dev` | `float` | Angular noise standard deviation in radians |
| `axis` | `rgl_axis_t` | Axis on which angular noise will be performed |

## Hitpoint-based angular noise

This type of noise adds angular noise to already computed hitpoints.
In other words, it is a rotation of each hitpoint around LiDAR's origin.
When this type of noise is applied, points may no longer be on the obstacle surface

| Step | Visualization |
|--------------------------|---------------------------------------------|
|Rays are provided by the user via RGL API| ![Angular hitpoint](image/Angular_hitpoint_1.png)|
|Raycasting is performed using user-provided rays. Hitpoints are produced | ![Angular hitpoint](image/Angular_hitpoint_2.png)|
|Hitpoints are rotated by the noise value| ![Angular hitpoint](image/Angular_hitpoint_3.png)|
|Hitpoints with hitpoint-based noise are sent for further processing| ![Angular hitpoint](image/Angular_hitpoint_4.png)|

### Usage

Create `GaussianNoiseAngularHitpointNode` using API call `rgl_node_gaussian_noise_angular_hitpoint`. Next, add this node to the RGL pipeline right after `RaytraceNode`.

| Parameter name | Type | Description |
|----------------|------|-------------|
| `mean` | `float` | Angular noise mean in radians |
| `st_dev` | `float` | Angular noise standard deviation in radians |
| `axis` | `rgl_axis_t` | Axis on which angular noise will be performed |

## Distance noise

This noise changes the distance between the hitpoint and the lidar's origin.

Standard deviation can depend (increase linearly) on the point's distance from the sensor origin.

The overall standard deviation of the distance error is computed as follows:

![distance sigma formula](image/distance_stdev.gif)

where

|symbol| description|
|------|------------|
|![sigma_distance](image/distance_noise_sigma_distance.gif)| computed distance standard deviation applied to hitpoints |
|![sigma_base](image/distance_noise_sigma_base.gif)| base value of distance standard deviation (configurable) |
|![d](image/distance_noise_d.gif)| current points distance from sensor origin |
|![sigma_slope](image/distance_noise_sigma_slope.gif)| rise of the standard deviation per 1 meter of distance (configurable) |

| Step | Visualization |
|--------------------------|---------------------------------------------|
|Rays are provided by the user via RGL API| ![Distance](image/Distance_1.png)|
|Raycasting is performed. Hitpoints are produced | ![Distance](image/Distance_2.png)|
|Hitpoints distance is changed by noise value| ![Distance](image/Distance_3.png)|
|Hitpoints with distance noise are sent for further processing| ![Distance](image/Distance_4.png)|

### Usage

Create `GaussianNoiseDistanceNode` using API call `rgl_node_gaussian_noise_distance`. Next, add this node to the RGL pipeline right after `RaytraceNode`.

| Parameter name | Type | Description |
|----------------|------|-------------|
| `mean` | `float` | Distance noise mean in meters |
| `st_dev_base` | `float` | Distance noise standard deviation base in meters. Represented as ![sigma_base](image/distance_noise_sigma_base.gif) in the [Distance noise](#distance-noise) chapter |
| `st_dev_rise_per_meter` | `float` | Distance noise standard deviation rise per meter. Represented as ![sigma_slope](image/distance_noise_sigma_slope.gif) in the [Distance noise](#distance-noise) chapter |
