# Mobile Robotics Project

## Probabilistic Sensor Fusion

This repository contains the entire timeline of the project and how we decided upon the factors and parameters for the project.

The requirements of the project as per the project instructions are:

<ins> Probabilistic Sensor Fusion </ins>
• Implement Kalman filter for fusing multiple sensor modalities
• Combine odometry with GPS or landmark observations
• Evaluate fusion performance under different sensor reliabilities
• Study the impact of sensor correlation on estimation accuracy

---

In order to achieve the Kalman sensor fusion and test its efficiency, we decided to break down the problem into smaller subsections:

1. First we use the odometery reading to get the position of the robot
2. Second we use a GPS to get the position of the robot as it moves - we predict this will already provide a lower error as it is more accurate when compared to an odometer.
3. We then implement the Kalman filter with 2 sensors: currently working on implementing fusing GPS with LiDAR but might consider GPS with Beam Model fusion depending on complexity as the project moves on.

### Error comparisons

To compare the performance of each sensor and whether the kalman filter improves the accuracy or not, we needed to have a simgle point of comparison between each of the metrics.

### Odometry readings

This was the first part subproblem which was tackled for the project.
