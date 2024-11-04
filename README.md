# Hierarchical SLAM for Orchard 3D Reconstruction

### Author: Bernardo Lanza, Ph.D. | Robotics Engineer | AI and Computer Vision Specialist in Agriculture

---

## Overview

This project presents a **novel hierarchical SLAM (Simultaneous Localization and Mapping) algorithm**, designed to improve 3D reconstruction in agricultural environments, specifically orchards. The algorithm, developed for use with RGB-D cameras and optimized for low-cost, high-frequency point cloud data, addresses limitations of traditional SLAM in high-uncertainty, outdoor settings. Key innovations include hierarchical ICP (Iterative Closest Point) and customized fusion techniques that enable more robust, drift-resistant mapping for orchard monitoring and analysis.

This repository contains the **Python implementation** of the algorithm, including custom utility functions and hierarchical structures for merging point clouds. 

---

## Motivation

In modern precision agriculture, accurate 3D reconstruction of orchards supports critical tasks like plant health monitoring, canopy geometry assessment, and yield optimization. While photonic sensors like LiDAR offer high precision, they are costly and complex to operate in dynamic environments. **RGB-D cameras**, like the Azure Kinect, provide a cost-effective solution but introduce challenges due to lower resolution and accuracy. This algorithm leverages RGB-D cameras to provide real-time mapping with a unique hierarchical SLAM approach, making precision 3D reconstruction more accessible.

---

## Key Features

1. **Coupled Point Cloud Fusion**: The algorithm fuses point clouds in pairs, ensuring that each pair has similar dimensions. This method enables precise alignment and minimizes errors due to size discrepancies between the point clouds.

2. **Recursive Halving**: Following a hierarchical approach, the algorithm iteratively reduces the number of point clouds by recursively halving them. In each epoch, pairs of point clouds are fused, progressively building a final, comprehensive 3D model.

3. **Pre-Transformation of Source**: Before applying the ICP algorithm, the source point cloud is pre-transformed using a prior transformation (`Tprior`). This step positions the source over the target, optimizing the alignment by maximizing the common area between the two clouds.

4. **ICP Algorithm with 1-Frame Displacement**: The ICP (Iterative Closest Point) algorithm is applied with a 1-frame displacement to align the source to the target based on the current trajectory. This technique ensures that the transformations build cumulatively over successive epochs, reducing drift and enhancing stability.

5. **Trajectory-Based Transformation**: Transformation matrices are generated based on trajectory data from previous epochs. Each new transformation is layered upon prior transformations, maintaining continuity and accurately reflecting the sensor’s movement over time.

6. **Scalability and Adaptability**: The hierarchical structure enables easy scaling from small sections of an orchard to larger plots and is adaptable for different sensor configurations, including dual-sided RGB-D setups.


---

## Algorithm Workflow

1. **Point Cloud Preprocessing**: Raw point clouds are downsampled and isolated points are removed to improve processing efficiency.
   
2. **Hierarchical SLAM ICP**:
   - Successive point clouds are registered in pairs, using prior transformations to ensure alignment.
   - Transformation matrices are calculated and applied to each point cloud pair, merging them progressively across epochs.
   
3. **Final Fusion**: After multiple epochs, a final global point cloud is generated, representing the entire scanned environment in high detail.

4. **Performance Metrics**: The merged point cloud quality is assessed using metrics such as density, sphericity, and uncertainty estimation, based on known reference objects like foam spheres used during field trials.

---

## Repository Structure

- **main.py**: The primary script to run the hierarchical SLAM process, executing each step from data loading to final point cloud fusion.
- **fusion_utilities.py**: Contains essential utility functions for point cloud processing, including downsampling, ICP implementation, and visualization functions.
- **hierarchy_custom_function.py**: Implements the hierarchical SLAM core, including the custom ICP-based point cloud fusion logic.

---

## Usage

### Requirements

- Python 3.8+
- Libraries: `numpy`, `open3d`, `matplotlib`

### Running the Algorithm

1. Clone this repository and navigate to the project directory.
2. Place raw point clouds in the specified input directory.
3. Run `main.py`:
   ```bash
   python main.py
   
The output will be a fused point cloud saved in `output_double/fused.ply`.

## Visualizations

- Use the built-in plotting functions to view the 3D trajectory and assess registration accuracy.
- Trajectory plots are generated using `plot_trajectory_3d` to visualize sensor movement.

---

## Experimental Validation

The algorithm has been validated in an experimental orchard as part of a broader project on precision agriculture. Using low-cost RGB-D cameras and high-precision laser scanners, the method effectively reconstructed 3D models of orchard rows, capturing detailed plant features. The custom SLAM approach successfully minimized drift, even in challenging lighting conditions and high-vegetation areas, achieving comparable results to high-performance LiDAR systems.

---

## About the Author

Bernardo Lanza is a robotics engineer specializing in the integration of AI and computer vision in agriculture. With a Ph.D. in Mechatronics, his work combines deep expertise in measurement science, statistics, and embedded vision systems to create innovative solutions for precision agriculture.

This repository represents Bernardo's latest contribution to the field, enhancing the accessibility and effectiveness of orchard monitoring through cutting-edge SLAM technology.

For more information, please refer to Bernardo's [Portfolio](https://bernardolanza93.github.io/portfolio/) and [LinkedIn](https://www.linkedin.com/in/bernardo-lanza-554064163/).

---

## Acknowledgments

This project has been developed in collaboration with the **University of Brescia** and the **University of Lleida** as part of the DIGIFRUIT project, funded by the European Union NextGeneration EU and the Spanish Ministry of Science and Innovation.

---

## Demonstration

Here’s an animation showcasing the hierarchical SLAM algorithm in action:

![SLAM Algorithm Demonstration](assets/demo.gif)