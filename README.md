# LiDAR Processing Pipeline

## Overview
The LiDAR Processing Pipeline showcases classical point cloud data processing techniques, leveraging libraries such as PCL and ROS2 (Humble). While PCL data structures are extensively utilized throughout the pipeline stages, efforts have been made to minimize reliance on third-party algorithms by developing most of the required data structures in-house. However, a complex algorithm like Delaunay triangulation is integrated using the Delaunator library, which has been modified to enable the construction of a concave hull from the triangulation. This pipeline efficiently performs ground segmentation, obstacle clustering, and polygonization, and it presents the results through real-time visualization in RViz2. The Dataloader node publishes `sensor_msgs::msg::PointCloud2` data at regular intervals (10 Hz), thereby constraining the processing pipeline to a 100 ms processing window to simulate real-time operation.

![pipeline](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/example_pipeline.gif)

![complete_pipeline](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/complete_pipeline.gif)

## Pipeline Stages

### Ground Segmentation

The algorithm is based on the research paper ["Fast Segmentation of 3D Point Clouds: A Paradigm on LiDAR Data for Autonomous Vehicle Applications"](https://www.researchgate.net/publication/318325507_Fast_Segmentation_of_3D_Point_Clouds_A_Paradigm_on_LiDAR_Data_for_Autonomous_Vehicle_Applications). It begins by subdividing the point cloud into multiple segments along the x-direction. Initially, seed points are randomly chosen to estimate the ground plane within each segment. This process involves an iterative procedure where the ground plane fit is refined within each segment through successive iterations. 

![segmentation](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/ground_segmentation.png)

### Obstacle Clustering

To perform clustering on segmented LiDAR points I used the adaptation of [Fast Euclidean Clustering](https://arxiv.org/abs/2208.07678). This is a hierarchical clustering algorithm designed for efficient clustering of cartesian point cloud data. The algorithm starts by partitioning the dataset into smaller subsets. These subsets are then progressively merged based on a predefined threshold distance, which determines if two subsets are close enough to be considered part of the same cluster.

![clustering](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/euclidean_clustering.png)

### Polygonization

The obstacle cloud clusters are post-processed further, finding a simplified polygon shapes around 3D point clusters. Polygonization is based on finding the concave hull simplification. The method relies on the efficient construction of concave polygons outlined in [Duckham et.al "Efficient generation of simple polygons for characterizing the shape of a set of points in the plane"](https://www.sciencedirect.com/science/article/abs/pii/S0031320308001180). The concave algorithm begins with Delaunay triangulation, using it as a foundational step for refining the perimeter of the convex outline to better conform to the actual spread of points, preserving the 2.5D flattened cluster shape.

![polygonization](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/concave_polygonization.png)

## Setup

Dependencies:
* ROS2 (Humble or compatible versions of ROS2)
* PCL
* Eigen3 
* TBB

To build the project, navigate to the project directory and run `./build.sh` to compile.

Run `./launch.sh` to start the processing nodes and visualization tools.

## Usage

Data from the `data/` directory is read and published on the `pointcloud` topic at a frequency of 10 Hz in sensor frame (without applying coordinate transforms). The processing node subscribes to the topic and performs processing steps within the same thread, publishing visualizable messages on several topics, including `ground_pointcloud`, `obstacle_pointcloud`, `clustered_pointcloud`, and `polygonization`.
