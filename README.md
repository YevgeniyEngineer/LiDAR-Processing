# LiDAR-Processing
Lidar processing pipeline based on ROS2 Humble that consists of:
* Reading point cloud data from `data/` folder and publishing on `pointcloud` topic at some predefined constant frequency
* Subscribing on `pointcloud` topic and performing several processing steps, such as 
    * Ground segmentation
    * Obstacle clustering
    * Obstacle cluster simplification
* Processing node publishes visualisable results on `ground_pointcloud`, `obstacle_pointcloud` and `convex_polygonization` topics.


### Build and Launch Nodes

Build: `./build.sh`

Launch processing nodes and the visualizer: `./launch.sh`

### Point Cloud Colored by Intensity
![image1](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/cloud_capture.png)

### Ground Segmentation
![image2](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/ground_segmentation.png)

### Obstacle Point Cloud Clusters
![image3](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/euclidean_clustering.png)

### Convex Simplification of Obstacle Point Cloud Clusters
![image4](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/polygonization.png)
