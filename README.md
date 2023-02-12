# LiDAR-Processing
Lidar processing pipeline based on ROS2 Humble that consists of:
* Reading point cloud data from `data/` folder and publishing on `pointcloud` topic at some predefined constant frequency
* Subscribing on `pointcloud` topic and performing several processing steps, such as 
    * Ground segmentation
    * Obstacle clustering
    * Obstacle cluster simplification
* Processing node publishes visualisable results on `ground_pointcloud`, `obstacle_pointcloud` and `convex_polygonization` topics.

### Build
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release .. 
cmake --build . --target all --config Release
```

### Build and Launch Nodes
```
./launch.sh
```

### Launch Visualiser
```
rviz2 -d rviz2_config/rviz2_config.rviz 
```
### Raw Point Cloud Colored by Intensity
![image1](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/cloud_capture.png)

### Ground Segmentation
![image2](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/ground_segmentation.png)

### Obstacle Point Cloud Clusters
![image3](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/euclidean_clustering.png)

### Convex Simplification of Obstacle Point Cloud Clusters
![image4](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/polygonization.png)
