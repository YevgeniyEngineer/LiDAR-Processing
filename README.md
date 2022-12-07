# LiDAR-Processing
Lidar processing pipeline based on ROS2 Humble

### Build
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release .. 
cmake --build . --target all --config Release
```

### Launch Nodes
Inside of `/build` folder:
```
./data_reader_node
./ground_segmentation_node
./obstacle_clustering_node
```

### Launch Visualiser
Inside of main folder:
```
rviz2 -d config/pointcloud.rviz
```
### Processing Examples
![image1](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/cloud_capture.png)
![image2](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/ground_segmentation.png)
![image3](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/euclidean_clustering.png)
![image4](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/polygonization.png)
