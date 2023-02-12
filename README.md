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
```
./launch.sh
```

### Launch Visualiser
Inside of main folder:
```
rviz2 -d rviz2_config/rviz2_config.rviz 
```
### Processing Examples
![image1](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/cloud_capture.png)
![image2](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/ground_segmentation.png)
![image3](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/euclidean_clustering.png)
![image4](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/polygonization.png)
