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
./data_reader_node
./ground_segmentation_node
```

### Launch Visualiser
```
rviz2 -d config/pointcloud.rviz
```