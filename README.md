# LiDAR-Processing
Lidar processing pipeline based on ROS2 Humble

### Build
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

### Run Nodes
```
./data_reader_node
rviz2 -d config/pointcloud.rviz
```