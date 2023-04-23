# LiDAR-Processing
Lidar processing pipeline based on ROS2 Humble that consists of:
* Reading point cloud data from `data/` folder and publishing on `pointcloud` topic at some predefined constant frequency. To closely simulate real time conditions, 10Hz frequency is used in the current pipeline.
* Subscribing on `pointcloud` topic and performing several processing steps, such as 
    * Ground segmentation.
    * Obstacle clustering.
    * Obstacle cluster simplification.
* Publishing visualisable results on `ground_pointcloud`, `obstacle_pointcloud` and `convex_polygonization` topics.

### Build and Launch Nodes

Update submodules: `git pull & git submodule update --init --recursive`

Build: `./build.sh`

Launch processing nodes and the visualizer: `./launch.sh`

### Point Cloud Colored by Intensity
![image1](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/cloud_capture.png)

### Ground Segmentation

The algorithm is based on the paper ["Fast Segmentation of 3D Point Clouds: A Paradigm on LiDAR Data for Autonomous Vehicle Applications"](https://www.researchgate.net/publication/318325507_Fast_Segmentation_of_3D_Point_Clouds_A_Paradigm_on_LiDAR_Data_for_Autonomous_Vehicle_Applications). In brief, the algorithm subdivides point cloud into several point cloud segments formed by splitting point cloud along x-direction. Initially, the seed points are randomly selected to produce initial estimation of ground plane for each point cloud segment. Next, the iterative procedure performs ground plane fit on each of the ground segment within the point cloud segment, and the results are refined iteratively. From the results of various tests, I observed that the algorithm provides higher quality results compared to RANSAC segmentation as well as has more deterministic number of iterations, despite the complexity of fitting the ground plane on a large number of points.

![image2](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/ground_segmentation.png)

### Obstacle Point Cloud Clusters

[Fast Euclidean Clustering](https://arxiv.org/abs/2208.07678) (FEC) is a hierarchical clustering algorithm that starts by dividing the data into small subsets, and then merges these subsets to form larger clusters. It uses a threshold distance to determine whether two subsets should be merged or not. 

[DBSCAN](https://en.wikipedia.org/wiki/DBSCAN) is a density-based clustering algorithm that groups together data points that are close to each other and have high density. It defines clusters as regions of high density separated by regions of low density. DBSCAN is more flexible than FEC because it can identify clusters of arbitrary shapes and sizes, and it can handle datasets with noise and outliers. Typically, you will observe more granular clusters when using DBSCAN approach compared to FEC.

![image3](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/euclidean_clustering.png)

### Convex Simplification

In order to simplify dense clusters formed by Fast Euclidean Clustering (FEC) or DBSCAN, currently the pipeline utilizes Convex Hull simplification algorithms based on either [Chan's Algorithm](https://en.wikipedia.org/wiki/Chan%27s_algorithm) that uses Graham Scan to form initial simplification contours, followed by Jarvis March that merges disconnected convex hull components into a larger convex hull, if the number of points is large, or relies purely on [Graham Scan](https://en.wikipedia.org/wiki/Graham_scan) if the number of points is relatively small. The threshold value is based on imperative observations.

#### Convex Simplification of Obstacle Point Cloud Clusters (Fast Euclidean Clustering)

The below image demonstrates polygonization results formed from Fast Euclidean Clustering as a preprocessing method for cluster formation.

![image4](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/polygonization.png)

#### Convex Simplification of Obstacle Point Cloud Clusters (DBSCAN)

The below image demonstrates polygonization results formed from DBSCAN as a preprocessing method for cluster formation.

![image4](https://github.com/YevgeniyEngineer/LiDAR-Processing/blob/main/images/polygonization_dbscan.png)
