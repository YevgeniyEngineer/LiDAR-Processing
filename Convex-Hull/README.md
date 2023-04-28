# Convex-Hull
Implementation of 2D and 3D Convex Hull algorithms in C++.

## Build and Run
```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
./test_convex_hull
```

## Example Output
```bash
./test_convex_hull
Generating convex hull from 99999 points

Constructing convex hull using GRAHAM SCAN 
Elapsed time (s): 0.0122264
Number of hull points: 35
Hull is oriented clockwise
Convex Hull Indices:
23547 63020 56439 29308 89027 87077 80762 89421 18234 70916 12844 69220 54871 69047 63965 45659 64049 87893 93780 23619 14237 59610 69908 49542 32328 99981 58437 92771 90107 89075 92313 96490 65582 73299 73658 

Constructing convex hull using ANDREW MONOTONE CHAIN 
Elapsed time (s): 0.0110248
Number of hull points: 35
Hull is oriented clockwise
Convex Hull Indices:
23547 63020 56439 29308 89027 87077 80762 89421 18234 70916 12844 69220 54871 69047 63965 45659 64049 87893 93780 23619 14237 59610 69908 49542 32328 99981 58437 92771 90107 89075 92313 96490 65582 73299 73658 

Constructing convex hull using JARVIS MARCH 
Elapsed time (s): 0.0157665
Number of hull points: 35
Hull is oriented clockwise
Convex Hull Indices:
54871 69047 63965 45659 64049 87893 93780 23619 14237 59610 69908 49542 32328 99981 58437 92771 90107 89075 92313 96490 65582 73299 73658 23547 63020 56439 29308 89027 87077 80762 89421 18234 70916 12844 69220 

Constructing convex hull using CHAN 
Elapsed time (s): 0.00650961
Number of hull points: 35
Hull is oriented clockwise
Convex Hull Indices:
54871 69047 63965 45659 64049 87893 93780 23619 14237 59610 69908 49542 32328 99981 58437 92771 90107 89075 92313 96490 65582 73299 73658 23547 63020 56439 29308 89027 87077 80762 89421 18234 70916 12844 69220
```