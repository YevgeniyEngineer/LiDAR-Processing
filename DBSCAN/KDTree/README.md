# KDTree

Fast C++ implementation of KDTree. A header only project, supports nearest neighbour search in 2D and 3D.
Supported functionality:
- Multithreaded construction of KDTree
- Nearest neighbour search
- K-Nearest neighbour search
- K-Nearest neighbour search within radius squared
- All nearest neighbour search within radius squared

## Benchmarks for 3D KDTree (worst cases):

CPU: 12th Gen Intel® Core™ i7-12700H × 20 | 16.0 GiB RAM

| Number of Points | KD-Tree Construction (Sequential) | KD-Tree Construction (Parallel) | KDTree Query |
| ---------------- | --------------------------------- | ------------------------------- | ------------ |
| 10,000           | 0.00165645                        | 0.00188793                      | 2.189e-06    |
| 100,000          | 0.0247329                         | 0.010698                        | 4.475e-06    |
| 1,000,000        | 0.247114                          | 0.0850132                       | 3.734e-06    |
| 10,000,000       | 2.9207                            | 1.12204                         | 5.224e-06    |
| 50,000,000       | 15.8889                           | 6.17333                         | 5.218e-06    |
