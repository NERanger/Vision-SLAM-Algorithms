# Setup g2o

G2o is a general framework for graph optimization, which means you can solve any least square problem that can be written as a graph optimization problem.

Here are g2o [github page](https://github.com/RainerKuemmerle/g2o) and its [description page on OpenSLAM](https://openslam-org.github.io/g2o.html)

## Dependencies

* [Eigen3](./SetupEigen3.md)

```bash
sudo apt-get install qt5-qmake qt5-default libqglviewer-dev-qt5 libsuitesparse-dev libcxsparse3 libcholmod3 
```

## Installation

```bash
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build
cd build
cmake ..
make
sudo make install
```

## Use with CMake

```cmake
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
#...
target_link_libraries(<your executable> ${G2O_LIBS})
```

