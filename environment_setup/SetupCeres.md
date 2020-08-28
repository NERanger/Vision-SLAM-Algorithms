# Setup Ceres

Ceres is an open source C++ library for modeling and solving large, complicated optimization problems.

See <http://ceres-solver.org/> for more detailed information.

## Dependencies

* [Eigen3](./SetupEigen3.md) (3.2.2 or later **strongly** recommended, 3.1.0 or later **required**.)

```bash
sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev 
```

## Installation

```bash
git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver
mkdir build
cd build
cmake ..
make
sudo make install
```

## Use Ceres with CMake

```cmake
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
#...
target_link_libraries(<your executable> ${CERES_LIBRARIES})
```

