# Setup Sophus

Sophus is a Lie Algebra library based on Eigen.

Sophus only provide double-precision class for Lie Group / Lie Algebra in early version. In later versions template class is used for different precision Lie Group / Lie Algebra, which also increase the usage difficulty.

There is no need to perform `sudo make install` since Sophus has register its path to cmake while performing `cmake`. But there is nothing wrong if you do the install.

## Dependencies

* [Eigen3](./SetupEigen3.md)

## None-Template Version Installation

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build
cd build
cmake ..
make
sudo make install # optional
```

## Template Version Installation

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
mkdir build
cd build
cmake ..
make
sudo make install # optional
```

## Use Sophus with CMake

```cmake
find_package(Sophus REQUIRED)
#...
target_link_libraries(<your executable> Sophus::Sophus)
```

## Troubleshooting

If you encounter the following problem while compiling Sophus

```
/Sophus/sophus/so2.cpp:32:26: error: lvalue required as left operand of assignment
unit_complex_.real() = 1.;

/Sophus/sophus/so2.cpp:33:26: error: lvalue required as left operand of assignment
unit_complex_.imag() = 0.;
```

Find the following part in *so2.cpp*

```cpp
SO2::SO2()
{
  unit_complex_.real() = 1.;
  unit_complex_.imag() = 0.;
}
```

Change to

```cpp
SO2::SO2()
{
  //unit_complex_.real() = 1.;
  //unit_complex_.imag() = 0.;
  unit_complex_.real(1.);
  unit_complex_.imag(0.);
}
```

