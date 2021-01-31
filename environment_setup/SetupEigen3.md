# Setup Eigen3

Eigen is an C++ open source library for linear algebra providing fast matrix computation, equation solving and etc.

Eigen only uses header files (no libraries), which means you only need to specify the location of the header file in CMakeLists.txt

## Install with apt

```bash
sudo apt install libeigen3-dev 
```

Normally the header files will be installed to "/usr/include/eigen3"

You can check the install location with the following command

```bash
sudo updatedb # update database used by "locate" command
locate eigen3
```

## Install from Source

Check the the latest stable release at <http://eigen.tuxfamily.org/index.php?title=Main_Page>

Here is an example installing Eigen 3.3.7 from source

```bash
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.bz2 # This is the download link, you can use any download tool to download it. Here we are using wget.
```

Extract files form the compressed file

```bash
tar -jxvf eigen-3.3.7.tar.bz2
```

Now there should be a directory named "eigen-3.3.7" containing all the source code.

```bash
cd eigen-3.3.7
mkdir build
cd build
cmake ..
sudo make install
```

Now Eigen3 should be installed properly

## Use Eigen3 with CMake

Directly add header file directory:

```cmake
# add header file
include_directories("/usr/include/eigen3")
```

"usr/include/eigen3" is the default installation directory. if you install Eigen3 at another location, you need to change it yours.

Another method is to use `find_package`

```cmake
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS}) # cannot use with 3.3.8
```

