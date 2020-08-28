# Setup PCL

The Point Cloud Library (PCL) is a standalone, large scale, open project for 2D/3D image and point cloud processing.

* [Official Github page](https://github.com/PointCloudLibrary/pcl)
* [Documentation](https://pcl-tutorials.readthedocs.io/en/latest/index.html)

## Compiling From Source

### Dependencies

* Boost

  Minimum version:

  ​	V1.40 (without OpenNI)

  ​	V1.47 (with OpenNI)

  Mandatory:

  ​	pcl_*

* Eigen 

  Minimum version: 3.0

  Mandatory: pcl_*

* FLANN

  Minimum version: 1.7.1

  Mandatory: pcl_*

* VTK

  Minimum version: 5.6

  Mandatory: pcl_visualization

* Qhull (optional)

  Minimum version: 2011.1

  Mandatory: pcl_surface

* OpenNI (optional)

  Minimum version: 1.3

  Mandatory: pcl_io

* CUDA (optional)

  Minimum version: 4.0

  Mandatory: pcl_*

### Compiling

Go to [Github](https://github.com/PointCloudLibrary/pcl/releases) and download the version number of your choice. (here we use v1.7.2 as an example)

```bash
 tar -zxvf pcl-pcl-1.7.2.tar.gz
 cd pcl-pcl-1.7.2
 mkdir build
 cd build
 cmake -DCMAKE_BUILD_TYPE=Release ..
 make
 sudo make install
```

## Reference

<https://pcl-tutorials.readthedocs.io/en/latest/compiling_pcl_posix.html#compiling-pcl-posix>