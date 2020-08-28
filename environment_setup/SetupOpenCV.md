# Setup OpenCV

OpenCV is a widly used image processing library.

Here is how to install opencv from source code.

Go to opencv website <https://opencv.org/releases/> to get different version. Here we are using OpenCV 4.3.0 as an example.

Get the source code

```bash
wget https://github.com/opencv/opencv/archive/4.3.0.zip # You can use other download tools
```

Extract files

```bash
unzip opencv-4.3.0.zip
```

Install dependencies

```bash
sudo apt install build-essential libgtk2.0-dev libvtk5-dev libjpeg-dev libtiff4-dev libjasper-dev libopenexr-dev libtbb-dev
```

> OpenCV has many dependencies, the lack of some dependencies will affect part of its function. OpenCV will ckeck dependencies and adjust its functions while performing cmake. The dependencies above is enough for all the algorithm in this repo. You can find more info on <https://docs.opencv.org/4.3.0/d7/d9f/tutorial_linux_install.html>

Build & Install

```bash
cd opencv-4.3.0
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
make
sudo make install
```

> Use `cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..` , without spaces after -D if the above example doesn't work.