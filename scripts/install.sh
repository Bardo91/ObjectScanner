#######################
# Create build folder
mkdir build

#######################
# Install dependencies
cd build
mkdir dep
cd dep

# opencv
sudo apt-get update
sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

mkdir opencv
cd opencv
git clone https://github.com/opencv/opencv
git clone https://github.com/opencv/opencv_contrib
cd opencv
mkdir build
cd build cmake .. -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules
make -j5
sudo make install

# PCL
cd ../..
mkdir pcl
git clone https://github.com/PointCloudLibrary/pcl
mkdir build
cd build
cmake ..
make -j5
sudo make install

########################
# Compile project
