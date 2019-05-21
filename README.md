# PDI: Panorama Depth Image

PDI provides a solution to get the panorama depth map based on a fisheye stereo image. For more information see
[https://astar.ai](https://astar.ai).
<p align="center">
  <img src="http://astar.support/dotai/pdi_3d_scene.png">
</p>

The following steps have been tested and passed on Ubuntu **16.04.5**.

### 1. OpenCV Installation

#### 1.1 Required Dependencies

	sudo apt-get install git cmake build-essential pkg-config libgtk2.0-dev libssl-dev libv4l-dev v4l-utils libjpeg-dev libtiff5-dev libjasper-dev libpng-dev libavcodec-dev libavformat-dev libswscale-dev libxvidcore-dev libx264-dev libxine2-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libqt4-dev libgtk-3-dev mesa-utils libgl1-mesa-dri libqt4-opengl-dev libatlas-base-dev gfortran libeigen3-dev python2.7-dev python3-dev python-numpy python3-numpy 

#### 1.2 Install OpenCV 3.0 (or Above)

	cd
	mkdir library
	cd library

	git clone https://github.com/opencv/opencv_contrib.git
	cd opencv_contrib
	git checkout tags/3.0.0
	cd ..

	git clone https://github.com/opencv/opencv.git
	cd opencv
	git checkout tags/3.0.0
	mkdir build
	cd build
	cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..
	make -j
	sudo make install
	
	echo 'export LD_LIBRARY_PATH=LD_LIBRARY_PATH:/lib:/usr/lib:/usr/local/lib' >> ~/.bashrc
	source ~/.bashrc

### 2. Compile

	git clone https://github.com/astar-ai/pdi.git
	cd pdi
	chmod 777 ./compile.sh
	./compile.sh

### 3. Run

	./pdi

### 4. Calibration Parameter File
To run PDI in the **LIVE** mode, you need to download the calibration parameter file from online.
Each CaliCam stereo/mono camera has a **UNIQUE** parameter file. Please download the corresponding parameter file by following the instructions at [https://astar.ai/collections/astar-products](https://astar.ai/collections/astar-products).

### 5. Operation

#### 5.1 'Fisheye Image' window
The first two trackbars are used to adjust the **numDisparities** and **blockSize** for [OpenCV stereo matching functions](https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereobm). 
The third trackbar 'Threshold' is used to adjust the field of view of the camera.

#### 5.2 'Panorama 3D Scene' OpenGL window
Mouse button: left drag - rotate, middle drag - pan, middle scroll: zoom, right drag: zoom

#### 5.3 Exit
Press 'q' or 'Esc' key on the 'Fisheye Image' window to exit.

### 6. Live Mode
To run CaliCam in a live mode, please change the variable live to true:

	bool      live = true;

and run

	./pdi YOUR_CALIBRATION_FILE.yml

