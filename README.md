# PDI: Panorama Depth Image

<p align="center">
  <img src="http://astar.support/dotai/pdi_fisheye_image1.jpg">
</p>

<p align="center">
  <img src="http://astar.support/dotai/pdi_3d_scene2.png">
</p>

PDI provides a solution to get the panorama depth image from a single fisheye stereo image pair. A wider view angle always benefits the environmental perception ability. For more information see
[https://astar.ai](https://astar.ai).

The following steps have been tested and passed on Ubuntu 16.04.5.

### 1. Theoretical Background

Fisheye Camera Model:
C. Mei and P. Rives, Single View Point Omnidirectional Camera Calibration From Planar Grids, ICRA 2007.

Fisheye Stereo Reconstruction:
S. Li, Binocular Spherical Stereo, ITS 2009.

### 2. OpenCV Installation 

Follow the steps in [CaliCam@GitHub](https://github.com/astar-ai/calicam).

### 3. Compile

	git clone https://github.com/astar-ai/pdi.git
	cd pdi
	chmod 777 ./compile.sh
	./compile.sh

### 4. Run

	./pdi

### 5. Operation

#### 5.1 'Fisheye Image' window
The first two trackbars are used to adjust the **numDisparities** and **blockSize** for [OpenCV stereo matching functions](https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereobm). 
The third trackbar 'Threshold' is used to adjust the field of view of the camera.

#### 5.2 'Panorama 3D Scene' OpenGL window
Mouse button: left drag - rotate, middle drag - pan, middle scroll - zoom, right drag - zoom. 

PDI uses **GLWindow** library from [http://ethaneade.org/](http://ethaneade.org/).

#### 5.3 Exit
Press 'q' or 'Esc' key on the 'Fisheye Image' window to exit.

### 6. Live Mode
To run PDI in a live mode, please change the variable live to true:

	bool      live = true;

and run

	./pdi YOUR_CALIBRATION_FILE.yml

### 7. Calibration Parameter File
To run PDI in the **LIVE** mode, you need to download the calibration parameter file from online.
Each CaliCam stereo/mono camera has a **UNIQUE** parameter file. Please download the corresponding parameter file by following the instructions at [https://astar.ai/collections/astar-products](https://astar.ai/collections/astar-products).


