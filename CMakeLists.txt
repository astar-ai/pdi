cmake_minimum_required(VERSION 2.8)
project(pdi LANGUAGES C CXX)

find_package(OpenCV 3.0.0 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(X11 REQUIRED)

add_executable(pdi pdi.cpp glwindow/glwindow_x11.cpp glwindow/scenewindow.cpp)
target_link_libraries(pdi  ${OPENGL_LIBRARIES}  ${X11_LIBRARIES} ${OpenCV_LIBRARIES})
