g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"glwindow/glwindow_x11.d" -MT"glwindow/glwindow_x11.o" -o "glwindow/glwindow_x11.o" "./glwindow/glwindow_x11.cpp"
g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"glwindow/scenewindow.d" -MT"glwindow/scenewindow.o" -o "glwindow/scenewindow.o" "./glwindow/scenewindow.cpp"
g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"pdi.d" -MT"pdi.o" -o "pdi.o" "./pdi.cpp"
g++  -o "pdi"  ./glwindow/glwindow_x11.o ./glwindow/scenewindow.o  ./pdi.o   -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_videoio -lopencv_calib3d -lGL -lX11
rm glwindow/*.d glwindow/*.o *.d *.o
