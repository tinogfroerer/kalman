# Use C++11 or newer to get pololu to work
CFLAGS += -std=c++11 -o

# Import libraries we are using
LIBS = -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs -lopencv_calib3d \
-lopencv_dnn -lopencv_features2d -lopencv_flann -lopencv_ml -lopencv_photo -lopencv_shape \
-lopencv_stitching -lopencv_superres -lopencv_videoio -lopencv_video -lopencv_videostab 

kalman : kalman.cpp
	c++ kalman.cpp $(LIBS) $(CFLAGS) kalman
 
