#This is cross platform make file. 
# To build on arm platform, please run "source env.sh" first.

TARGET = CVStereoDepth

TARGET_SRC = CVStereoCalibration.cpp \
			 CVStereoCorrespondance.cpp \
			 CVStereoDepth.cpp \
			 globaldata.cpp \
			 CVStereoWebcam.cpp \
			 stdafx.cpp \
			 stereo_calibrate.cpp \
			 UseFaceDetection/facedetection_camera.cpp

INC = -isystem/usr/local/include \
      -IUseFaceDetection/include \
      -IUseFaceDetection \
	  -I.


CFLAGS +=  $(INC) -Wall -g
CXXFLAGS += $(CFLAGS) -std=c++14
LDFLAGS = -L/usr/local/lib -L/usr/lib -LUseFaceDetection/lib \
			-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_calib3d \
            -lopencv_features2d -lopencv_imgcodecs -lopencv_videoio -lrt -pthread -lseeta_facedet_lib \
			-Wl,-rpath-link,/usr/lib,-rpath-link,/usr/local/lib

TARGET_OBJ = $(patsubst %.cpp,build/%.o,$(TARGET_SRC))

all :$(TARGET)
	
$(TARGET) : $(TARGET_OBJ)  
	$(CXX) $^ $(LDFLAGS)  -o $@ 

build/%.o : %.cpp
	$(CXX) -c $(CXXFLAGS) $< -o $@

#.PHONY: clean
clean:
	rm -rf $(TARGET)
	rm -rf $(TARGET_OBJ)