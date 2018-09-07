#This is cross platform make file. 
# To build on arm platform, please run "source env.sh" first.

TARGET = CVStereoDepth

TARGET_SRC = CVStereoCalibration.cpp \
			 CVStereoCorrespondance.cpp \
			 CVStereoDepth.cpp \
			 CVStereoWebcam.cpp \
			 globaldata.cpp \
			 stdafx.cpp \
			 stereo_calibrate.cpp

INC = -isystem/usr/include \
	  -I.


CFLAGS +=  $(INC) -Wall -g
CXXFLAGS += $(CFLAGS) -std=c++14
LDFLAGS = -L/usr/lib/x86_64-linux-gnu -L/usr/lib \
			-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -lopencv_legacy -lopencv_features2d -lrt -pthread \
			-Wl,-rpath-link,/usr/lib,-rpath-link,/usr/lib/x86_64-linux-gnu

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