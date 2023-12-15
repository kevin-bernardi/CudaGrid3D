PROGRAM=grid3d
CPP_FILE=test.cpp
SHARED_LIB_NAME=lib$(PROGRAM)
EXECUTABLE=test

INCLUDE_OPENCV=-I/usr/include/opencv4/
#INCLUDE_STAR=-I/home/serl/STAR/AutonomousRobots/Libraries/star/include

LIBS_OPENCV=-lopencv_core -lopencv_highgui
#LIBS_STAR=-L/home/serl/STAR/AutonomousRobots/Libraries/star/lib/
LIBS_CUDA=-L/usr/local/cuda/lib64 -lcuda -lcudart
LIBS_CIMG=-L/usr/X11R6/lib -lm -lpthread -lX11
LIBS_PROGRAM=-L./ -l$(PROGRAM)


build: $(EXECUTABLE)
	mkdir -p output/CudaGrid3D
	cp $(SHARED_LIB_NAME).so ./output/CudaGrid3D/$(SHARED_LIB_NAME).so
	mkdir -p ./output/CudaGrid3D/include
	cp $(PROGRAM).h ./output/CudaGrid3D/include/$(PROGRAM).h

$(EXECUTABLE): $(SHARED_LIB_NAME).so $(PROGRAM).h $(CPP_FILE)
	g++ -o $(EXECUTABLE) $(CPP_FILE) $(LIBS_PROGRAM) $(LIBS_OPENCV) $(INCLUDE_OPENCV)

$(SHARED_LIB_NAME).so: $(PROGRAM).o
	g++ -shared -fPIC -o $(SHARED_LIB_NAME).so $(PROGRAM).o $(LIBS_CUDA) $(LIBS_CIMG) $(LIBS_OPENCV)

$(PROGRAM).o: $(PROGRAM).cu $(PROGRAM).h
	nvcc -c -Xcompiler -fPIC $(PROGRAM).cu $(INCLUDE_OPENCV)

clean:
	rm -f $(PROGRAM).o $(SHARED_LIB_NAME).so *.obj *.bmp $(OUTPUT)
	rm -rf output