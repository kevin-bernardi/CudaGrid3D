PROGRAM=vector3d
SHARED_LIB_NAME=lib$(PROGRAM)
OUTPUT=test

INCLUDE_OPENCV=-I/usr/include/opencv4/
INCLUDE_STAR=-I/home/serl/STAR/AutonomousRobots/Libraries/star/include

$(OUTPUT): $(SHARED_LIB_NAME).so $(PROGRAM).h main.cpp
	g++  -o $(OUTPUT) main.cpp -L./ -L/home/serl/STAR/AutonomousRobots/Libraries/star/lib/ -l$(PROGRAM) -lopencv_core $(INCLUDE_OPENCV) -lstar_robotics_math $(INCLUDE_STAR)

$(SHARED_LIB_NAME).so: $(PROGRAM).o
	g++ -shared -fPIC -o $(SHARED_LIB_NAME).so $(PROGRAM).o  -L/usr/local/cuda/lib64 -lcuda -lcudart -lopencv_core

$(PROGRAM).o: $(PROGRAM).cu $(PROGRAM).h
	nvcc -c -Xcompiler -fPIC $(PROGRAM).cu $(INCLUDE_OPENCV) $(INCLUDE_STAR)

clean:
	rm -f $(PROGRAM).o $(SHARED_LIB_NAME).so *.obj $(OUTPUT)