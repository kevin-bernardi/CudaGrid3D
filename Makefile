all: main.cpp
	g++ -o test main.cpp

cuda: transfer.cu
	nvcc -o transfer.out transfer.cu

vec: vector3d.cu
	nvcc -o vector.out vector3d.cu

help: struct.cu
	nvcc -o struct struct.cu

clean:
	rm *.out *.o *.txt *.obj