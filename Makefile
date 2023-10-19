general: lib
	g++ test.cpp -o lib -L./ -lvector

lib: vector
	g++ -shared -o libvector.so vector3d.o -L/usr/local/cuda/lib64 -lcuda -lcudart
	
vector: vector3d.cu
	nvcc -c -Xcompiler -fPIC vector3d.cu

clean:
	rm *.o *.out *.so *.obj