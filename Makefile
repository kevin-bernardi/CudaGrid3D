lib: cuda
	g++ -shared -o libvector.so vector3d.o -L/usr/local/cuda/lib64 -lcuda -lcudart
	g++ test.cpp -o lib -L./ -lvector
	export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/serl/Documents/3d_analysis/

cuda: vector3d.cu
	nvcc -c -Xcompiler -fPIC vector3d.cu

clean:
	rm *.o *.out *.so