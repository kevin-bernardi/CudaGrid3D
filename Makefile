PROGRAM=vector3d
SHARED_LIB_NAME=lib$(PROGRAM)
OUTPUT=test

$(OUTPUT): $(SHARED_LIB_NAME).so $(PROGRAM).h main.cpp
	g++ main.cpp -o $(OUTPUT) -L./ -l$(PROGRAM)

$(SHARED_LIB_NAME).so: $(PROGRAM).o
	g++ -shared -o $(SHARED_LIB_NAME).so $(PROGRAM).o -L/usr/local/cuda/lib64 -lcuda -lcudart
	
$(PROGRAM).o: $(PROGRAM).cu $(PROGRAM).h
	nvcc -c -Xcompiler -fPIC $(PROGRAM).cu

clean:
	rm -f $(PROGRAM).o $(SHARED_LIB_NAME).so *.obj $(OUTPUT)