
JAIGO_LIB = -L/opt/jai/ebus_sdk/Ubuntu-20.04-x86_64/lib \
	-lPvAppUtils                 \
	-lPtConvertersLib            \
	-lPvBase                     \
	-lPvBuffer                   \
	-lPvGenICam                  \
	-lPvSystem                   \
	-lPvStream                   \
	-lPvDevice                   \
	-lPvTransmitter              \
	-lPvVirtualDevice            \
	-lPvPersistence              \
	-lPvSerial                   \
	-lPvCameraBridge
OPENCV_LIB = $(shell pkg-config opencv4 --libs)

JAIGO_INC = -I/opt/jai/ebus_sdk/Ubuntu-20.04-x86_64/include
OPENCV_INC = $(shell pkg-config opencv4 --cflags)
PY_INC := $(shell python3 -m pybind11 --includes)
EIGEN_INC := $(shell pkg-config eigen3 --cflags)

SUFFIX := $(shell python3-config --extension-suffix)

sample$(SUFFIX):sample.o
	g++ sample.o -o sample$(SUFFIX) -O3 -Wall -shared -std=c++11 -fPIC $(JAIGO_LIB) $(OPENCV_LIB)

sample.o:sample.cpp
	g++ sample.cpp -o sample.o -c -O3 -D_UNIX_ -D_LINUX_ -fPIC -std=c++11 $(JAIGO_INC) $(OPENCV_INC) $(PY_INC) $(EIGEN_INC) 

clean:
	rm -f sample.o sample$(SUFFIX)



	