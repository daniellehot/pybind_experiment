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

converter$(SUFFIX):converter.o
	g++ converter.o -o converter$(SUFFIX) -O3 -Wall -shared -std=c++11 -fPIC $(JAIGO_LIB) $(OPENCV_LIB)

converter.o:converter.cpp
	g++ converter.cpp -o converter.o -c -O3 -D_UNIX_ -D_LINUX_ -fPIC -std=c++11 $(JAIGO_INC) $(OPENCV_INC) $(PY_INC) $(EIGEN_INC) 

clean:
	rm -f converter.o converter$(SUFFIX)



	