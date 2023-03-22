
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

SUFFIX := $(shell python3-config --extension-suffix)

pyJaiGo$(SUFFIX):jaiGo.o
	g++ jaiGo.o -o pyJaiGo$(SUFFIX) -O3 -Wall -shared -std=c++11 -fPIC $(JAIGO_LIB) $(OPENCV_LIB)

jaiGo.o:jaiGo.cpp
	g++ jaiGo.cpp -o jaiGo.o -c -O3 -D_UNIX_ -D_LINUX_ -fPIC -fvisibility=hidden -std=c++11 $(JAIGO_INC) $(OPENCV_INC) $(PY_INC)

clean:
	rm -f jaiGo.o pyJaiGo$(SUFFIX)



	