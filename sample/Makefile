INCLUDE := $(shell python3 -m pybind11 --includes)
FLAG := -O3 -Wall -shared -std=c++11 -fPIC
SUFFIX := $(shell python3-config --extension-suffix)

demo:
	c++ $(FLAG) $(INCLUDE) sample.cpp -o sample$(SUFFIX)
