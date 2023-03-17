// include pybind11 header files so that we can use PYBIND11_MODULE macro
#include <pybind11/pybind11.h>

namespace py = pybind11;

int sum(int start, int end){
    // calculate sum from i to j
  if (start > end) return 0;

  int sum = 0;
  for (int i = start; i <= end; i++){
    sum += i;
  }

  return sum;
}

float divide(float a, float b){
    if ( b!=0 ){
        return (a/b);
    } 
    else
     {
        return -99999999;
     } 
}

PYBIND11_MODULE(sample, m) {
    m.doc() = "pybind11 demo plugin"; // optional module docstring

    m.def("sum", &sum, "calculate sum from start to end", py::arg("start") = 1, py::arg("end") = 1000);
    m.def("divide", &divide, "divide two numbers");
}

