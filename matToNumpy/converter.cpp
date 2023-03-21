// include pybind11 header files so that we can use PYBIND11_MODULE macro
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
namespace py = pybind11;

#include <Eigen/Dense>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/eigen.hpp>

#include <PvSampleUtils.h>
#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>
#include <PvPipeline.h>
#include <PvBuffer.h>

//Source for Mat to Numpy conversion https://stackoverflow.com/questions/60949451/how-to-send-a-cvmat-to-python-over-shared-memory/60959732#60959732
py::dtype determine_np_dtype(int depth)
{
    switch (depth) {
    case CV_8U: return py::dtype::of<uint8_t>();
    case CV_8S: return py::dtype::of<int8_t>();
    case CV_16U: return py::dtype::of<uint16_t>();
    case CV_16S: return py::dtype::of<int16_t>();
    case CV_32S: return py::dtype::of<int32_t>();
    case CV_32F: return py::dtype::of<float>();
    case CV_64F: return py::dtype::of<double>();
    default:
        throw std::invalid_argument("Unsupported data type.");
    }
}

std::vector<std::size_t> determine_shape(cv::Mat& m)
{
    if (m.channels() == 1) {
        return {
            static_cast<size_t>(m.rows)
            , static_cast<size_t>(m.cols)
        };
    }

    return {
        static_cast<size_t>(m.rows)
        , static_cast<size_t>(m.cols)
        , static_cast<size_t>(m.channels())
    };
}

py::capsule make_capsule(cv::Mat& m)
{
    return py::capsule(new cv::Mat(m)
        , [](void *v) { delete reinterpret_cast<cv::Mat*>(v); }
        );
}

py::array mat_to_nparray(cv::Mat& m)
{
    if (!m.isContinuous()) {
        throw std::invalid_argument("Only continuous Mats supported.");
    }

    return py::array(determine_np_dtype(m.depth())
        , determine_shape(m)
        , m.data
        , make_capsule(m));
}



py::array readImage(){
  cv::Mat bgr_img = cv::imread("../img.bmp");
  auto npMat = mat_to_nparray(bgr_img);
  return npMat;
}

class Rectangle {
  private:
    int width, height;
  public:
    void set_values (int x,int y)
    {
       width = x;
       height = y;   
    };
    
    int area() 
    {
        return width*height;
    }
};

struct Pet {
    Pet(const std::string &name) : name(name) { }
    void setName(const std::string &name_) { name = name_; }
    const std::string &getName() const { return name; }

    std::string name;
};


PYBIND11_MODULE(converter, cppCV) {
    cppCV.doc() = "pybind11 plugin openCV demo"; // optional module docstring
    cppCV.def("readImage", &readImage, "Read image from a disk");

    py::class_<Pet>(cppCV, "Pet")
        .def(py::init<const std::string &>())
        .def("setName", &Pet::setName)
        .def("getName", &Pet::getName);

    /*py::class<Rectangle>(cppCV, "Rectangle")
        .def(py::init<>())
        .def("set_values", &Rectangle::set_values)
        .def("area", &Regtangle::area);*/
}

