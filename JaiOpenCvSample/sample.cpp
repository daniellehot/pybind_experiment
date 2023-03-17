// include pybind11 header files so that we can use PYBIND11_MODULE macro
#include <pybind11/pybind11.h>
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

cv::Mat readImage(){
  //Sanity check
  cout<<"This is written from readImage()"<<endl;
  int test = 12 + 45;
  cout<<test<<endl;
  //Eigen check
  Eigen::MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << "Here is the matrix m:\n" << m << std::endl;
  //Opencv check
  cv::Mat img = cv::imread("../img.bmp");
  std::cout << "Img shape " << img.rows << "X" << img.cols << endl;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> eigen_mat;
  cv::cv2eigen(img, eigen_mat);
  cout<<eigen_mat.rows()<<endl;
  cout<<eigen_mat.cols()<<endl;
  return img;
}

PYBIND11_MODULE(sample, cppCV) {
    cppCV.doc() = "pybind11 plugin openCV demo"; // optional module docstring
    cppCV.def("readImage", &readImage, "Read image from a disk");
}

