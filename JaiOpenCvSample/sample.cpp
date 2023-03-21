// include pybind11 header files so that we can use PYBIND11_MODULE macro
#include <pybind11/pybind11.h>
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

Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> readImage(){
  //Sanity check
  cout<<"This is written from C++ readImage()"<<endl;
  int test = 12 + 45;
  cout<<test<<endl;

  //Eigen check
  //Eigen::MatrixXd m(2,2);
  Eigen::Matrix<double, 2, 2> m;
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << "Here is the matrix m:\n" << m << std::endl;

  //Opencv check
  cv::Mat bgr_img = cv::imread("../img.bmp");
  std::cout << "Img shape " << bgr_img.rows << "X" << bgr_img.cols << endl;

  cv::Mat bgr[3];   //destination array
  cv::split(bgr_img,bgr);//split source  
  //cout<<"BGR array size "<<sizeof(bgr)/sizeof(bgr[0])<<endl;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> eigenB;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> eigenG;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> eigenR;
  cv::cv2eigen(bgr[0], eigenB);
  cv::cv2eigen(bgr[1], eigenG);
  cv::cv2eigen(bgr[2], eigenR);
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> eigenImg;
  cout<<"Test"<<endl;
  eigenImg = eigenB;
  cout<<"Test 2"<<endl;
  cout<<"eigenImg ROWS "<< eigenImg.rows() << " COLS "<<eigenImg.cols()<<endl;

  /*
  cv::Mat img;
  cv::cvtColor(bgr_img, img, cv::COLOR_BGR2GRAY);
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> eigen_mat;
  cv::cv2eigen(img, eigen_mat);
  cout<<eigen_mat.rows()<<endl;
  cout<<eigen_mat.cols()<<endl;
  */
  cout<<"///////////////"<<endl;
  return eigenImg;
}




PYBIND11_MODULE(sample, cppCV) {
    cppCV.doc() = "pybind11 plugin openCV demo"; // optional module docstring
    cppCV.def("readImage", &readImage, "Read image from a disk");
}

