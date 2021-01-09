#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

using namespace std
using namespace cv

int main()
{
    cv::Mat m_image = imread("/home/hyper/navigation/detection_ws/1.jpg");
    if (m_image.empty())
    {
        cout << "m_image  is empty" << endl;
        return 0;
    }
    //read para
    double markerlength = 0.1043;
    cv::Mat intrinsics = (Mat_<double>(3, 3) << 623.555, 0.0, 621.244,
                          0.0, 291.413, 265.64,
                          0.0, 0.0, 1.0);

    cv::Mat distCoeffs = (cv::Mat_<double>(4, 1) << 0.064817, -0.219, 0.008, -0.006);
    cv::Mat imageCopy;
    m_image.copyTo(imageCopy);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(m_image, dictionary, corners, ids); //检测靶标
    // if at least one marker detected
    if (ids.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids); //绘制检测到的靶标的框
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, markerlength, intrinsics, distCoeffs, rvecs, tvecs); //求解旋转矩阵rvecs和平移矩阵tvecs
        for (unsigned int i = 0; i < ids.size(); i++)
        {
            // std::vector<cv::Vec3d> rvecs, tvecs;
            // cv::aruco::estimatePoseSingleMarkers(corners[i], markerlength, intrinsics, distCoeffs, rvecs, tvecs); //求解旋转矩阵rvecs和平移矩阵tvecs
            cv::aruco::drawAxis(imageCopy, intrinsics, distCoeffs, rvecs[i], tvecs[i], 0.1);
            //3.rotaion vector to eulerAngles
            cv::Mat rmat;
            Rodrigues(rvecs[i], rmat);
            Eigen::Matrix3d rotation_matrix3d;
            cv2eigen(rmat, rotation_matrix3d);
            // Eigen::Vector3d eulerAngle = rotation_matrix3d.eulerAngles(0, 1, 2); //(0,1,2)表示分别绕XYZ轴顺序，即 顺序，逆时针为正
            // cout << "pitch " << eulerAngle.x() << "yaw " << eulerAngle.y() << "roll" << eulerAngle.z() << endl;
            cout << "rotation_matrix3d:" << rotation_matrix3d << endl;
            cout << "x= " << tvecs[i][0] << "y=" << tvecs[i][1] << "z=" << tvecs[i][2] << endl;
        }
    }
    // cv::imshow("out", imageCopy);
    // cv::waitKey();

    return 0;
}