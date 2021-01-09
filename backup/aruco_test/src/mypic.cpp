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
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h> //cv_bridge

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"

#include <sstream>

// #define PI 

using std::cout;
using std::endl;
using std::string;
using std::stringstream;

using namespace std;
using namespace cv;

unsigned int fileNum = 1;
bool saveCloud(false);

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv::imshow("Show RgbImage", cv_bridge::toCvShare(msg, "rgb8")->image);
    char key;
    key = cvWaitKey(33);
    if (key == 32) //the Ascii of "Space key" is 32
        saveCloud = true;
    if (saveCloud)
    {
        stringstream stream;
        stringstream stream1;
        stream << "Goal RgbImage" << fileNum << ".jpg";
        stream1 << "/home/hyper/navigation/" << fileNum << ".jpg";
        string filename = stream.str();
        string filename1 = stream1.str();
        cv::imwrite(filename1, cv_bridge::toCvShare(msg)->image);
        saveCloud = false;
        fileNum++;
        cout << filename << " had Saved." << endl;

        cv::Mat m_image = imread(filename1);
        if (m_image.empty())
        {
            cout << "m_image  is empty" << endl;
            // return 0;
            return;
        }
        //read para
        double markerlength = 0.1;
        cv::Mat intrinsics = (Mat_<double>(3, 3) << 537.0006940530445, 0.0, 325.9583194349277,
                              0.0, 537.939344182713, 234.4887972391361,
                              0.0, 0.0, 1.0);

        cv::Mat distCoeffs = (cv::Mat_<double>(4, 1) << 0.04869698991798918, -0.1602608523554675, 4.888899721193616e-05, -0.0006380580859116176);
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

                // Eigen::Quaterniond q = Eigen::Quaterniond (Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d(0,0,1)));
                // cout<<q.coeffs()<<endl;

                geometry_msgs::Pose temp;
                temp.position.x=tvecs[i][0];
                temp.position.x=tvecs[i][1];
                temp.position.x=tvecs[i][2];
                temp.orientation.x=0;
                temp.orientation.y=0;
                temp.orientation.z=0;
                temp.orientation.w=1;
                // goal_pub.publish(temp);
            }
        }

    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Image_listener");
    ros::NodeHandle nh;
    cv::namedWindow("Show RgbImage");
    //   cv::startWindowThread();

    ros::Publisher goal_pub = nh.advertise<geometry_msgs::Pose>("mygoal", 1000);


    cout << "1";
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);
    ros::spin();
    //   cv::destroyWindow("Show RgbImage");
}
