#include <iostream>
#include <sstream>
// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp> //turns out to be necessary
// include eigen before opencv2/core/eigen.hpp
// #include <eigen3/Eigen> error
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
// ros & message
#include <ros/ros.h>
#include <image_transport/image_transport.h> //image
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>  //pose
#include <cv_bridge/cv_bridge.h> //cv_bridge

using std::cout;
using std::endl;

class GoalDetector
{
public:
    GoalDetector();
    ~GoalDetector();
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void detectAndPub(cv::Mat &image);

    ros::NodeHandle nh;
    ros::Publisher goal_pub;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;

    // camera params
    cv::Mat intrinsics;
    cv::Mat distCoeffs;
    double markerLength;

    int picNum = 0;
    cv::VideoWriter writer;
    bool writer_init = false;
};

GoalDetector::GoalDetector() : it(nh)
{
    goal_pub = nh.advertise<geometry_msgs::Pose>("mygoal", 1);
    image_sub = it.subscribe("/camera/rgb/image_raw", 1, &GoalDetector::imageCallback, this);

    // get the camera params
    std::vector<double> K;
    nh.getParam("K", K);
    intrinsics = (cv::Mat_<double>(3, 3) << K[0], K[1], K[2],
                  K[3], K[4], K[5],
                  K[6], K[7], K[8]);
    std::vector<double> D;
    nh.getParam("D", D);
    distCoeffs = (cv::Mat_<double>(5, 1) << D[0], D[1], D[2], D[3], D[4]);
    nh.getParam("markerLength", markerLength);

    // open a window
    cv::namedWindow("Show RgbImage");
}

GoalDetector::~GoalDetector()
{
    cv::destroyWindow("Show RgbImage");
}

void GoalDetector::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::imshow("Show RgbImage", image);

    if (!writer_init)
    {
        writer.open("/home/hyper/navigation/45_ws/src/mydetector/myvideoOutput.avi", CV_FOURCC('M', 'J', 'P', 'G'), 20, cv::Size(image.cols, image.rows));
        writer_init = true;
    }
    // writer << image;

    // detect and publish when "Space key" is pressed
    char key = cvWaitKey(33);
    if (key == 32) //the Ascii of "Space key" is 32
    {
        // // save the pic, how to define path??
        // std::stringstream ss;
        // ss << "/home/hyper/navigation/"
        //    << "goal_" << picNum++ << ".jpg";
        // cv::imwrite(ss.str(), image);

        // detect and publish
        // detectAndPub(image);
    }
    detectAndPub(image);
}

// warning: only publish the first detected marker for convenience
void GoalDetector::detectAndPub(cv::Mat &image)
{
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(image, dictionary, corners, ids);
    // if at least one marker detected
    if (ids.size() > 0)
    {
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, markerLength, intrinsics, distCoeffs, rvecs, tvecs);
        cv::Mat outputImage;
        image.copyTo(outputImage);
        for (int i = 0; i < rvecs.size(); ++i)
        {
            auto rvec = rvecs[i];
            auto tvec = tvecs[i];
            cv::aruco::drawAxis(outputImage, intrinsics, distCoeffs, rvec, tvec, 0.07);
            writer << outputImage;
        }
        // for (unsigned int i = 0; i < ids.size(); i++)
        // {
        //     // std::vector<cv::Vec3d> rvecs, tvecs;
        //     // cv::aruco::estimatePoseSingleMarkers(corners[i], markerlength, intrinsics, distCoeffs, rvecs, tvecs); //求解旋转矩阵rvecs和平移矩阵tvecs
        //     cv::aruco::drawAxis(imageCopy, intrinsics, distCoeffs, rvecs[i], tvecs[i], 0.1);
        //     //3.rotaion vector to eulerAngles
        //     cv::Mat rmat;
        //     Rodrigues(rvecs[i], rmat);
        //     Eigen::Matrix3d rotation_matrix3d;
        //     cv2eigen(rmat, rotation_matrix3d);
        //     // Eigen::Vector3d eulerAngle = rotation_matrix3d.eulerAngles(0, 1, 2); //(0,1,2)表示分别绕XYZ轴顺序，即 顺序，逆时针为正
        //     // cout << "pitch " << eulerAngle.x() << "yaw " << eulerAngle.y() << "roll" << eulerAngle.z() << endl;
        //     cout << "rotation_matrix3d:" << rotation_matrix3d << endl;
        //     cout << "x= " << tvecs[i][0] << "y=" << tvecs[i][1] << "z=" << tvecs[i][2] << endl;

        //     // Eigen::Quaterniond q = Eigen::Quaterniond (Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d(0,0,1)));
        //     // cout<<q.coeffs()<<endl;

        //     geometry_msgs::Pose temp;
        //     temp.position.x=tvecs[i][0];
        //     temp.position.y=tvecs[i][1];
        //     temp.position.z=tvecs[i][2];
        //     temp.orientation.x=0;
        //     temp.orientation.y=0;
        //     temp.orientation.z=0;
        //     temp.orientation.w=1;
        //     goal_pub.publish(temp);
        // }
        cv::Mat rmat;
        cv::Rodrigues(rvecs[0], rmat);
        Eigen::Matrix3d rotation_matrix3d;
        cv::cv2eigen(rmat, rotation_matrix3d);
        Eigen::Matrix3d Rwb(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
        Eigen::Matrix3d Rwc = Rwb * (rotation_matrix3d.transpose());
        Eigen::Vector3d tc(tvecs[0][0], tvecs[0][1], tvecs[0][2]);
        tc = Rwc * tc;
        // Eigen::Vector3d ta(tc(0)-0.04,tc(2)+0.1-0.11-0.02,-tc(1)+0.25);

        geometry_msgs::Pose temp;
        temp.position.x = tc(0);
        temp.position.y = tc(1);
        temp.position.z = tc(2);
        cout << "x:" << tc(0) << " y:" << tc(2) << " z:" << -tc(1) << endl;
        // Eigen::Quaterniond q =
        temp.orientation.x = 0.0;
        temp.orientation.y = 0.0;
        temp.orientation.z = 0.70710678;
        temp.orientation.w = 0.70710678;
        goal_pub.publish(temp);
    }
    else
    {
        writer << image;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GoalDetector");
    GoalDetector goalDetector;
    ros::spin();

    return 0;
}