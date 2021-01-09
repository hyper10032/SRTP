#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <sstream>

using namespace std;
using namespace cv;

int main()
{
    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    for (size_t i = 0; i < 3; i++)
    {
        cv::aruco::drawMarker(dictionary, 20 + i, 200, markerImage, 1);
        imwrite("./aruco_tag_" + to_string(i) + ".png", markerImage);
        // imshow("test", markerImage);//显示marker
        // waitKey();
    }

    return 0;
}