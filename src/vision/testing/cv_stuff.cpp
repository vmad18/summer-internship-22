#include <bits/stdc++.h>

#include "ros/ros.h" 
#include "geometry_msgs/Quaternion.h"
#include "opencv2/opencv.hpp"

using namespace std;

using namespace ros;

using namespace cv;


int main(int argc, char **argv){
    
    string img_path = samples::findFile("src/vision/testing /tennis.png");

    Mat img = imread(img_path, IMREAD_COLOR); 

    imshow("img", img); 

    int wait = waitKey(0);


    cout<<wait<<endl;

    return 0;
}