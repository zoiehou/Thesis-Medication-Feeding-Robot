#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>

//(1) include face header
#include "opencv2/face.hpp"
#include "opencv2/face/facemark_train.hpp"
#include <opencv2/face/facemarkLBF.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

//(2) include face header
#include "opencv2/objdetect.hpp"
#include <iostream>

#include "ros/ros.h"
#include "opencv2/core/core.hpp"
#include <image_transport/image_transport.h>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <cv.h>
#include "opencv2/opencv.hpp"
#include <list>
#include "std_msgs/Int64.h"
#include "std_msgs/Int32.h"

#include "stdlib.h"
#include "time.h"
#include <cmath>

using namespace cv;
using namespace std;
using namespace ml;
using namespace cv::face;


//(3) Global variables 
Ptr<Facemark> facemark;
//const facemark;
String cascade_name;
CascadeClassifier faceDetector;
String window_name = "Capture - Face detection";
static const std::string OPENCV_WINDOW = "Image window";
//double focal_length = 554.25597119;
//double focal_length = 374.164;  // experimental value (mm)
double focal_length = 419.437;  // experimental value (mm)
//double face_real_width = 156.4;  // measured value (mm)
double face_real_width;
//double face_square_diff = 48; // difference between the actual face width and the face square
//double face_image_width = 196;
//double distance = 0;



void process(Mat img, Mat imgcol, int* diff_x, int* diff_y, double* forward_dist) {  //(black and white, colored)
    // origin (x, y) is on top left corner
    vector<Rect> faces;
    faceDetector.detectMultiScale(img, faces);
    vector< vector<Point2f> > shapes;
    Mat imFace;
    Rect img_center;

    // center of image
    int img_centerx = img.cols / 2;
    int img_centery = img.rows / 2;

    // only contains one face
    if (faces.size() != 0) {  // && faces[0].width > 200; check if face width > 200 pixels to ensure its a face
	// MAYBE CAN CHECK HOW FAR THE ROBOT CAN REACH AND TAKE AN APPROX VALUE OF HOW WIDE AVG
	// FACES ARE AT THAT POINT
	//for (size_t i = 0; i < faces.size(); i++)
        for (size_t i = 0; i < 1; i++)  // will only have one face in camera frame
        {
	    // rectangle of face
            cv::rectangle(imgcol, faces[i], Scalar(255, 0, 0));
	    // rectangle of center
	    img_center = Rect(img_centerx - faces[i].width / 2, img_centery - faces[i].height / 2, faces[i].width, faces[i].height);
	    
	    cv::rectangle(imgcol, img_center, Scalar(0, 0, 255));


	    // compute the difference between center and face rectangles
	    // the x and y the camera needs to move in to reach face
	    *diff_x = faces[i].x - img_center.x;
	    *diff_y = faces[i].y - img_center.y;

	    // compute the distance from camera to face in x-direction (forward)
	    *forward_dist = (face_real_width * focal_length) / faces[i].width;
	    //float Focal_length = (400 * (faces[i].width - face_square_diff)) / face_real_width;
	    //cout << "distance" << endl;
	    //cout << *forward_dist << endl;
	    //cout << "face image width" << endl;
	    //cout << faces[i].width << endl;
	}
	
        waitKey(5);
    }
    else {
	*diff_x = -10000;  // indicate a face is not detected
	*diff_y = -10000;  // indicate a face is not detected
	*forward_dist = 0;
        cout << "Faces not detected." << endl;
    }

    // compute distance to the face
    // Measured_Distance = distance of camera to face?
    //Focal_length = (Measured_Distance * Width_In_Image) / Real_Width;

}



// ROS callback function
//image_transport::Publisher chatter_pub;

ros::Publisher chatter_pub;
ros::Publisher chatter_pub2;
ros::Publisher chatter_pub3;

void imageCallback(const sensor_msgs::ImageConstPtr& source)
{
    cv_bridge::CvImagePtr aa = cv_bridge::toCvCopy(source, sensor_msgs::image_encodings::MONO8);

    static cv_bridge::CvImagePtr prev = cv_bridge::toCvCopy(source, sensor_msgs::image_encodings::MONO8);
    cv_bridge::CvImagePtr out = cv_bridge::toCvCopy(source, sensor_msgs::image_encodings::RGB8);

    Mat imgColored;
    cvtColor(out->image, imgColored, COLOR_BGR2RGB);  //COLOR_BGR2GRAY 
    // declare variables that the end effector needs to move in each direction   
    int x, y = 0;
    double forward_dist = 0;
    process(out->image, imgColored, &x, &y, &forward_dist);
    // Update GUI Window
    namedWindow(OPENCV_WINDOW, WINDOW_AUTOSIZE);
    imshow(OPENCV_WINDOW, imgColored);
    waitKey(5);
    
    // Output x and y distances the robotic arm needs to move
    //ROS_INFO("%s", msg.data.c_str());
    //string x_string = to_string(x);

    // x information
    std_msgs::Int32 msg;
    msg.data = x;
    // y information
    std_msgs::Int32 msg2;
    msg2.data = y;
    // forward information
    std_msgs::Int32 msg3;
    msg3.data = forward_dist;
    // publish information
    chatter_pub.publish(msg);
    chatter_pub2.publish(msg2);
    chatter_pub3.publish(msg3);
}



// main function
int main( int argc, char** argv )
{

    ros::init(argc, argv, "facedetect");
    
    //-- 1. Load the cascades
    cv::CommandLineParser parser(argc, argv,
        "{cascade|../include/haarcascades/haarcascade_frontalface_alt2.xml|}"
        "{scale|1|}{try-flip||}{@filename||}"
    );

    cascade_name = parser.get<string>("cascade");

    if (!parser.check())
    {
        parser.printErrors();
        return 0;
    }

    if( !faceDetector.load( cascade_name ) )
    {
        cerr << "ERROR: Could not load classifier cascade" << endl;
        return -1;
    }

    // get input of user's face width in real life
    cout << "Enter the user's face width please (in mm):" << endl;
    cout << "(If a fixed forward distance of 15cm is desired, enter 0)" << endl;
    cin >> face_real_width;

    //-- 2. ROS from camera
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub;

    sub = it.subscribe("/camera/color/image_raw", 1, imageCallback);
    chatter_pub = n.advertise<std_msgs::Int32>("chatter", 1);  // publishes the x distance differences between robot and face
    chatter_pub2 = n.advertise<std_msgs::Int32>("chatter2", 1);  // publishes the y distance differences between robot and face
    chatter_pub3 = n.advertise<std_msgs::Int32>("chatter3", 1);  // publishes the forward distance from robot to face
    
    srand(time(0));
    
    ROS_INFO("facedetect running");

    ros::spin();
    
    //-- 2. Read the video stream
    /*VideoCapture cap("../include/images/5-FemaleNoGlasses.avi");

    for (;;)
    {
        if (!cap.isOpened()) {
            cout << "Video Capture Fail" << endl;
            break;
        }
        else {
            Mat img;
            Mat imgbw;
            cap >> img;  
            //resize(img, img, Size(460, 460), 0, 0, INTER_LINEAR_EXACT);
            cvtColor(img, imgbw, COLOR_BGR2GRAY);           
            process(imgbw, img);
            namedWindow(OPENCV_WINDOW, WINDOW_AUTOSIZE);
            imshow(OPENCV_WINDOW, img);
            waitKey(5);
        }
    }

    // When everything done, release the video capture object
    cap.release();

    // Closes all the frames
    destroyAllWindows();*/

    return 0;
}
