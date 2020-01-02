#include "CameraController.h" 
#include <opencv2/opencv.hpp> 
#include <thread>
#include <chrono>
#include "KJCalibration.h" //  Kejing - Smarteye calibration library

using namespace std;
using namespace cv;
double fx_d, px_d, py_d, px_d_2, baseline, offset;
double w_x, w_y, w_z;
void Calculate_world_coord(double x, double y, double dif)
{
    double X, Y, Z, W;
    X = x - px_d;
    Y = y - py_d;
    Z = fx_d;
    W = (dif - offset) / baseline;
    w_x = X / W;
    w_y = Y / W;
    w_z = Z / W;
}

void ResizeShow(Mat img1, string WindowName)
{
	Mat img2;
	int w1, w2, h1, h2;
	h1 = img1.rows;
	w1 = img1.cols;
	h2 = 640;
	w2 = (w1*h2) / h1;
	resize(img1, img2, Size(w2, h2), 0, 0, CV_INTER_LINEAR);
	//namedWindow(WindowName);
	imshow(WindowName, img2);
}

void ResizeShow_two(Mat img1, Mat img2, string WindowName,string message)
{
    Mat dispImg;
    int w, w1, h, h1;
    h = 560;
    h1 = img1.rows;
    w1 = img1.cols;
    w = (w1*h) / h1;

    dispImg.create(Size(100 + 2*w, 100 + h), CV_8UC3);

    Mat imgROI = dispImg(Rect(20, 80, w, h));
    resize(img1, imgROI, Size(w, h), 0, 0, CV_INTER_LINEAR);

    imgROI = dispImg(Rect(40 + w, 80, w, h));
    resize(img2, imgROI, Size(w, h), 0, 0, CV_INTER_LINEAR);

    putText(dispImg, message, Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 23, 0), 4, 8);

    imshow(WindowName, dispImg);
}

CameraController camera;

bool isCapturingFinish = false;
void callback_single();
void callback_single()
{
	//std::cout << "callback() : I'm called!" << std::endl;
	isCapturingFinish = true;
}

int main()
{
    //Calibration Prepare
    vector<cv::Point2f> corners_L, corners_R;
    Size board_size = Size(11, 8); // checkboard corner size
    bool isSuccess;
    int POS_NUM = 0;
    int POS_NUM_all = 11;
    string calib_message;
    Mat greenAreaLeft, greenAreaRight;

    KJCalibration KJcalib(11, 8, 15); // calib checkboard - corner number and size

    KJcalib.setPositionNums(POS_NUM_all); // calib image number

    Mat  frame_L, frame_R, L_see, R_see,frame_LN,frame_RN;

    //Rectify prepare
    int flag_rectify = 0;
    int flag_corner_depth = 0;
    cv::Mat MX1, MY1, MX2, MY2;
    cv::Mat M1, M2, D1, D2, T, R;
    cv::Mat Rl, Rr, Pl, Pr;
    cv::Mat QP;
    Mat view_diff(4, 1, CV_64FC1),world(4, 1, CV_64FC1);
    //Camera Initia
	int PatternNum = 4;
    int camera_W,camera_H;

    if (SE_STATUS_SUCCESS != camera.init()) 
    {
        system("pause");
        return -1;
    }
	camera.setExposureTime2D(3000);
	camera.setExposureTime3D(20000);

	camera.registerCallBackProcess(callback_single);
    camera.setPatternsNum(1);
	camera.setCapture3DModelFinishFlag(false);
    camera.setTriggerSource(0);

    cout  << camera.getSensorWidth() <<endl;
    cout << camera.getSensorHeight() << endl;
    camera_W = camera.getSensorWidth();
    camera_H = camera.getSensorHeight();

    greenAreaLeft.create(camera_H, camera_W, CV_8UC3);
    greenAreaLeft.setTo(Scalar(0, 0, 0));
    greenAreaRight.create(camera_H, camera_W, CV_8UC3);
    greenAreaRight.setTo(Scalar(0, 0, 0));

	if (SE_STATUS_SUCCESS != camera.start(true))
	{
		std::cout << "camera.start() failed" << endl;
		return -1;
	}

    while (1)
    {

        camera.softTrigger();

        while (false == isCapturingFinish)
        {
            waitKey(20);
            //cout << "not finish" << endl;
        }

        char key = waitKey(100); // for button press time

        std::vector<cv::Mat> leftImages, rightImages;
        camera.processCaptureImages(leftImages, rightImages);

   /*     ResizeShow(camera.src[0], "L");
        ResizeShow(camera.srcR[0], "R");*/


        frame_L = leftImages[0].clone(); // frame is your want image
        frame_R = rightImages[0].clone(); // frame is your want image

        frame_LN = leftImages[0].clone();
        frame_RN = rightImages[0].clone();

        cvtColor(frame_L, frame_L, CV_GRAY2BGR);
        cvtColor(frame_R, frame_R, CV_GRAY2BGR);
        // draw corners for chessboard
        bool found_L = findChessboardCorners(frame_L, board_size, corners_L, CALIB_CB_FAST_CHECK);
        bool found_R = findChessboardCorners(frame_R, board_size, corners_R, CALIB_CB_FAST_CHECK);
        if (found_L && found_R)
        {
            drawChessboardCorners(frame_L, board_size, corners_L, found_L);
            drawChessboardCorners(frame_R, board_size, corners_R, found_R);
        }
        else
        {
            //cout << "not found!" << endl;
        }
        L_see = frame_L + greenAreaLeft;
        R_see = frame_R + greenAreaRight;


        if (key == 27) //ESC button for quit
        {
            break;
        }
        if (key == 32) // Space button for capture one Image
        {
            isSuccess = KJcalib.setOnePosition(frame_LN, frame_RN);
            if (true == isSuccess)
            {
                KJcalib.setGreenArea(frame_LN, greenAreaLeft);
                KJcalib.setGreenArea(frame_RN, greenAreaRight);
                POS_NUM++;
                cout << "Positions obtained :  " << POS_NUM << "/" << POS_NUM_all << endl;
                calib_message = "Positions obtained :  " + to_string(POS_NUM) + "/" + to_string(POS_NUM_all);
            }
            else
            {
                cout << "Invalid, Positions obtained :  " << POS_NUM << "/" << POS_NUM_all << endl;
                calib_message = "Invalid, Positions obtained :  " + to_string(POS_NUM) + "/" + to_string(POS_NUM_all);
            }
        }
        if (key == 97) // 'a' button for calibrate
        {
            KJcalib.calibrate();
        }
        if (key == 113)// 'q' button for stereorectify
        {
            cv::FileStorage fs;
            if (fs.open("result.yml", cv::FileStorage::READ))
            {
                fs["camL_K"] >> M1;
                fs["camL_kc"] >> D1;
                fs["camR_K"] >> M2;
                fs["camR_kc"] >> D2;
                fs["R"] >> R;
                fs["T"] >> T;
                fs.release();
                stereoRectify(M1, D1, M2, D2, Size(camera_W, camera_H), R, T, Rl, Rr, Pl, Pr, QP, 0);
                flag_rectify = 1;
                fx_d = QP.at<double>(2, 3);  // f
                px_d = -QP.at<double>(0, 3); // u0
                py_d = -QP.at<double>(1, 3); // v0
                baseline = 1 / QP.at<double>(3, 2); // -Tx
                offset = - (QP.at<double>(3, 3) / QP.at<double>(3, 2)); //u0-u0'
                cout << QP << endl;
                cout << fx_d << endl;
                cout << px_d << endl;
                cout << py_d << endl;
                cout << baseline << endl;
                cout << offset << endl;
            }
            else
            {
                calib_message = "Open File error!";
            }
        }

        if (key == 119)// 'w' button for corner depth caliculate
        {
            if (flag_rectify > 0)
            {
                flag_corner_depth = 1;
            }
            else
            {
                calib_message = "Please stereorectify first !";
            }
        }


        if (flag_rectify > 0)
        {
            initUndistortRectifyMap(M1, D1, Rl, Pl, frame_LN.size(), CV_32FC1, MX1, MY1);
            initUndistortRectifyMap(M2, D2, Rr, Pr, frame_RN.size(), CV_32FC1, MX2, MY2);
            remap(frame_LN, frame_LN, MX1, MY1, cv::INTER_LINEAR);
            remap(frame_RN, frame_RN, MX2, MY2, cv::INTER_LINEAR);
            cvtColor(frame_LN, frame_LN, CV_GRAY2BGR);
            cvtColor(frame_RN, frame_RN, CV_GRAY2BGR);
            L_see = frame_LN.clone();
            R_see = frame_RN.clone();
            L_see.convertTo(L_see, CV_8UC3);
            R_see.convertTo(R_see, CV_8UC3);
            calib_message = "Stereorectify ing!";

            if (flag_corner_depth > 0)
            {
                bool found_L = findChessboardCorners(L_see, board_size, corners_L, CALIB_CB_FAST_CHECK);
                bool found_R = findChessboardCorners(R_see, board_size, corners_R, CALIB_CB_FAST_CHECK);
                if (found_L && found_R)
                {
                    drawChessboardCorners(L_see, board_size, corners_L, found_L);
                    drawChessboardCorners(R_see, board_size, corners_R, found_R);

               /*     cout << QP << endl;
                    cout << view_diff << endl;
                    cout << world << endl;*/

                    Calculate_world_coord(corners_L[0].x, corners_L[0].y, corners_L[0].x - corners_R[0].x);
                    float a = w_x, b = w_y, c = w_z;
                    Calculate_world_coord(corners_L[1].x, corners_L[1].y, corners_L[1].x - corners_R[1].x);
                    cout << w_x << endl;
                    cout << w_y << endl;
                    cout << w_z << endl;

                    cout << sqrt((a - w_x)*(a - w_x) + (b - w_y)*(b - w_y) + (c - w_z)*(c - w_z)) << endl;
                }
            }
        }




        namedWindow("Video_LR", 1);
        //namedWindow("Video_L", 1);
        /*ResizeShow(L_see, "Video_L");*/
       // namedWindow("Video_R", 1);
        /*ResizeShow(R_see, "Video_R");*/

        ResizeShow_two(L_see, R_see, "Video_LR", calib_message);

        waitKey(20);
        isCapturingFinish = false;
        camera.setCapture3DModelFinishFlag(false);
        camera.clearImages();

    }
	cout << "Success!" << endl;
	system("pause");
	return 0;
}
