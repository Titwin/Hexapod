#include <iostream>
#include <aruco/aruco.h>
#include <opencv2/opencv.hpp>

#include "UDP.hpp"


using namespace cv;

#define UDP_SEND_PORT "5016"
#define UDP_STREAM_PORT "5017"
#define VIDEO_STREAM true
#define UDP_HOSTNAME "Thibault-SED-PC.local"
#define UDP_SCAN_FRAME 50

cv::Mat getRTMatrix(const cv::Mat &_rvec, const cv::Mat &_tvec);
void getQTFromMatrix4x4(const cv::Mat &M, cv::Mat &Q, cv::Mat &T);

bool quit = false;

int main()
{
	/// message service initialization
	UDPsocket UDP(5015);
	std::string computerIP;
	UDP.getIpFromHostname(UDP_HOSTNAME, &computerIP);
	
	///	aruco initialization
    aruco::CameraParameters CamParam; 					    // intrinsic camera's parameters
    aruco::MarkerDetector MDetector;  					    // config regarding detection of markers
    float markerSize = 0.052f; 							    // marker size
	
    MDetector.loadParamsFromFile("arucoConfig.yml");        // Marker detection parameters
	CamParam.readFromXMLFile("raspicam1_1280x720.yml"); 	// Camera intrinsic and distortion coeeficients
	//https://github.com/UbiquityRobotics/raspicam_node/tree/kinetic/camera_info
	
    std::vector<aruco::Marker> Markers;						// all reconized marker in the image frame
    std::map<uint32_t, aruco::MarkerPoseTracker> MTracker;  // use a map so that for each id, we use a different pose tracker
	
	/// frame buffers initialization
    cv::Mat InImage;
	std::vector<uint8_t> cframe;
	unsigned long long loopCount = 0;
	std::chrono::time_point<std::chrono::system_clock> loopingTime;
	
	/// encoding parameters
	std::vector<int> param;
	param.push_back(CV_IMWRITE_JPEG_QUALITY);
	param.push_back(85);
	
	/// video initialization
    VideoCapture cap(0);
    cap.set(CV_CAP_PROP_FPS, 90);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
	
    if (cap.isOpened()) cap >> InImage;
    else {
        std::cerr << "Could not open input" << std::endl;
        return -1;
    }
	
	///	program
    while(!quit)
	{
		/// begin
		loopingTime = std::chrono::high_resolution_clock::now();
		for(int i = 0; i < 10; i++)
			cap.grab();
        cap >> InImage;
        Markers = MDetector.detect(InImage);

		/// compute message
		std::string s;
		for (auto& marker : Markers)
		{
            MTracker[marker.id].estimatePose(marker, CamParam, markerSize);  // call its tracker and estimate the pose
			s += std::to_string(marker.id) + " : ";
			
			cv::Mat Q, T;
			cv::Mat r = MTracker[marker.id].getRvec();   r.convertTo(r, CV_32F);
			cv::Mat t = MTracker[marker.id].getTvec();   t.convertTo(t, CV_32F);
			
			if(r.checkVector(1) != 3 || t.checkVector(1) != 3)
				continue;
			
			getQTFromMatrix4x4(getRTMatrix(r, t), Q, T);
			
			s += std::to_string(T.at<float>(0)) + ' ';
			s += std::to_string(T.at<float>(1)) + ' ';
			s += std::to_string(T.at<float>(2)) + ' ';
			
			s += std::to_string(Q.at<float>(0)) + ' ';
			s += std::to_string(Q.at<float>(1)) + ' ';
			s += std::to_string(Q.at<float>(2)) + ' ';
			s += std::to_string(Q.at<float>(3)) + '\n';
        }
		std::cout << s << std::endl;
		
		///	send message
		UDP.sendMessageTo(0, (const uint8_t*)s.c_str(), s.size(), "127.0.0.1", UDP_SEND_PORT);
		if((loopCount%UDP_SCAN_FRAME) == 0)
            UDP.getIpFromHostname(UDP_HOSTNAME, &computerIP, (computerIP.empty() ? -1 : 1));
		if(!computerIP.empty() && VIDEO_STREAM)
		{
			UDP.sendMessageTo(0, (const uint8_t*)s.c_str(), s.size(), computerIP.c_str(), UDP_SEND_PORT);
			
			for (auto& marker : Markers)
				marker.draw(InImage, Scalar(255, 0, 255));
			
			cv::resize(InImage, InImage, cv::Size(), 0.5, 0.5);
			cv::imencode(".jpg", InImage, cframe, param);
			UDP.sendMessageTo(0, (const uint8_t*)cframe.data(), cframe.size(), computerIP.c_str(), UDP_STREAM_PORT);
		}
		
		/// end
        loopCount++;
		UDP.incrementTimestamp();
		std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - loopingTime).count() << "ms" << std::endl << std::endl;
    }
	
	///	quit
    return 0;
}

cv::Mat getRTMatrix(const cv::Mat &_rvec, const cv::Mat &_tvec)
{
	cv::Mat Matrix(4, 4, CV_32F);
	float *rt_44 = Matrix.ptr<float>(0);
	
	//makes a fast conversion to the 4x4 array passed
	float rx = _rvec.ptr<float>(0)[0];
	float ry = _rvec.ptr<float>(0)[1];
	float rz = _rvec.ptr<float>(0)[2];
	float tx = _tvec.ptr<float>(0)[0];
	float ty = _tvec.ptr<float>(0)[1];
	float tz = _tvec.ptr<float>(0)[2];
	
	float nsqa = rx * rx + ry * ry + rz * rz;
	float a = std::sqrt(nsqa);
	float i_a = static_cast<float>(a ? 1. / a : 0);
	
	float rnx = rx * i_a;
	float rny = ry * i_a;
	float rnz = rz * i_a;
	float cos_a = cos(a);
	float sin_a = sin(a);
	float _1_cos_a = static_cast<float>(1. - cos_a);
	
	rt_44[0] = cos_a + rnx * rnx * _1_cos_a;
	rt_44[1] = rnx * rny * _1_cos_a - rnz * sin_a;
	rt_44[2] = rny * sin_a + rnx * rnz * _1_cos_a;
	rt_44[3] = tx;
	
	rt_44[4] = rnz * sin_a + rnx * rny * _1_cos_a;
	rt_44[5] = cos_a + rny * rny * _1_cos_a;
	rt_44[6] = -rnx * sin_a + rny * rnz * _1_cos_a;
	rt_44[7] = ty;
	
	rt_44[8] = -rny * sin_a + rnx * rnz * _1_cos_a;
	rt_44[9] = rnx * sin_a + rny * rnz * _1_cos_a;
	rt_44[10] = cos_a + rnz * rnz * _1_cos_a;
	rt_44[11] = tz;
	
	rt_44[12] = rt_44[13] = rt_44[14] = 0;
	
	rt_44[15] = 1;
	
	return Matrix;
}

void getQTFromMatrix4x4(const cv::Mat &M, cv::Mat &Q, cv::Mat &T)
{
	// extract the rotation part
    //cv::Mat r33 = cv::Mat(M, cv::Rect(0, 0, 3, 3));
    //cv::SVD svd(r33);
    //cv::Mat Rpure = svd.u * svd.vt;

    /* qw= √(1 + m00 + m11 + m22) /2
     * qx = (m21 - m12)/( 4 *qw)
     * qy = (m02 - m20)/( 4 *qw)
     * qz = (m10 - m01)/( 4 *qw)
     */
    Q.create(1, 4, M.type());
    Q.ptr<float>(0)[3] = sqrt(1 + M.at<float>(0, 0) + M.at<float>(1, 1) + M.at<float>(2, 2))/2;
    Q.ptr<float>(0)[0] = (M.at<float>(2, 1) - M.at<float>(1, 2))/(4*Q.ptr<float>(0)[3]);
    Q.ptr<float>(0)[1] = (M.at<float>(0, 2) - M.at<float>(2, 0))/(4*Q.ptr<float>(0)[3]);
    Q.ptr<float>(0)[2] = (M.at<float>(1, 0) - M.at<float>(0, 1))/(4*Q.ptr<float>(0)[3]);


    T.create(1, 3, M.type());
    if (M.type() == CV_32F)
        for (int i = 0; i < 3; i++)
            T.ptr<float>(0)[i] = M.at<float>(i, 3);
    else
        for (int i = 0; i < 3; i++)
            T.ptr<double>(0)[i] = M.at<double>(i, 3);
}