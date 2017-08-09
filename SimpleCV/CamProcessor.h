#pragma once
#include <OpenNI.h>
#include <opencv2/opencv.hpp>

class CamProcessor
{
public:
	CamProcessor(openni::Device& device, openni::VideoStream& depth);
	~CamProcessor();

	virtual openni::Status init(int argc, char **argv);
	virtual openni::Status run();

	virtual void display();

	openni::VideoFrameRef	m_depthFrame;
	openni::Device&			m_device;
	openni::VideoStream&	m_depthStream;
	openni::VideoStream**	m_streams;

private:
	const openni::DepthPixel* depthBuffer;

	unsigned int depthBufferX;
	unsigned int depthBufferY;

	cv::Mat m_cvDepthImage;
	cv::Mat m_foregroundMaskMOG2;
	cv::Ptr<cv::BackgroundSubtractorMOG2> MOG2BackgroundSubtractor;
	cv::Ptr<cv::SimpleBlobDetector> blobDetector;

	cv::Mat erosion_dst;
	cv::Mat im_with_keypoints;

	int learnTime;
	int timePassed;
};

