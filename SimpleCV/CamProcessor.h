#pragma once
#include <OpenNI.h>
#include <opencv2/opencv.hpp>

#include "oscpkt/oscpkt.hh"
#include "oscpkt/udp.hh"

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

	cv::Mat m_erosion_dst;
	cv::Mat im_with_keypoints;
	cv::Mat m_depth8Channel;
	cv::Mat m_inpainted;
	cv::Mat m_small_depth, m_tmp;
	cv::Mat m_rectElement1, m_rectElement2;
	cv::Mat m_depthMinusBackground;
	cv::Mat m_drawingMat;

	oscpkt::PacketWriter m_packetWriter;
	oscpkt::Message m_oscMessage;

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	int learnTime;
	int timePassed;

	const int portNumber = 9109;
	oscpkt::UdpSocket sock;
};

