#pragma once
#include <OpenNI.h>
#include <opencv2/opencv.hpp>

#include "oscpkt/oscpkt.hh"
#include "oscpkt/udp.hh"

using namespace cv;
using namespace openni;
using namespace std;
using namespace oscpkt;

class CamProcessor
{
public:
	CamProcessor(Device& device, VideoStream& depth);
	~CamProcessor();

	virtual Status init(int argc, char **argv);
	virtual Status run();

	virtual void display();

	VideoFrameRef	m_depthFrame;
	Device&			m_device;
	VideoStream&	m_depthStream;
	VideoStream**	m_streams;

private:
	const DepthPixel* m_depthPixelBuffer;

	Mat m_cvDepthImage;

	vector<Point2f> m_features_prev, m_features_next;
	int m_numFeaturesX, m_numFeaturesY;
	bool m_intialised;

	Mat m_displayImg;
	Mat m_scaledDepthImg;
	Mat m_inpaintedImg;
	Mat m_tempDepthImg, m_tmp;
	Mat m_rectElement1, m_rectElement2;
	Mat m_depthMinusBackground;
	Mat m_drawingMat;
	Mat m_imgPrev;

	oscpkt::PacketWriter m_packetWriter;
	oscpkt::Message m_oscMessage;

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	int m_resetTimeInterval;
	int m_timePassed;

	const int portNumber = 9109;
	oscpkt::UdpSocket sock;
};

