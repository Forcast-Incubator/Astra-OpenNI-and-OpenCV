#pragma once

#pragma once
#include <OpenNI.h>
#include <opencv2/opencv.hpp>

#include "oscpkt/oscpkt.hh"
#include "oscpkt/udp.hh"
#include "OniSampleUtilities.h"

using namespace std;
using namespace cv;
using namespace openni;

class OpticalFlow
{
public:
	OpticalFlow(Device& device, VideoStream& depth);
	~OpticalFlow();

	Status Initialise();
	void Start();
	void MainLoop();
	void Display();

	VideoFrameRef	m_depthFrame;
	Device&			m_device;
	VideoStream&	m_depthStream;
	VideoStream**	m_streams;
};

