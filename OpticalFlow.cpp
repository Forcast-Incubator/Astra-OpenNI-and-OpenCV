#include "OpticalFlow.h"

/** CONSTRUCTOR **/
OpticalFlow::OpticalFlow(Device& device, VideoStream& depth) : m_device(device), m_depthStream(depth)
{

}


Status OpticalFlow::Initialise()
{
	VideoMode depthVideoMode;

	if (m_depthStream.isValid())
	{
		depthVideoMode = m_depthStream.getVideoMode();
	}
	else {
		printf("Depth stream not valid\n");
		return openni::STATUS_ERROR;
	}

	// OpenNI stream to grab depth data
	m_streams = new VideoStream*[1];
	m_streams[0] = &m_depthStream;
}

void OpticalFlow::Start()
{

}

void OpticalFlow::MainLoop()
{
	// GRAB DEPTH FRAME

	// INPAINT MISSING DEPTH DATA

	// CALCULATE OPTICAL FLOW

	// COMBINE TO FINAL MAT

	// PACKAGE FOR OSC

	// SEND OVER OSC
}

void OpticalFlow::Display()
{

}


/** DESTRUCTOR **/
OpticalFlow::~OpticalFlow()
{

}