// Suppress warnings for deprecated/unsafe C functions
#ifndef _CRT_SECURE_NO_DEPRECATE 
#define _CRT_SECURE_NO_DEPRECATE 1
#endif

#ifndef OSCPKT_OSTREAM_OUTPUT
#define OSCPKT_OSTREAM_OUTPUT
#endif

#include "CamProcessor.h"

#define WIN_SIZE_X	640
#define WIN_SIZE_Y	480

#define ASTRA_FEED_X	640
#define ASTRA_FEED_Y	480

#define ASTRA_MAX_RANGE	10000
#define ASTRA_MIN_RANGE	600

CamProcessor::CamProcessor(Device& device, VideoStream& depth) : m_device(device), m_depthStream(depth)
{
	// assign references to depth stream and device to member variables
}

CamProcessor::~CamProcessor()
{
	// destructor makes sure all streams and buffers are correctly deleted
	delete[] m_depthPixelBuffer;
	if (m_streams != NULL)
	{
		delete[]m_streams;
	}
}


Status CamProcessor::init(int argc, char **argv)
{
	VideoMode depthVideoMode;
	m_intialised = false;

	if (m_depthStream.isValid())
	{
		// store video mode
		depthVideoMode = m_depthStream.getVideoMode();

		// initialise time variables used for updating background model
		m_timePassed = 0;
		m_resetTimeInterval = 10;
	}
	else {
		printf("Depth stream not valid\n");
		return STATUS_ERROR;
	}

	// set up OpenNI streams
	m_streams = new VideoStream*[1];
	m_streams[0] = &m_depthStream;

	// initialise openNI version of \ depth buffer
	m_depthPixelBuffer = new DepthPixel[ASTRA_FEED_X * ASTRA_FEED_Y];

	// OPTICAL FLOW SET UP
	m_numFeaturesX = 30;
	m_numFeaturesY = 20;

	for (int i = 0; i < m_numFeaturesX * m_numFeaturesY; i++)
	{
		int x = i % m_numFeaturesX;
		int y = i / m_numFeaturesX;
		
		float xPos = (ASTRA_FEED_X / m_numFeaturesX) * x + (ASTRA_FEED_X / (m_numFeaturesX * 2));
		float yPos = (ASTRA_FEED_Y / m_numFeaturesY) * y + (ASTRA_FEED_Y / (m_numFeaturesY * 2));

		m_features_prev.push_back(Point2f(xPos, yPos));

	}

	namedWindow("KeyPoints", WINDOW_NORMAL | WINDOW_KEEPRATIO);
	resizeWindow("KeyPoints", WIN_SIZE_X, WIN_SIZE_Y);

	return STATUS_OK;
}

Status CamProcessor::run()
{
	socket.connectTo("localhost", portNumber);
	if (!socket.isOk()) {
		cerr << "Error connection to port " << portNumber << ": " << socket.errorMessage() << "\n";
	}
	else {
		cout << "Client started, will send packets to port " << portNumber << endl;

		while (socket.isOk()) {
			display();

			imshow("KeyPoints", m_displayImg);
			waitKey(1);
		}

		cout << "sock error: " << socket.errorMessage() << " -- is the server running?\n";
	}

	return STATUS_OK;
}

void CamProcessor::display()
{
	int changedIndex;
	Status rc = OpenNI::waitForAnyStream(m_streams, 1, &changedIndex);
	
	if (rc != STATUS_OK)
	{
		printf("Wait failed\n");
		return;
	}

	switch (changedIndex)
	{
	case 0:
		m_depthStream.readFrame(&m_depthFrame); break;
	default:
		printf("Error in wait\n");
	}

	if (m_depthFrame.isValid())
	{
		m_depthPixelBuffer = (const DepthPixel*)m_depthFrame.getData();

		m_cvDepthImage.create(m_depthFrame.getHeight(), m_depthFrame.getWidth(), CV_16UC1);
		memcpy(m_cvDepthImage.data, m_depthPixelBuffer, m_depthFrame.getHeight()*m_depthFrame.getWidth() * sizeof(uint16_t));

		/***** IN-PAINTING ******/
		m_cvDepthImage.convertTo(m_scaledDepthImg, CV_8U, 256.0f / (ASTRA_MAX_RANGE - ASTRA_MIN_RANGE), 0);

		resize(m_scaledDepthImg, m_tempDepthImg, Size(), 0.2, 0.2);
		
		//inpaint only the "unknown" pixels
		inpaint(m_tempDepthImg, (m_tempDepthImg == 0), m_tempDepthImg, 1.0, INPAINT_TELEA);

		resize(m_tempDepthImg, m_tempDepthImg, m_scaledDepthImg.size());
		m_inpaintedImg = m_scaledDepthImg;
		m_tempDepthImg.copyTo(m_inpaintedImg, (m_scaledDepthImg == 0));  //add the original signal back over the inpaint

		blur(m_inpaintedImg, m_inpaintedImg, Size(10, 10));

		// OPTICAL FLOW
		vector<uchar> status;
		vector<float> err;

		if (m_timePassed > m_resetTimeInterval) {

			m_features_prev.clear();

			for (int i = 0; i < m_numFeaturesX * m_numFeaturesY; i++)
			{
				int x = i % m_numFeaturesX;
				int y = i / m_numFeaturesX;

				float xPos = (ASTRA_FEED_X / m_numFeaturesX) * x + (ASTRA_FEED_X / (m_numFeaturesX * 2));
				float yPos = (ASTRA_FEED_Y / m_numFeaturesY) * y + (ASTRA_FEED_Y / (m_numFeaturesY * 2));

				m_features_prev.push_back(Point2f(xPos, yPos));
			}

			m_timePassed = 0;
		}
		else {

			if (m_intialised == false)
			{
				m_intialised = true;
				m_imgPrev = m_inpaintedImg.clone();
			}

			calcOpticalFlowPyrLK(m_imgPrev, m_inpaintedImg, m_features_prev, m_features_next, status, err, Size(5, 5));

			m_imgPrev = m_inpaintedImg.clone();
			m_displayImg = m_inpaintedImg.clone();

			

			for (int i = 0; i < m_features_next.size(); i++)
			{
				uchar depth;
				if (m_features_next[i].x >= ASTRA_FEED_X || m_features_next[i].x <= 0 || m_features_next[i].y >= ASTRA_FEED_Y || m_features_next[i].y <= 0)
				{
					depth = 0;
				}
				else {
					depth = m_inpaintedImg.at<uchar>(Point((int)m_features_next[i].x, (int)m_features_next[i].y));
				}

				if (depth > 0 && depth < 50) {
					Point differenceVector = m_features_prev[i] - m_features_next[i];
					float squareMagnitude = differenceVector.x*differenceVector.x + differenceVector.y*differenceVector.y;

					if (squareMagnitude < (50 * 50) && squareMagnitude >(5 * 5))
					{
						drawMarker(m_displayImg, m_features_next[i], Scalar(255, 255, 255), MARKER_DIAMOND, 255 / depth, 1, 0);
						line(m_displayImg, m_features_prev[i], m_features_next[i], Scalar(255, 255, 255));

						m_oscMessage.init("/flow/");
						
						m_oscMessage.pushInt32(i);
						m_oscMessage.pushFloat(m_features_prev[i].x);
						m_oscMessage.pushFloat(m_features_prev[i].y);
						m_oscMessage.pushInt32(depth);
						m_oscMessage.pushFloat(m_features_next[i].x);
						m_oscMessage.pushFloat(m_features_next[i].y);
						m_oscMessage.pushInt32(depth);

						m_packetWriter.init();
						m_packetWriter.startBundle();
						m_packetWriter.addMessage(m_oscMessage); 
						m_packetWriter.endBundle();

						socket.sendPacket(m_packetWriter.packetData(), m_packetWriter.packetSize());
					}
				}
			}
			m_features_prev = m_features_next;
		}
		m_timePassed++;

		
	}
}


