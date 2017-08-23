// Undeprecate CRT functions
#ifndef _CRT_SECURE_NO_DEPRECATE 
#define _CRT_SECURE_NO_DEPRECATE 1
#endif

#ifndef OSCPKT_OSTREAM_OUTPUT
#define OSCPKT_OSTREAM_OUTPUT
#endif

#include "CamProcessor.h"
#include "OniSampleUtilities.h"

#define GL_WIN_SIZE_X	1024
#define GL_WIN_SIZE_Y	512
#define TEXTURE_SIZE	512

#define ASTRA_DEPTH_WIDTH 640
#define ASTRA_DEPTH_HEIGHT 480

#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_DEPTH

#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

#define ASTRA_MAX_RANGE 10000
#define ASTRA_MIN_RANGE 600

CamProcessor::CamProcessor(openni::Device& device, openni::VideoStream& depth) : m_device(device), m_depthStream(depth)
{
	// assign references to depth stream and device to member variables
}


CamProcessor::~CamProcessor()
{
	// destructor makes sure all streams and buffers are correctly deleted
	delete[] depthBuffer;
	if (m_streams != NULL)
	{
		delete[]m_streams;
	}
}


openni::Status CamProcessor::init(int argc, char **argv)
{
	openni::VideoMode depthVideoMode;
	initialised = false;

	if (m_depthStream.isValid())
	{
		// store video mode
		depthVideoMode = m_depthStream.getVideoMode();

		// Initialise background subtractor
		MOG2BackgroundSubtractor = cv::createBackgroundSubtractorMOG2();
		MOG2BackgroundSubtractor->setNMixtures(5);			//number of gaussian components in the background model
		MOG2BackgroundSubtractor->setHistory(200);			//number of last frames that affect the background model
		MOG2BackgroundSubtractor->setVarThreshold(1000);	//variance threshold for the pixel-model match
		MOG2BackgroundSubtractor->setDetectShadows(false);	//don't mark shadows (only useful for color frames)

		// initialise time variables used for updating background model
		timePassed = 0;
		learnTime = 10;
	}
	else {
		printf("Depth stream not valid\n");
		return openni::STATUS_ERROR;
	}

	// set up OpenNI streams
	m_streams = new openni::VideoStream*[1];
	m_streams[0] = &m_depthStream;

	// store depth buffer size
	depthBufferX = MIN_CHUNKS_SIZE(depthVideoMode.getResolutionX(), TEXTURE_SIZE);
	depthBufferY = MIN_CHUNKS_SIZE(depthVideoMode.getResolutionY(), TEXTURE_SIZE);

	// initialise openNI version of \ depth buffer
	depthBuffer = new openni::DepthPixel[depthBufferX * depthBufferY];

	int kernel_size = 5;

	m_rectElement1 = cv::getStructuringElement(cv::MORPH_RECT,
		cv::Size(2 * kernel_size + 1, 2 * kernel_size + 1),
		cv::Point(kernel_size, kernel_size));

	kernel_size = 10;

	m_rectElement2 = cv::getStructuringElement(cv::MORPH_RECT,
		cv::Size(2 * kernel_size + 1, 2 * kernel_size + 1),
		cv::Point(kernel_size, kernel_size));

	// OPTICAL FLOW SET UP

	numFeaturesX = 30;
	numFeaturesY = 20;
	for (int i = 0; i < numFeaturesX; i++)
	{
		for (int j = 0; j < numFeaturesY; j++)
		{
			features_prev.push_back(
				cv::Point2f((ASTRA_DEPTH_WIDTH /numFeaturesX) * i + (ASTRA_DEPTH_WIDTH /(numFeaturesX*2)), 
				(ASTRA_DEPTH_HEIGHT / numFeaturesY) * j + (ASTRA_DEPTH_HEIGHT /(numFeaturesY*2))));

		}
	}

	cv::namedWindow("KeyPoints", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
	cv::resizeWindow("KeyPoints", 640, 480);

	return openni::STATUS_OK;
}

openni::Status CamProcessor::run()
{
	sock.connectTo("localhost", portNumber);
	if (!sock.isOk()) {
		std::cerr << "Error connection to port " << portNumber << ": " << sock.errorMessage() << "\n";
	}
	else {
		std::cout << "Client started, will send packets to port " << portNumber << std::endl;

		while (sock.isOk()) {
			display();
			//cv::imshow("Depth", m_cvDepthImage);
			//cv::imshow("Background Subtraction", m_foregroundMaskMOG2);
			//cv::imshow("Depth Estimation", im_with_keypoints);

			cv::imshow("KeyPoints", im_with_keypoints);
			cv::waitKey(1);
		}

		std::cout << "sock error: " << sock.errorMessage() << " -- is the server running?\n";
	}

	return openni::STATUS_OK;
}

void CamProcessor::display()
{
	int changedIndex;
	openni::Status rc = openni::OpenNI::waitForAnyStream(m_streams, 1, &changedIndex);
	
	if (rc != openni::STATUS_OK)
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
		depthBuffer = (const openni::DepthPixel*)m_depthFrame.getData();

		m_cvDepthImage.create(m_depthFrame.getHeight(), m_depthFrame.getWidth(), CV_16UC1);
		memcpy(m_cvDepthImage.data, depthBuffer, m_depthFrame.getHeight()*m_depthFrame.getWidth() * sizeof(uint16_t));

		/***** IN-PAINTING ******/
		m_cvDepthImage.convertTo(m_depth8Channel, CV_8UC1, 255.0f / (ASTRA_MAX_RANGE - ASTRA_MIN_RANGE), 0);

		cv::resize(m_depth8Channel, m_small_depth, cv::Size(), 0.2, 0.2);
		
		//inpaint only the "unknown" pixels
		cv::inpaint(m_small_depth, (m_small_depth == 0), m_small_depth, 5.0, cv::INPAINT_TELEA);

		cv::resize(m_small_depth, m_tmp, m_depth8Channel.size());
		m_inpainted = m_depth8Channel;
		m_tmp.copyTo(m_inpainted, (m_depth8Channel == 0));  //add the original signal back over the inpaint

		/***** BACKGROUND SUBTRACTION ******/
		/*
		if (learnTime > timePassed) {
			MOG2BackgroundSubtractor->apply(m_cvDepthImage, m_foregroundMaskMOG2, 0.5);
		}
		else {
			MOG2BackgroundSubtractor->apply(m_cvDepthImage, m_foregroundMaskMOG2, 0);
		}
		

		// EROSION AND DILATION
		cv::erode(m_foregroundMaskMOG2, m_erosion_dst, m_rectElement1);

		cv::dilate(m_erosion_dst, m_erosion_dst, m_rectElement2);
		cv::erode(m_erosion_dst, m_erosion_dst, m_rectElement1);
		*/

		m_cvDepthImage.convertTo(m_cvDepthImage, CV_8U, 255.0f / (ASTRA_MAX_RANGE - ASTRA_MIN_RANGE), 0);

		// OPTICAL FLOW
		std::vector<uchar> status;
		std::vector<float> err;

		if (timePassed > learnTime) {
			features_prev.clear();
			for (int i = 0; i < numFeaturesX; i++)
			{
				for (int j = 0; j < numFeaturesY; j++)
				{
					features_prev.push_back(cv::Point2f((ASTRA_DEPTH_WIDTH / numFeaturesX) * i + (ASTRA_DEPTH_WIDTH / (numFeaturesX * 2)),
						(ASTRA_DEPTH_HEIGHT / numFeaturesY) * j + (ASTRA_DEPTH_HEIGHT / (numFeaturesY * 2))));
				}
			}
			timePassed = 0;
			
		}
		if(initialised == false)
		{
			initialised = true;
			m_imgPrev = m_inpainted.clone();
		}

		cv::calcOpticalFlowPyrLK(m_imgPrev, m_inpainted, features_prev, features_next, status, err,cv::Size(9,9));
		m_imgPrev = m_inpainted.clone();
		
		depth_features.clear();
		for (int i = 0; i < features_next.size(); i++)
		{
			cv::Scalar colour;
			if (features_next[i].x >= ASTRA_DEPTH_WIDTH || features_next[i].x <= 0|| features_next[i].y >= ASTRA_DEPTH_HEIGHT || features_next[i].y <= 0)
			{
				colour = cv::Scalar(5);
			}
			else {
				colour = m_cvDepthImage.at<uchar>(cv::Point((int)features_next[i].x, (int)features_next[i].y));
			}
			depth_features.push_back(cv::Point3f(features_next[i].x, features_next[i].y, colour.val[0]));
		}

		/*
		// CONTOURS
		cv::findContours(m_erosion_dst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		
		std::vector<int> contourDepths(contours.size());
		std::vector<int> contourSizes(contours.size());

		m_depthMinusBackground = m_inpainted - (255-m_erosion_dst);

		m_drawingMat = cv::Mat::zeros(m_cvDepthImage.size(), CV_8UC3);

		for (int i = 0; i< contours.size(); i++)
		{
			cv::Mat thisContour = cv::Mat::zeros(m_cvDepthImage.size(), CV_8U);

			cv::Scalar color = cv::Scalar(255,255,255);
			cv::drawContours(thisContour, contours, i, color, -1, 8, hierarchy, 0, cv::Point());

			int histSize = 256;

			/// Set the ranges ( for B,G,R) )
			float range[] = { 0, 256 };
			const float* histRange = { range };
			bool uniform = true;
			bool accumulate = false;

			cv::Mat hist;
			cv::calcHist(&m_depthMinusBackground, 1, 0, thisContour, hist, 1, &histSize, &histRange, uniform, accumulate);

			double min, max;
			int minIdX[2], maxIdX[2];
			cv::minMaxIdx(hist, &min, &max, minIdX, maxIdX);
			contourDepths[i] = maxIdX[0];
			contourSizes[i] = max;
			
			if (contourDepths[i] > 0 && contourSizes[i] > 200) {
				cv::drawContours(m_drawingMat, contours, i, cv::Scalar(255-maxIdX[0]*2, maxIdX[0], 255), 1, 8, hierarchy, 0, cv::Point());
			}

		}
		

		std::vector<cv::Moments> mu(contours.size());

		for (int i = 0; i < contours.size(); i++)

		{
			mu[i] = moments(contours[i], false);
		}

		//Get the mass centers (image has multiple contours):

		std::vector<cv::Point2f> mc(contours.size());

		for (int i = 0; i < contours.size(); i++)

		{
			mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
		}

		// HISTOGRAM

		m_erosion_dst = m_cvDepthImage - (255 - m_erosion_dst);
		applyColorMap(m_erosion_dst, m_erosion_dst, cv::COLORMAP_HSV);

		im_with_keypoints = m_erosion_dst + m_drawingMat;

		for (int i = 0; i < mc.size(); i++)
		{
			if (contourDepths[i] > 0 && contourSizes[i] > 200) {
				cv::drawMarker(im_with_keypoints, mc[i], cv::Scalar(255, 255, 255), 0, 20, 1, 8);
				cv::putText(im_with_keypoints, std::to_string(contourDepths[i]), mc[i], cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));

				// send X
				std::stringstream msgText;
				msgText << "/contour/" << i << "/x";
				m_oscMessage = msgText.str();
				m_oscMessage.pushFloat(mc[i].x);
				
				m_packetWriter.startBundle().startBundle().addMessage(m_oscMessage);

				// send Y
				msgText.str(std::string());
				msgText << "/contour/" << i << "/y";
				m_oscMessage = msgText.str();
				m_oscMessage.pushFloat(mc[i].y);
				m_packetWriter.addMessage(m_oscMessage);

				// send Z
				msgText.str(std::string());
				msgText << "/contour/" << i << "/z";
				m_oscMessage = msgText.str();
				m_oscMessage.pushInt32(contourDepths[i]);
				m_packetWriter.addMessage(m_oscMessage).endBundle().endBundle();

				bool ok = sock.sendPacket(m_packetWriter.packetData(), m_packetWriter.packetSize());
				//std::cout << "Client: sent /contour/" << i << ", ok: " << ok << std::endl;
			}
		}
		*/

		im_with_keypoints = m_inpainted.clone();

		for (int i = 0; i < features_next.size(); i++)
		{
			
			int markerSize = (255 / (depth_features[i].z));
			if (markerSize < 100 && markerSize > 0) {
				if (depth_features[i].z < 50 && depth_features[i].z >0) {
					

					if (timePassed > 0) {
						cv::Point diff = features_prev[i] - features_next[i];
						
						float mag = cv::sqrt(diff.x*diff.x + diff.y*diff.y);
						if (mag < 50.0f && mag > 0.0f)
						{
							cv::drawMarker(im_with_keypoints, features_next[i], cv::Scalar(255, 255, 255), cv::MARKER_DIAMOND, markerSize, 1, 0);
							cv::line(im_with_keypoints, features_prev[i], features_next[i], cv::Scalar(255, 255, 255));
						}
					}
				}
			}
		}

		timePassed++;
		features_prev = features_next;
	}
}


