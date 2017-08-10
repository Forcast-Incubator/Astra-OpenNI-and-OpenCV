// Undeprecate CRT functions
#ifndef _CRT_SECURE_NO_DEPRECATE 
#define _CRT_SECURE_NO_DEPRECATE 1
#endif

#include "CamProcessor.h"
#include "OniSampleUtilities.h"

#define GL_WIN_SIZE_X	1024
#define GL_WIN_SIZE_Y	768
#define TEXTURE_SIZE	512

#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_DEPTH

#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

#define ASTRA_MAX_RANGE 10000
#define ASTRA_MIN_RANGE 600

CamProcessor::CamProcessor(openni::Device& device, openni::VideoStream& depth) : m_device(device), m_depthStream(depth)
{

}


CamProcessor::~CamProcessor()
{
	delete[] depthBuffer;
	if (m_streams != NULL)
	{
		delete[]m_streams;
	}
}


openni::Status CamProcessor::init(int argc, char **argv)
{
	openni::VideoMode depthVideoMode;

	if (m_depthStream.isValid())
	{
		depthVideoMode = m_depthStream.getVideoMode();

		// Init OpenCV parameters
		MOG2BackgroundSubtractor = cv::createBackgroundSubtractorMOG2();
		MOG2BackgroundSubtractor->setBackgroundRatio(0.7);
		MOG2BackgroundSubtractor->setNMixtures(5);
		MOG2BackgroundSubtractor->setHistory(200);
		MOG2BackgroundSubtractor->setVarThreshold(1000);
		MOG2BackgroundSubtractor->setDetectShadows(false);


		cv::SimpleBlobDetector::Params blobDetectorParams;
		
		// Filter by colour
		blobDetectorParams.filterByColor = true;
		blobDetectorParams.blobColor = 255;

		// Change thresholds
		blobDetectorParams.minThreshold = 0;
		blobDetectorParams.maxThreshold = 1000;

		// Filter by Area.
		blobDetectorParams.filterByArea = true;
		blobDetectorParams.minArea = 100;
		blobDetectorParams.maxArea = 100000;

		// Filter by Circularity
		blobDetectorParams.filterByCircularity = false;
		blobDetectorParams.minCircularity = 0.1;

		// Filter by Convexity
		blobDetectorParams.filterByConvexity = false;
		blobDetectorParams.minConvexity = 0.87;

		// Filter by Inertia
		blobDetectorParams.filterByInertia = false;
		blobDetectorParams.minInertiaRatio = 0.01;

		blobDetector = cv::SimpleBlobDetector::create(blobDetectorParams);

		timePassed = 0;
		learnTime = 10;
	}
	else {
		printf("Depth stream not valid\n");
		return openni::STATUS_ERROR;
	}

	m_streams = new openni::VideoStream*[1];
	m_streams[0] = &m_depthStream;

	depthBufferX = MIN_CHUNKS_SIZE(depthVideoMode.getResolutionX(), TEXTURE_SIZE);
	depthBufferY = MIN_CHUNKS_SIZE(depthVideoMode.getResolutionY(), TEXTURE_SIZE);

	depthBuffer = new openni::DepthPixel[depthBufferX * depthBufferY];

	return openni::STATUS_OK;
}

openni::Status CamProcessor::run()
{
	while (true) {
		display();
		//cv::imshow("Depth", m_cvDepthImage);
		//cv::imshow("Background Subtraction", m_foregroundMaskMOG2);
		cv::imshow("Erosion", im_with_keypoints);
		
		//cv::imshow("KeyPoints", im_with_keypoints);
		cv::waitKey(1);
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

		if (learnTime > timePassed) {
			timePassed++;
			MOG2BackgroundSubtractor->apply(m_cvDepthImage, m_foregroundMaskMOG2, 0.5);
		}
		else {
			MOG2BackgroundSubtractor->apply(m_cvDepthImage, m_foregroundMaskMOG2, 0);
		}
		
		//cv::Mat image8uc1;
		//m_foregroundMaskMOG2.convertTo(image8uc1, CV_8U, 1); // CV_8U should work as well
		
		//m_cvDepthImage = 8*(8000-m_cvDepthImage);

		// EROSION AND DILATION
		int erosion_size = 5;

		cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
			cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
			cv::Point(erosion_size, erosion_size));

		cv::erode(m_foregroundMaskMOG2, erosion_dst, element);
		//cv::erode(m_cvDepthImage, m_cvDepthImage, element);
		cv::dilate(erosion_dst, erosion_dst, element);

		m_cvDepthImage.convertTo(m_cvDepthImage, CV_8U, 255.0f / (ASTRA_MAX_RANGE - ASTRA_MIN_RANGE), 0);

		//m_cvDepthImage.convertTo(m_cvDepthImage, CV_8UC1, 255 / 8000, 0);

		// BLOB DETECTION
		//cv::cvtColor(erosion_dst, erosion_dst, CV_RGB2GRAY);
		//erosion_dst = erosion_dst > 128;
		cv::threshold(erosion_dst, erosion_dst, 128, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
		std::cout << erosion_dst.channels() << std::endl;
		std::vector<cv::KeyPoint> keypoints;
		blobDetector->detect(erosion_dst, keypoints);


		// CONTOURS
		/*
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(erosion_dst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		cv::Mat drawing = cv::Mat::zeros(m_cvDepthImage.size(), CV_8UC3);
		for (int i = 0; i< contours.size(); i++)
		{
			cv::Scalar color = cv::Scalar(255,255,255);
			drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());

		}
		*/

		// HISTOGRAM
		/*
		int histSize = 256;

		/// Set the ranges ( for B,G,R) )
		float range[] = { 0, 256 };
		const float* histRange = { range };
		bool uniform = true; 
		bool accumulate = false;

		cv::Mat hist;
		cv::calcHist(&m_cvDepthImage, 1, 0, erosion_dst, hist, 1, &histSize, &histRange, uniform, accumulate);
		cv::imshow("Hist", hist);
		erosion_dst = m_cvDepthImage - (255-erosion_dst);

		applyColorMap(erosion_dst, erosion_dst, cv::COLORMAP_JET);
		*/

		im_with_keypoints = erosion_dst;

		drawKeypoints(erosion_dst, keypoints, im_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	}
}