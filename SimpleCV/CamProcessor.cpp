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
		
		// Change thresholds
		blobDetectorParams.minThreshold = 10;
		blobDetectorParams.maxThreshold = 200;

		// Filter by Area.
		blobDetectorParams.filterByArea = true;
		blobDetectorParams.minArea = 1500;

		// Filter by Circularity
		blobDetectorParams.filterByCircularity = true;
		blobDetectorParams.minCircularity = 0.1;

		// Filter by Convexity
		blobDetectorParams.filterByConvexity = true;
		blobDetectorParams.minConvexity = 0.87;

		// Filter by Inertia
		blobDetectorParams.filterByInertia = true;
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
		cv::imshow("Depth", m_cvDepthImage);
		cv::imshow("Background Subtraction", m_foregroundMaskMOG2);
		cv::imshow("Erosion", erosion_dst);
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
		
		cv::Mat image8uc1;
		m_foregroundMaskMOG2.convertTo(image8uc1, CV_8U, 1); // CV_8U should work as well
		
		//cv::cvtColor(m_cvDepthImage, m_cvDepthImage, CV_BGR2GRAY);

		int erosion_size = 5;

		cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
			cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
			cv::Point(erosion_size, erosion_size));

		cv::erode(image8uc1, erosion_dst, element);
		cv::dilate(erosion_dst, erosion_dst, element);

	}
}