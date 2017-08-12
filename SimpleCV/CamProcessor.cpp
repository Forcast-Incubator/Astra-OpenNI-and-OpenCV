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
		cv::imshow("Depth Estimation", im_with_keypoints);
		
		//cv::imshow("KeyPoints", im_with_keypoints);
		cv::waitKey(1);
	}

	return openni::STATUS_OK;
}


void CamProcessor::display()
{

	/*
	ALGORITHM LAYOUT

	1. Grab the depth frame from the camera
	2. Learn background model and use to segment foreground from background
	3. Erode(5)-Dilate(10)-Erode(5) the foreground mask to remove noise
	4. Draw contours around cleaned up forground mask (binary texture)
	5. Use a low resolution version of the frame to inpaint the missing pixels
	6. Calculate the centers of mass for each contour
	7. Generate histogram for the inside of each contour from the inpainted texture
	8. Sort the histogram and select 1-5 local maxima which satisfy a seperation requirement in the z axis
	9. Generate a texture which contains all the points which are the same colour as the local maxima
	10. Find the center of mass of all pixels in this texture, add to list of keypoints
	11. Create a quarter scale version of this material, erode by 5, and find contours on this picture low resolution picture
	12. If more than one contour is found, calculate center of mass for each contour and add it to list of keypoints
	10. Compare all keypoints with keypoints from previous frame
	 -> If they are within a certain magnitude from another keypoint, treat them as identical, store velocity as magnitude
	 -> Otherwise assign a new id and save to list of keypoints
	11. Send the positions and velocities over OSC!
	*/

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

		// IN PAINTING
		
		cv::Mat depth8Channel;

		m_cvDepthImage.convertTo(depth8Channel, CV_8U, 255.0f / (ASTRA_MAX_RANGE - ASTRA_MIN_RANGE), 0);
		
		cv::Mat small_depthf, _tmp;
		cv::resize(depth8Channel, small_depthf, cv::Size(), 0.2, 0.2);
		
		//inpaint only the "unknown" pixels
		cv::inpaint(small_depthf, (small_depthf == 0), small_depthf, 5.0, cv::INPAINT_TELEA);

		cv::resize(small_depthf, _tmp, depth8Channel.size());
		cv::Mat inpainted = depth8Channel;
		_tmp.copyTo(inpainted, (depth8Channel == 0));  //add the original signal back over the inpaint
		

		// Background subtraction

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

		int kernel_size = 5;

		cv::Mat erosionElement1 = cv::getStructuringElement(cv::MORPH_RECT,
			cv::Size(2 * kernel_size + 1, 2 * kernel_size + 1),
			cv::Point(kernel_size, kernel_size));

		cv::erode(m_foregroundMaskMOG2, erosion_dst, erosionElement1);
		//cv::erode(m_cvDepthImage, m_cvDepthImage, element);

		kernel_size = 10;

		cv::Mat erosionElement2 = cv::getStructuringElement(cv::MORPH_RECT,
			cv::Size(2 * kernel_size + 1, 2 * kernel_size + 1),
			cv::Point(kernel_size, kernel_size));

		cv::dilate(erosion_dst, erosion_dst, erosionElement2);
		cv::erode(erosion_dst, erosion_dst, erosionElement1);

		m_cvDepthImage.convertTo(m_cvDepthImage, CV_8U, 255.0f / (ASTRA_MAX_RANGE - ASTRA_MIN_RANGE), 0);
		

		// OPENING AND CLOSING

		//cv::morphologyEx(m_foregroundMaskMOG2, erosion_dst, cv::MORPH_OPEN, 2, cv::Point(),1,cv::MORPH_RECT);
		//cv::morphologyEx(m_foregroundMaskMOG2, erosion_dst, cv::MORPH_CLOSE, 5);

		//m_cvDepthImage.convertTo(m_cvDepthImage, CV_8U, 255.0f / (ASTRA_MAX_RANGE - ASTRA_MIN_RANGE), 0);

		/*
		// BLOB DETECTION
		//cv::cvtColor(erosion_dst, erosion_dst, CV_RGB2GRAY);
		//erosion_dst = erosion_dst > 128;
		cv::threshold(erosion_dst, erosion_dst, 128, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
		std::cout << erosion_dst.channels() << std::endl;
		std::vector<cv::KeyPoint> keypoints;
		blobDetector->detect(erosion_dst, keypoints);
		*/

		// CONTOURS
		
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(erosion_dst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		
		std::vector<int> contourDepths(contours.size());
		std::vector<int> contourSizes(contours.size());

		cv::Mat depthMinusBackground = inpainted - (255-erosion_dst);

		cv::Mat drawing = cv::Mat::zeros(m_cvDepthImage.size(), CV_8UC3);

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
			cv::calcHist(&depthMinusBackground, 1, 0, thisContour, hist, 1, &histSize, &histRange, uniform, accumulate);

			double min, max;
			int minIdX[2], maxIdX[2];
			cv::minMaxIdx(hist, &min, &max, minIdX, maxIdX);
			contourDepths[i] = maxIdX[0];
			contourSizes[i] = max;
			
			if (contourDepths[i] > 0 && contourSizes[i] > 200) {
				cv::drawContours(drawing, contours, i, cv::Scalar(255-maxIdX[0]*2, maxIdX[0], 255), 1, 8, hierarchy, 0, cv::Point());
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
		*/

		//erosion_dst = m_cvDepthImage - (255-erosion_dst);

		//applyColorMap(erosion_dst, erosion_dst, cv::COLORMAP_JET);
		

		erosion_dst = m_cvDepthImage - (255 - erosion_dst);
		applyColorMap(erosion_dst, erosion_dst, cv::COLORMAP_JET);

		im_with_keypoints = erosion_dst + drawing;

		
		for (int i = 0; i < mc.size(); i++)
		{
			if (contourDepths[i] > 0 && contourSizes[i] > 200) {
				cv::drawMarker(im_with_keypoints, mc[i], cv::Scalar(255, 255, 255), 0, 20, 1, 8);
				cv::putText(im_with_keypoints, std::to_string(contourDepths[i]), mc[i], cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
			}
		}
		

		//drawKeypoints(erosion_dst, keypoints, im_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	}
}