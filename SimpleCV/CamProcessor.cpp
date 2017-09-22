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

Status CamProcessor::Initialise(int argc, char **argv)
{

	/* OPENNI SET UP */
	VideoMode depthVideoMode;
	if (m_depthStream.isValid())
	{
		// store video mode
		depthVideoMode = m_depthStream.getVideoMode();
	}
	else {
		printf("Depth stream not valid\n");
		return STATUS_ERROR;
	}

	// set up OpenNI streams
	m_streams = new VideoStream*[1];
	m_streams[0] = &m_depthStream;

	// initialise openNI version of depth buffer
	m_depthPixelBuffer = new DepthPixel[ASTRA_FEED_X * ASTRA_FEED_Y];

	namedWindow("Depth Image GUI", WINDOW_NORMAL | WINDOW_KEEPRATIO);
	resizeWindow("Depth Image GUI", WIN_SIZE_X, WIN_SIZE_Y);

	/* OPEN CV SET UP */
	MOG2BackgroundSubtractor = createBackgroundSubtractorMOG2();

	MOG2BackgroundSubtractor->setNMixtures(5);			//number of gaussian components in the background model
	MOG2BackgroundSubtractor->setHistory(200);			//number of last frames that affect the background model
	MOG2BackgroundSubtractor->setVarThreshold(1000);	//variance threshold for the pixel-model match
	MOG2BackgroundSubtractor->setDetectShadows(false);	//don't mark shadows since the feature is only useful for color data
	
	learnRate = 0.99;
	erosionAmount = 5;
	dilationAmount = 5;
	showGUI = true;

	distanceThreshold = 30.0f * 30.0f;
	
	RedrawGUI();
	
	for (int i = 0; i < MAX_PEOPLE; i++)
	{
		idList[i] = false;
	}

	if (m_oscHandler.StartConnection())
		cout << "Connection established" << endl;
	else
		cout << "Connection failed" << endl;

	return STATUS_OK;
}

Status CamProcessor::Start()
{
	while (true)
	{
		Update();
		Display();
	}

	return STATUS_OK;
}

void CamProcessor::Display()
{	
	imshow("Depth Image GUI", m_outputImage);
	keyPress = waitKey(1);
}

void CamProcessor::Update()
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

		m_rawDepthImage.create(m_depthFrame.getHeight(), m_depthFrame.getWidth(), CV_16UC1);
		memcpy(m_rawDepthImage.data, m_depthPixelBuffer, m_depthFrame.getHeight()*m_depthFrame.getWidth() * sizeof(uint16_t));

		///// KEYBOARD HANDLING /////
		learnRate = 0.0;

		switch (keyPress)
		{
		case 'h':
			showGUI = !showGUI;
			RedrawGUI();
			break;
		case 'r':
			learnRate = 0.99;
			break;
		case '+':
		case '=':
			erosionAmount++;
			RedrawGUI();
			break;
		case '_':
		case '-':
			if (erosionAmount <= 0)
				erosionAmount = 0;
			else
				erosionAmount--;
			RedrawGUI();
			break;
		case 'p':
			dilationAmount++;
			RedrawGUI();
			break;
		case 'o':
			dilationAmount -= (dilationAmount <= 0) ? 0 : 1;
			RedrawGUI();
			break;
		default:
			break;
		}
		
		/* OPEN CV OPERATIONS */

		///// BACKGROUND SUBTRACTION /////
		MOG2BackgroundSubtractor->apply(m_rawDepthImage, m_binaryForegroundMask, learnRate);

		///// OPEN OPERATION /////
		erode(m_binaryForegroundMask, m_openedBinaryForegroundMask, m_erosionBrush);
		dilate(m_openedBinaryForegroundMask, m_openedBinaryForegroundMask, m_dilationBrush);

		///// CONTOUR DETECTION /////
		findContours(m_openedBinaryForegroundMask, m_currentFrameContours, m_currentFrameContourHierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		m_contourDrawings = Mat::zeros(m_rawDepthImage.size(), CV_8UC1);

		m_currentFramePeople.clear();
		
		list<PeopleDistance> distances;
		
		// Iterate through all contours found in the current frame
		for (int contour = 0; contour<m_currentFrameContours.size(); contour++)
		{
			Scalar color = cv::Scalar(255, 255, 255);

			// Simplify the contour into a polygon with less vertices
			approxPolyDP(Mat(m_currentFrameContours[contour]), m_currentFrameContours[contour], 3, true);

			// Find the details of the contour data
			Moments contourMoments = moments(m_currentFrameContours[contour], false);
			Point2d massCenter = Point2d(contourMoments.m10 / contourMoments.m00, contourMoments.m01 / contourMoments.m00);

			// Push this contour into a storage container for some reason
			m_currentFramePeople.push_back(Person(0, true, massCenter, m_currentFrameContours[contour]));

			// FIND DISTANCE BETWEEN THIS CONTOUR AND ALL CONTOURS FROM THE PREVIOUS FRAME
			float minDist = numeric_limits<float>::max();
			
			Person* thisPerson = &m_currentFramePeople.back();

			// Iterate through all people from previous frame
			for (list<Person>::iterator iterator = m_lastFramePeople.begin(), end = m_lastFramePeople.end(); iterator != end; iterator++)
			{
				// Figure out square distance between this contour and all others
				float dist = iterator->GetSquareDistance(*thisPerson);
				if (dist < minDist && dist < distanceThreshold)
				{
					minDist = dist;
					// if it's within the threshold, store in an array of people
					distances.push_back(PeopleDistance(dist, thisPerson, &*iterator));
				}
			}
		}

		// sort all the distances that were within the threshold to find the smallest distances
		distances.sort(is_smaller_functor());

		// Copy the data from each person over, in the order of how close they were between frames
		for (list<PeopleDistance>::iterator i = distances.begin(); i != distances.end(); ++i)
		{
			if (i->person1->m_copied != true && i->person2->m_copied != true) {
				i->person2->CopyData(*i->person1);
				i->person1->m_copied = true;
				i->person1->m_alive = true;
				i->person2->m_copied = true;
				i->person2->m_alive = true;
			} else {
				//i->person1.m_copied = false;
				//i->person1.m_alive = false;
				//i->person2.m_copied = false;
				//i->person2.m_alive = false;
			}
		}

		distances.clear();

		// Add any new people to the people list, give them a unique ID
		for (list<Person>::iterator iterator = m_currentFramePeople.begin(), end = m_currentFramePeople.end(); iterator != end; iterator++)
		{
			if (iterator->m_copied != true) {
				iterator->m_id = GetUnusedID();
				m_lastFramePeople.push_back(*iterator);
			}

			//drawMarker(m_contourDrawings, iterator->m_centroidNext, Scalar(150, 150, 150), 0, 20, 2, 8);
			//putText(m_contourDrawings, to_string(iterator->m_id), iterator->m_centroidNext - Point2f(0, 30), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(160, 160, 160), 0, LINE_8);
		}

		// Update the whole list of people
		list<Person>::iterator person = m_lastFramePeople.begin();
		while (person != m_lastFramePeople.end())
		{
			if (person->m_copied != true)
			{
				// NO MATCH WAS FOUND WITH THE PREVIOUS FRAME!
				person->m_alive = false;
			}
			else {
				//person->m_alive = false;
			}
			std::vector<std::vector<cv::Point> > contourVec;
			contourVec.push_back(person->m_contourPrev);
			contourVec.push_back(person->m_contourNext);

			cv::drawContours(m_contourDrawings, contourVec, 0, Scalar(10 * person->m_destructionCountdown), 2, LINE_AA);
			cv::drawContours(m_contourDrawings, contourVec, 1, Scalar(25 * person->m_destructionCountdown), 1, LINE_AA);
			
			drawMarker(m_contourDrawings, person->m_centroidNext, Scalar(255), MARKER_DIAMOND, 15, 1, LINE_AA);
			line(m_contourDrawings, person->m_centroidPrev, person->m_centroidNext, Scalar(255), 1, LINE_AA);
			putText(m_contourDrawings, to_string(person->m_id), person->m_centroidNext - Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255), 0, LINE_8);

			person->CalculateContourVelocities();

			for (int i = 0; i < person->m_velocityList.size(); i++)
			{
				if (person->m_velocityList[i] != person->m_contourNext[i])
				{
					line(m_contourDrawings, person->m_velocityList[i], person->m_contourNext[i], Scalar(255), 1, LINE_AA);
					//drawMarker(m_contourDrawings, person->m_contourNext[i], Scalar(200), MARKER_TRIANGLE_UP, 5, 1, LINE_AA);
				}
			}

			person->Update();

			if (person->m_deleted != false)
			{
				drawMarker(m_contourDrawings, person->m_centroidNext, Scalar(200), MARKER_TILTED_CROSS, 15, 1, LINE_AA);

				idList[person->m_id] = false;
				m_lastFramePeople.erase(person++);
			}
			else {
				if (m_oscHandler.ConnectionValid()) {
					m_oscHandler.SendPerson(*person);
				} else {
					cout << "Person not sent: Connection invalid" << endl;
					m_oscHandler.StartConnection();
				}
				person++;
			}
		}

		RedrawGUI();

		WriteGUIText("Number of Contours Previous: " + to_string(m_lastFramePeople.size()), Point(5, 450));
		WriteGUIText("Number of Contours Current: " + to_string(m_currentFramePeople.size()), Point(5, 465));

		///// CREATE FINAL IMAGE /////
		m_outputImage = m_textOverlay + m_contourDrawings;
	}
}

void CamProcessor::WriteGUIText(const cv::String &text, Point position)
{
	putText(m_textOverlay, text, position, FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 0, LINE_8);
}


void CamProcessor::RedrawGUI()
{
	m_textOverlay = Mat::zeros(Size(WIN_SIZE_X, WIN_SIZE_Y), CV_8UC1);

	if (showGUI) {
		WriteGUIText("h: hide text controls (h again to show)", Point(5, 15));
		WriteGUIText("r: recalculate background subtraction model", Point(5, 30));
		WriteGUIText("Erosion brush size: " + to_string(erosionAmount), Point(5, 45));
		WriteGUIText("+: increase erosion brush size", Point(5, 60));
		WriteGUIText("-: decrease erosion brush size", Point(5, 75));
		WriteGUIText("Dilation brush size: " + to_string(dilationAmount), Point(5, 90));
		WriteGUIText("p: increase dilation brush size", Point(5, 105));
		WriteGUIText("o: decrease dilation brush size", Point(5, 120));
	}

	m_erosionBrush = getStructuringElement(MORPH_RECT, Size(2 * erosionAmount + 1, 2 * erosionAmount + 1), Point(erosionAmount, erosionAmount));
	m_dilationBrush = getStructuringElement(MORPH_RECT, Size(2 * dilationAmount + 1, 2 * dilationAmount + 1), Point(dilationAmount, dilationAmount));
}


int CamProcessor::GetUnusedID()
{
	int i = 0;

	while (idList[i] != false && i<MAX_PEOPLE)
		i++;

	if (i < MAX_PEOPLE) {
		idList[i] = true;
		return i;
	}
	else {
		return -1;
	}
}