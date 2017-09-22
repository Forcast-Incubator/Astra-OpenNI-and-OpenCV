#pragma once
#include <OpenNI.h>
#include <opencv2/opencv.hpp>

#include "Person.h"
#include "OSCHandler.h"

using namespace cv;
using namespace openni;
using namespace std;

#define MAX_PEOPLE 50

class CamProcessor
{
public:
	CamProcessor(Device& device, VideoStream& depth);
	~CamProcessor();

	Status Initialise(int argc, char **argv);
	Status Start();
	
	void Display();
	void Update();

	VideoFrameRef	m_depthFrame;
	Device&			m_device;
	VideoStream&	m_depthStream;
	VideoStream**	m_streams;

private:
	const DepthPixel* m_depthPixelBuffer;

	Mat m_rawDepthImage;
	Mat m_outputImage;
	Mat m_textOverlay;

	Ptr<BackgroundSubtractorMOG2> MOG2BackgroundSubtractor;

	Mat m_binaryForegroundMask;
	
	Mat m_openedBinaryForegroundMask;
	Mat m_erosionBrush, m_dilationBrush;

	Mat m_contourDrawings;

	double learnRate;
	char keyPress;
	int erosionAmount, dilationAmount;
	bool showGUI;
	float distanceThreshold;

	void CamProcessor::WriteGUIText(const cv::String &text, Point position);
	void CamProcessor::RedrawGUI();

	vector<vector<Point>> m_currentFrameContours;
	vector<Vec4i> m_currentFrameContourHierarchy;

	list<Person> m_lastFramePeople;
	list<Person> m_currentFramePeople;

	bool idList[MAX_PEOPLE];
	int GetUnusedID();

	OSCHandler m_oscHandler;
};

struct PeopleDistance
{
	public:
		float distance;
		Person* person1;
		Person* person2;

	PeopleDistance::PeopleDistance(float _distance, Person* _person1, Person* _person2) : distance(_distance), person1(_person1), person2(_person2){
	}
};

struct is_smaller_functor
{
	bool operator()(const PeopleDistance& x, const PeopleDistance& y) const
	{
		return (x.distance < y.distance);
	}
};