#pragma once

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class Person
{
public:
	Person();
	~Person();

	void ComparePeople(Person &otherPerson);
	void CompareContour(vector<Point2f> otherContour, float maxPointVelocity);

	void CalculateContourVelocities();

private:
	int m_id;

	bool m_alive;
	float m_destructionCountdown;

	Point2f m_centroidPrev;
	Point2f m_centroidNext;
	float m_contourMass;
	vector<Point2f> m_contourPrev;
	vector<Point2f> m_contourNext;
};

