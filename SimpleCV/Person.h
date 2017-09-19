#pragma once

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class Person
{
public:
	Person(int id, bool alive, Point2f centroid, vector<Point> &contour);
	~Person();

	float ComparePeople(Person &otherPerson);
	void CompareContour(vector<Point> &otherContour, float maxPointVelocity);

	void CopyData(Person &otherPerson);

	void CalculateContourVelocities();
	int m_id;
	bool m_alive;
	bool m_copied;

	Point2f m_centroidPrev;
	Point2f m_centroidNext;

	vector<Point> m_contourPrev;
	vector<Point> m_contourNext;
	

	void Update();

private:
	float m_destructionCountdown;

	
};

