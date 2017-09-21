#pragma once

#include <opencv2/opencv.hpp>
#include <math.h> 

using namespace cv;
using namespace std;

class Person
{
public:
	Person(int id, bool alive, Point centroid, vector<Point> &contour);
	~Person();

	float GetSquareDistance(Person &otherPerson);

	void CopyData(Person &otherPerson);

	void CalculateContourVelocities();
	int m_id;
	bool m_alive;
	bool m_copied;
	bool m_deleted;

	Point m_centroidPrev;
	Point m_centroidNext;

	vector<Point> m_contourPrev;
	vector<Point> m_contourNext;
	float m_destructionCountdown;

	void Update();

	vector<Point> m_velocityList;
};

struct PointPair
{
public:
	float squareDistance;
	int nextPointIndex;
	int prevPointIndex;
	PointPair(float _squareDistance, int _nextPointIndex, int _prevPointIndex)
		: squareDistance(_squareDistance), nextPointIndex(_nextPointIndex), prevPointIndex(_prevPointIndex) {
	}
};

struct smaller_of_pair_functor
{
	bool operator()(const PointPair& x, const PointPair& y) const
	{
		return (x.squareDistance < y.squareDistance);
	}
};