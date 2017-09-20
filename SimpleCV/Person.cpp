#include "Person.h"

Person::Person(int id, bool alive, Point2f centroid, vector<Point> &contour)
{
	m_id = id;
	m_alive = alive;
	m_centroidPrev = centroid;
	m_centroidNext = centroid;

	m_contourNext = contour;
	m_contourPrev = contour;
	
	m_copied = false;
	m_deleted = false;
	m_destructionCountdown = 10;
}

Person::~Person()
{
	m_contourNext.clear();
	m_contourPrev.clear();
}

float Person::GetSquareDistance(Person &otherPerson) 
{
	Point2f difference = otherPerson.m_centroidPrev - m_centroidNext;
	return difference.x * difference.x + difference.y * difference.y;
	
}

void Person::CompareContour(vector<Point> &otherContour, float maxPointVelocity)
{
	/*
	for (int i = 0; i < otherContour.size(); i++)
	{
		for (int j = 0; j < m_contourPrev.size(); j++)
		{
			Point2f point0 = otherContour[i];
			Point2f point1 = m_contourPrev[j];
			float sqrDist = (point0.x * point1.x) + (point0.y * point1.y);
			if (sqrDist <= maxPointVelocity)
			{
				// points are close enough to be considered the same
				m_contourNext.push_back(point0);
			}
		}
	}
	*/
}

void Person::CalculateContourVelocities() 
{

}

void Person::CopyData(Person &otherPerson)
{
	m_centroidNext = otherPerson.m_centroidNext;
	m_contourNext = otherPerson.m_contourNext;
}

void Person::Update()
{
	m_contourPrev = m_contourNext;
	m_centroidPrev = m_centroidNext;

	if (m_alive)
	{
		m_destructionCountdown = 10;
	}
	else {
		m_destructionCountdown--;
		if (m_destructionCountdown <= 0)
		{
			m_deleted = true;
		}
	}
}