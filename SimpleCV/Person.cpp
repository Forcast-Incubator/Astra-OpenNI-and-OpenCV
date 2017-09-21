#include "Person.h"

Person::Person(int id, bool alive, Point centroid, vector<Point> &contour)
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
	Point difference = otherPerson.m_centroidPrev - m_centroidNext;
	return difference.x * difference.x + difference.y * difference.y;
}

void Person::CalculateContourVelocities() 
{
	float maxVelocityDistance = 10.0f * 10.0f;
	list<PointPair> pointPairs;
	for (int i = 0; i < m_contourNext.size(); i++)
	{
		for (int j = 0; j < m_contourPrev.size(); j++)
		{
			Point difference = m_contourNext[i] - m_contourPrev[j];
			float squareDistance = difference.x * difference.x + difference.y * difference.y;
			if (squareDistance < maxVelocityDistance)
			{
				pointPairs.push_back(PointPair(squareDistance, i, j));
			}
		}
	}

	pointPairs.sort(smaller_of_pair_functor());
	
	m_velocityList.clear();
	for (int i = 0; i < m_contourNext.size(); i++)
	{
		m_velocityList.push_back(m_contourNext[i]);
	}

	for (list<PointPair>::iterator i = pointPairs.begin(); i != pointPairs.end(); ++i)
	{
		if (m_velocityList[i->nextPointIndex] == m_contourNext[i->nextPointIndex])
		{
			m_velocityList[i->nextPointIndex] = m_contourPrev[i->prevPointIndex];
		}
	}
	
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

	m_copied = false;

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