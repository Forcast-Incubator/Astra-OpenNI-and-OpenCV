#include "Person.h"

Person::Person()
{

}

Person::~Person()
{

}

void Person::ComparePeople(Person &otherPerson) 
{
}

void Person::CompareContour(vector<Point2f> otherContour, float maxPointVelocity)
{
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
}

void Person::CalculateContourVelocities() 
{

}