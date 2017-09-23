#include "OSCHandler.h"

OSCHandler::OSCHandler()
{
}

OSCHandler::~OSCHandler()
{
}

bool OSCHandler::StartConnection()
{
	return socket.connectTo("localhost", portNumber);
}

bool OSCHandler::ConnectionValid()
{
	return socket.isOk();
}

void OSCHandler::SendPerson(Person& person)
{
	
	if (person.m_id >= 0)
	{
		m_oscMessage.init("/flow/");

		m_oscMessage.pushInt32(person.m_id);
		m_oscMessage.pushBool(person.m_alive);
		m_oscMessage.pushFloat(person.m_destructionCountdown);

		//CENTROID
		m_oscMessage.pushInt32(person.m_centroidNext.x);
		m_oscMessage.pushInt32(person.m_centroidNext.y);
		m_oscMessage.pushInt32(person.m_centroidPrev.x);
		m_oscMessage.pushInt32(person.m_centroidPrev.y);

		// CONTOURS
		int numContours = 0;
		for (int i = 0; i < person.m_contourNext.size(); i++)
		{
			Point position = person.m_contourNext[i];
			Point velocity = person.m_velocityList[i];
			Point difference = position - velocity;
			if (difference.x * difference.x + difference.y * difference.y > 1.0f)
			{
				m_oscMessage.pushInt32(position.x);
				m_oscMessage.pushInt32(position.y);
				m_oscMessage.pushInt32(velocity.x);
				m_oscMessage.pushInt32(velocity.y);
				numContours++;
			}
		}

		m_oscMessage.pushInt32(numContours);

		m_packetWriter.init();
		m_packetWriter.startBundle();
		m_packetWriter.addMessage(m_oscMessage);
		m_packetWriter.endBundle();

		socket.sendPacket(m_packetWriter.packetData(), m_packetWriter.packetSize());
	}
}