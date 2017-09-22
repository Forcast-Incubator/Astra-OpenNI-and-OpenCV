#pragma once

#include "oscpkt/oscpkt.hh"
#include "oscpkt/udp.hh"

#include "Person.h"

using namespace oscpkt;

class OSCHandler
{
public:
	OSCHandler();
	~OSCHandler();

	bool StartConnection();
	bool ConnectionValid();
	void SendPerson(Person &person);

private:
	PacketWriter m_packetWriter;
	Message m_oscMessage;
	UdpSocket socket;

	const int portNumber = 9109;
};

