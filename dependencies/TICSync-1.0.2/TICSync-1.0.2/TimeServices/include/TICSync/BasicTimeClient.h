/* 
 * copyright {
 * TICSync - A Clock Synchronization Library
 * Copyright 2012 Isis Innovation Limited
 * University of Oxford
 * 
 * Authors: Alastair Harrison
 *          arh@robots.ox.ac.uk
 *
 * See included LICENSE file for license details
 * }
 */
 
#ifndef __BasicTimeClient_h
#define __BasicTimeClient_h

#include "TICSync/TimeServices.h"
#include <zmq.hpp>
#include <string>
#include "Poco/Timestamp.h"
#include "TICSync/TimeServerResponseMsg.h"


namespace TICSync
{


class TimeServices_API BasicTimeClient
/// Connects to a TimeServer and provides methods for obtaining the
/// time at the server
{
public:
  BasicTimeClient(const std::string& zmqServerAddr);
  ~BasicTimeClient();

  Poco::Timestamp requestServerTime();
  Poco::Timestamp requestServerTime(Poco::Timestamp& reqTime, Poco::Timestamp& rcvTime);

private:
  // Copying is not allowed at the moment, as zmq objects can't be copied
  BasicTimeClient(const BasicTimeClient& o);
  BasicTimeClient& operator=(const BasicTimeClient& o);

  void unpackServerResponse(zmq::message_t& msg, TimeServerResponseMsg& r);

private:
  zmq::context_t m_context;
  zmq::socket_t m_socket;

  Poco::Int64 m_nextId;
};

}

#endif // __BasicTimeClient_h
