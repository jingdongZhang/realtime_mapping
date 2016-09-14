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

#include "TICSync/BasicTimeClient.h"
#include "TICSync/TimeServerResponseMsg.h"
#include "TICSync/timeserviceExceptions.h"
#include "TICSync/TimestampSerialization.h"
#include "Poco/BinaryReader.h"
#include "Poco/BinaryWriter.h"
#include "Poco/Thread.h"
#include "TICSync/bufferstream.h"
#include <string>
#include <iostream>
#include <zmq.hpp>
#include <cstdlib>

namespace TICSync
{


BasicTimeClient::BasicTimeClient(const std::string& zmqServerAddr) :
      m_context(1), m_socket(m_context, ZMQ_REQ), m_nextId(0)
{
  m_socket.connect(zmqServerAddr.c_str());
}

BasicTimeClient::~BasicTimeClient()
{
}

void BasicTimeClient::unpackServerResponse(zmq::message_t& msg, TimeServerResponseMsg& r)
{
  // Unpack the server response to get its message
  TICSync::ibufferstream stream(static_cast<const char*> (msg.data()), msg.size());
  Poco::BinaryReader reader(stream, Poco::BinaryReader::NETWORK_BYTE_ORDER);
  reader >> r;
}


Poco::Timestamp BasicTimeClient::requestServerTime()
{
  // Instantiate everything first, to save time later
  zmq::message_t query(0);
  zmq::message_t result;

  m_socket.send(query);
  m_socket.recv(&result);

  TimeServerResponseMsg serverResponse;
  unpackServerResponse(result, serverResponse);

  return serverResponse.getTimestamp();
}


Poco::Timestamp BasicTimeClient::requestServerTime(Poco::Timestamp& reqTime, Poco::Timestamp& rcvTime)
{
  // Instantiate everything first, to save time later
  zmq::message_t query(sizeof(Poco::Int64));
  TICSync::obufferstream stream(static_cast<char*> (query.data()), query.size());
  Poco::BinaryWriter writer(stream, Poco::BinaryWriter::NETWORK_BYTE_ORDER);
  writer << m_nextId;

  zmq::message_t result;

  reqTime.update();

  if (!m_socket.send(query))
    throw IOException();

  if (!m_socket.recv(&result))
    throw IOException();

  rcvTime.update();

  TimeServerResponseMsg serverResponse;
  unpackServerResponse(result, serverResponse);

  if (m_nextId != serverResponse.getId())
  {
    // The Ids don't match up!  ZMQ ought to stop this from happening.
    throw IOException();
  }

  ++m_nextId;

  return serverResponse.getTimestamp();
}


}
