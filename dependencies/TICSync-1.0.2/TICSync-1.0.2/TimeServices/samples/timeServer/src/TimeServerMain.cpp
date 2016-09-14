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

#include <string>
#include <sstream>
#include <vector>
#include <iostream>
#include <GetPot>
#include <zmq.hpp>
#include "Poco/ThreadPool.h"
#include "Poco/Runnable.h"
#include "Poco/Timestamp.h"
#include "Poco/BinaryWriter.h"
#include "TICSync/vectorstream.h"
#include "TICSync/bufferstream.h"
#include "TICSync/TimeServerResponseMsg.h"


zmq::message_t& prepareTimestampMessage(zmq::message_t& msg, const Poco::Timestamp& timestamp, Poco::Int64 nId=0)
{
  TICSync::TimeServerResponseMsg response(timestamp, nId);

  TICSync::vectorstream stream;
  Poco::BinaryWriter writer(stream, Poco::BinaryWriter::NETWORK_BYTE_ORDER);
  writer << response;

  msg.rebuild(stream.vector().size());
  std::copy(stream.vector().begin(), stream.vector().end(), static_cast<char*> (msg.data()));

  return msg;
}


class WorkerRunnable: public Poco::Runnable
{
public:
  WorkerRunnable(zmq::context_t& context) :
    m_context(context)
  {
  }

  virtual void run()
  {
    zmq::socket_t s(m_context, ZMQ_REP);

    s.connect("inproc://workers");

    while (1)
    {
      zmq::message_t request;
      s.recv(&request, 0);

      // Read request message - it might contain an ID
      Poco::Int64 nId = 0;
      if (request.size() == sizeof(Poco::Int64))
      {
        TICSync::ibufferstream stream(static_cast<char*> (request.data()), request.size());
        Poco::BinaryReader reader(stream, Poco::BinaryReader::NETWORK_BYTE_ORDER);
        reader >> nId;
      }

      zmq::message_t reply;
      s.send(prepareTimestampMessage(reply, Poco::Timestamp(), nId), 0);
    }
  }

private:
  zmq::context_t& m_context;
};




int main(int argc, char** argv)
{
  GetPot cl(argc, argv);

  if( cl.search("--help") )
  {
    std::cerr << cl[0] << " -p <port> -t <num_threads>\n";
    return 0;
  }

  int port(5555);
  if ( cl.search(2, "-p", "--port") )
    port = cl.next(5555);

  int numThreads = 4;
  if ( cl.search(2, "-t", "--num-threads") )
    numThreads = cl.next(4);

  std::stringstream s;
  s << "tcp://*:" << port;
  std::string zmq_address(s.str());


  //  Prepare our context and sockets
  zmq::context_t context(1);

  zmq::socket_t clients(context, ZMQ_XREP);
  clients.bind(zmq_address.c_str());

  zmq::socket_t workers(context, ZMQ_XREQ);
  workers.bind("inproc://workers");

  //  Launch pool of worker threads
  WorkerRunnable runnable(context);
  for (int i = 0; i < numThreads; ++i)
  {
    Poco::ThreadPool::defaultPool().start(runnable);
  }

  //  Connect work threads to client threads via a queue
  zmq_device(ZMQ_QUEUE, clients, workers);
  return 0;
}


