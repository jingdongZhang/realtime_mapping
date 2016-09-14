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
 
#include <iostream>
#include <string>
#include "Poco/Thread.h"
#include "Poco/Format.h"
#include "Poco/Timestamp.h"
#include "Poco/DateTimeFormatter.h"
#include "Poco/DateTimeFormat.h"
#include "TICSync/BasicTimeClient.h"

int main(int argc, char** argv)
{

  TICSync::BasicTimeClient timeClient("tcp://localhost:5555");

  unsigned int n = 0;
  while (1)
  {
    Poco::Timestamp serverTime(timeClient.requestServerTime());

    std::cout << "Received: ";
    std::cout << Poco::DateTimeFormatter::format(serverTime, Poco::DateTimeFormat::SORTABLE_FORMAT);
    std::cout << "\n";

    Poco::Thread::sleep(100);
    ++n;
  }

  return 0;
}
