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
 
#ifndef TWOWAYTIMESTAMPS_H_
#define TWOWAYTIMESTAMPS_H_

#include "TICSync/TimeServices.h"

template <class T>
class TwoWayTimestamps
{
public:
  TwoWayTimestamps() : requestTime(0), serverTime(0), receiptTime(0) {}

  T requestTime; // Time of client request (according to client)
  T serverTime;  // Time at server (according to server)
  T receiptTime; // Time response is received (according to client)
};


#endif /* TWOWAYTIMESTAMPS_H_ */
