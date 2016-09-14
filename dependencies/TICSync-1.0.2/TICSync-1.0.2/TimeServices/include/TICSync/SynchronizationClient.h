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
 
#ifndef SYNCHRONIZATIONCLIENT_H_
#define SYNCHRONIZATIONCLIENT_H_

#include "TICSync/TimeServices.h"
#include <string>
#include "TICSync/BasicTimeClient.h"
#include "TICSync/LinearTwoWayClockSync.h"
#include "TICSync/SwitchingTwoWayClockSync.h"
#include "Poco/Timer.h"
#include "Poco/Stopwatch.h"
#include "Poco/Timestamp.h"
#include "Poco/RWLock.h"



namespace TICSync
{

/**
 * This class performs automatic synchronization
 * to a remote computer that is running a TICSyncTimeServer.
 * It polls the time server at regular intervals and estimates
 * the mapping between the two clocks.
 */
class TimeServices_API SynchronizationClient
{
public:
  SynchronizationClient(const std::string& zmqServerAddr, Poco::Timestamp::TimeDiff switchPeriod);
  SynchronizationClient(const std::string& zmqServerAddr);
  ~SynchronizationClient();

  /**
   * Start the background polling of the Time Server
   */
  void start();

  /**
   * Stop the background polling of the Time Server
   */
  void stop();

  // Conversion functions apply the mapping in opposite directions
  // These assume a linear mapping, so the further a time stamp is from
  // the most recent update time, the worse the estimate will be.

  /**
   * Convert a client time stamp to server time.
   */
  Poco::Timestamp toServerTime(const Poco::Timestamp& clientTime) const;

  /**
   * Convert a server time stamp to client time.
   */
  Poco::Timestamp toClientTime(const Poco::Timestamp& serverTime) const;

  /**
   * Indicates whether the underlying filter has enough measurements to
   * produce a prediction
   */
  bool isStable();

  /**
   * Returns the time span covered by the underlying filter
   */
  Poco::Timestamp::TimeDiff span() const;

  /// Get the time at the server
  Poco::Timestamp serverTimeNow() const;

  /// A version of serverTimeNow for the convenience of Dr. Gabe Sibley
  Poco::Timestamp whatsTheFrickinTime() const;

  /// Relative skew between clocks, defined as
  /// d(clientTime)/dt - d(serverTime)/dt
  double skew() const;

  /**
   * Most recent offset between the clocks, defined as
   * clientTime - serverTime
   */
  Poco::Timestamp::TimeDiff mostRecentOffset() const;

  /// Run in testing mode in foreground thread
  void test();

private:
  // Copying is currently prohibited
  SynchronizationClient(const SynchronizationClient& o);
  SynchronizationClient& operator=(const SynchronizationClient& o);

private:
  /// Called periodically by the Timer, to contact the time server.
  void onTimer(Poco::Timer& timer);

  long pollingPeriod(Poco::Timestamp::TimeDiff elapsed);

private:
  TICSync::SwitchingTwoWayClockSync<Poco::Timestamp::TimeVal> m_clockMapper;
  BasicTimeClient           m_timeclient;
  Poco::Timer               m_timer;
  Poco::Stopwatch           m_stopwatch;
  mutable Poco::RWLock      m_rwlock;

};


//
// inlines
//

inline Poco::Timestamp SynchronizationClient::serverTimeNow() const
{
  return toServerTime(Poco::Timestamp());
}

inline Poco::Timestamp SynchronizationClient::whatsTheFrickinTime() const
{
  return toServerTime(Poco::Timestamp());
}


} // namespace TICSync

#endif /* SYNCHRONIZATIONCLIENT_H_ */
