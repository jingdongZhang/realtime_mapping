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

#include "TICSync/TwoWayTimeSim.h"
#include <iostream>
#include <cstdlib>
#include <ctime>

namespace TICSync
{

TwoWayTimeSim::TwoWayTimeSim() :
    m_count(0), m_lastClientTime(0.0), m_freq(1.0),
    m_minDelay(0.0), m_m1(1.0), m_m2(1.0), m_c1(0.0), m_c2(0.0)
{
  /* initialize random seed: */
  srand ( static_cast<unsigned int> (time(NULL)) );
}


void TwoWayTimeSim::reset()
{
  m_count = 0;
  m_lastClientTime = 0.0;
  m_freq = 1.0;
  m_minDelay = 0.0;
  m_m1 = 1.0;
  m_m2 = 1.0;
  m_c1 = 0.0;
  m_c2 = 0.0;
}


void TwoWayTimeSim::nextMeas(TSTriple& ts)
{
  double t1=0;
  double t2=0;
  double t3=0;

  // Earliest start of transaction
    double t = static_cast<double> (m_count) / m_freq;

    // Can't have time going backwards!
    if (t < m_lastClientTime) t = m_lastClientTime;

    // Generate set of three timestamps
    t1 =  t + randDelay(10);
    t2 = t1 + randDelay(40) + m_minDelay;
    t3 = t2 + randDelay(40) + m_minDelay;

    // Push through clock models
    ts.requestTime = m_m2*t1 + m_c2; // Client requests
    ts.serverTime  = m_m1*t2 + m_c1; // Server responds
    ts.receiptTime = m_m2*t3 + m_c2; // Client receives

    ++m_count;
  }

  double TwoWayTimeSim::skew()
  {
    return (m_m2 - m_m1) / m_m1;
  }

  double TwoWayTimeSim::initialOffset()
  {
    return m_c2 - (m_m2 * m_c1) / m_m1;
  }


  double TwoWayTimeSim::randDelay(int max_ms)
  {
    int delay_ms = rand() % (max_ms * 1000);
    return delay_ms / 1e6;
  }


}
