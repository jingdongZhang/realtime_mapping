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
 
#ifndef TWOWAYTIMESIM_H_
#define TWOWAYTIMESIM_H_

#include "TICSync/TimeServices.h"
#include "TICSync/TwoWayTimestamps.h"

namespace TICSync
{

typedef TwoWayTimestamps<double> TSTriple;

class TimeServices_API TwoWayTimeSim
{
public:
  TwoWayTimeSim();

  void nextMeas(TSTriple& ts);
  void reset();

  double skew();
  double initialOffset();

  double getFreq();
  double getMinDelay();
  double getM1();
  double getM2();
  double getC1();
  double getC2();
  void setFreq(double f);
  void setMinDelay(double v);
  void setM1(double m1);
  void setM2(double m2);
  void setC1(double c1);
  void setC2(double c2);

private:
  double randDelay(int max_ms);
private:
  long   m_count;
  double m_lastClientTime;
  double m_freq;     // Sample frequency (Hz)
  double m_minDelay; // Minimum packet delay
  double m_m1; // Server clock rate
  double m_m2; // Client clock rate
  double m_c1; // Server clock initial offset
  double m_c2; // Client clock initial offset
};


//
// inlines
//

inline double TwoWayTimeSim::getFreq()
{
  return m_freq;
}

inline double TwoWayTimeSim::getMinDelay()
{
  return m_minDelay;
}

inline double TwoWayTimeSim::getM1()
{
  return m_m1;
}

inline double TwoWayTimeSim::getM2()
{
  return m_m2;
}

inline double TwoWayTimeSim::getC1()
{
  return m_c1;
}

inline double TwoWayTimeSim::getC2()
{
  return m_c2;
}

inline void TwoWayTimeSim::setFreq(double f)
{
  m_freq = f;
}

inline void TwoWayTimeSim::setMinDelay(double v)
{
  m_minDelay = v;
}

inline void TwoWayTimeSim::setM1(double m1)
{
  m_m1 = m1;
}

inline void TwoWayTimeSim::setM2(double m2)
{
  m_m2 = m2;
}

inline void TwoWayTimeSim::setC1(double c1)
{
  m_c1 = c1;
}

inline void TwoWayTimeSim::setC2(double c2)
{
  m_c2 = c2;
}

}

#endif /* TWOWAYTIMESIM_H_ */
