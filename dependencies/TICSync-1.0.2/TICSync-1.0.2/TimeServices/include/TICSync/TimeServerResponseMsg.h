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

#ifndef TIMESERVERRESPONSEMSG_H_
#define TIMESERVERRESPONSEMSG_H_

#include "TICSync/TimeServices.h"
#include "Poco/Timestamp.h"
#include "Poco/BinaryReader.h"
#include "Poco/BinaryWriter.h"

namespace TICSync
{

class TimeServices_API TimeServerResponseMsg
{
public:
  TimeServerResponseMsg();
  TimeServerResponseMsg(const Poco::Timestamp& ts, const Poco::Int64 id);

public:
  void setTimestamp(const Poco::Timestamp& ts);
  void setId(const Poco::Int64& id);

  const Poco::Timestamp& getTimestamp() const;
  const Poco::Int64& getId() const;

private:
  Poco::Timestamp m_timestamp;
  Poco::Int64 m_id;
};


//
// inlines
//
inline void TimeServerResponseMsg::setTimestamp(const Poco::Timestamp& ts)
{
  m_timestamp = ts;
}

inline void TimeServerResponseMsg::setId(const Poco::Int64& id)
{
  m_id = id;
}

inline const Poco::Timestamp& TimeServerResponseMsg::getTimestamp() const
{
  return m_timestamp;
}

inline const Poco::Int64& TimeServerResponseMsg::getId() const
{
  return m_id;
}

} // namespace TICSync



//
// Serialization
//
Poco::BinaryReader& operator>>(Poco::BinaryReader& s, TICSync::TimeServerResponseMsg& val);
Poco::BinaryWriter& operator<<(Poco::BinaryWriter& s, const TICSync::TimeServerResponseMsg& val);


#endif /* TIMESERVERRESPONSEMSG_H_ */
