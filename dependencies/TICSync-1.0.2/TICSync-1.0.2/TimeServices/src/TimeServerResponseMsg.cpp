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

#include "TICSync/TimeServerResponseMsg.h"
#include "TICSync/TimestampSerialization.h"
#include "Poco/Timestamp.h"

namespace TICSync
{

TimeServerResponseMsg::TimeServerResponseMsg() :
    m_timestamp(0), m_id(0)
{
}

TimeServerResponseMsg::TimeServerResponseMsg(const Poco::Timestamp& ts, Poco::Int64 id=0) :
  m_timestamp(ts), m_id(id)
{
}


} // namespace TICSync


Poco::BinaryReader& operator>>(Poco::BinaryReader& s, TICSync::TimeServerResponseMsg& val)
{
  // TODO error checking - look for flags on input stream, in case we weren't given
  // enough bytes
  Poco::Timestamp ts(0);
  s >> ts;
  val.setTimestamp(ts);

  Poco::Int64 id(0);
  s >> id;
  val.setId(id);

  return s;
}

Poco::BinaryWriter& operator<<(Poco::BinaryWriter& s, const TICSync::TimeServerResponseMsg& val)
{
  return (s << val.getTimestamp() << val.getId());
}
