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
 
#include "TICSync/SynchronizationClient.h"
#include "Poco/Timer.h"
#include "Poco/Timestamp.h"
#include "Poco/DateTimeFormatter.h"
#include "Poco/DateTimeFormat.h"
#include "Poco/RWLock.h"
#include <cmath>
#include <iostream>

namespace TICSync
{

const float BURST_FREQ = 30.0; // Run at this rate for first couple of secs
const float BURST_LEN = 2.0; // Length of initial burst
const float MAX_FREQ = 20.0; // Initial query rate in Hz
const float MIN_FREQ = 5.0; // Steady state query rate in Hz
const int SIGMA = 30; // Variance of frequency gaussian in s

// Cause the filter to switch every 5 minutes.  This allows us to track
// the effects of clock drift
const Poco::Int64 DEFAULT_SWITCHING_PERIOD = 5 * 60 * 1000000;

SynchronizationClient::SynchronizationClient(const std::string& zmqServerAddr) :
    m_clockMapper(DEFAULT_SWITCHING_PERIOD), m_timeclient(zmqServerAddr),
            m_timer(0, 0)
{
}

SynchronizationClient::SynchronizationClient(const std::string& zmqServerAddr, Poco::Timestamp::TimeDiff switchPeriod) :
    m_clockMapper(switchPeriod), m_timeclient(zmqServerAddr),
            m_timer(0, 0)
{
}

SynchronizationClient::~SynchronizationClient()
{
}

void SynchronizationClient::start()
{
    m_timer.stop();
    m_timer.setPeriodicInterval(static_cast< long > (1000.0 / MAX_FREQ));

    {
        Poco::ScopedWriteRWLock lock(m_rwlock);
        m_clockMapper.reset();
    }

    m_stopwatch.restart();

    Poco::TimerCallback< SynchronizationClient > callback(*this,
            &SynchronizationClient::onTimer);
    m_timer.start(callback);
}

void SynchronizationClient::stop()
{
    m_timer.stop();
}

bool SynchronizationClient::isStable()
{
    Poco::ScopedReadRWLock lock(m_rwlock);
    return m_clockMapper.isStable();
}

Poco::Timestamp::TimeDiff SynchronizationClient::span() const
{
    Poco::ScopedReadRWLock lock(m_rwlock);
    return m_clockMapper.span();
}

void SynchronizationClient::test()
{
    m_timer.stop();

    {
        Poco::ScopedWriteRWLock lock(m_rwlock);
        m_clockMapper.reset();
    }

    m_stopwatch.restart();

    for (int i = 0; i < 50000; ++i)
    {
        onTimer(m_timer);

        try
        {
            Poco::Timestamp clientNow;
            Poco::Timestamp serverNow = toServerTime(clientNow);

            std::cout << "clientNow: " << Poco::DateTimeFormatter::format(
                    clientNow, Poco::DateTimeFormat::SORTABLE_FORMAT) << "\n";
            std::cout << "serverNow: " << Poco::DateTimeFormatter::format(
                    serverNow, Poco::DateTimeFormat::SORTABLE_FORMAT) << "\n";
        }
        catch (...)
        {
        }

        std::cout << "\n";
        std::cout.flush();

        Poco::Thread::sleep(100);
    }

    //m_clockMapper.dump();
}

void SynchronizationClient::onTimer(Poco::Timer& timer)
{
    // First calculate the new sampling interval
    // We start fast and slow down over time
    timer.setPeriodicInterval(pollingPeriod(m_stopwatch.elapsed()));

    // Now grab a time stamp from the server
    Poco::Timestamp serverTime(0);
    Poco::Timestamp requestTime(0);
    Poco::Timestamp responseTime(0);
    serverTime = m_timeclient.requestServerTime(requestTime, responseTime);

    // Feed the values into the TICSync estimator
    Poco::ScopedWriteRWLock lock(m_rwlock);
    m_clockMapper.update(requestTime.epochMicroseconds(),
            serverTime.epochMicroseconds(), responseTime.epochMicroseconds());
}

long SynchronizationClient::pollingPeriod(Poco::Timestamp::TimeDiff elapsed)
{
    float secs = static_cast< float > (elapsed) / 1e6;

    float freq = BURST_FREQ;
    if (secs > BURST_LEN)
    {
        freq = exp(-(secs * secs) / (2 * SIGMA * SIGMA))
                * (MAX_FREQ - MIN_FREQ) + MIN_FREQ;
    }

    return static_cast< long > (1000.0 / freq);
}

double SynchronizationClient::skew() const
{
    Poco::ScopedReadRWLock lock(m_rwlock);
    return m_clockMapper.skew();
}

Poco::Timestamp::TimeDiff SynchronizationClient::mostRecentOffset() const
{
    Poco::ScopedReadRWLock lock(m_rwlock);
    return m_clockMapper.mostRecentOffset();
}

Poco::Timestamp SynchronizationClient::toServerTime(const Poco::Timestamp& clientTime) const
{
    Poco::ScopedReadRWLock lock(m_rwlock);
    return Poco::Timestamp(m_clockMapper.clientTimeToServerTime(
            clientTime.epochMicroseconds()));
}

Poco::Timestamp SynchronizationClient::toClientTime(const Poco::Timestamp& serverTime) const
{
    Poco::ScopedReadRWLock lock(m_rwlock);
    return Poco::Timestamp(m_clockMapper.serverTimeToClientTime(
            serverTime.epochMicroseconds()));
}

} // namespace TICSync
