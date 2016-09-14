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

#include <GetPot>
#include <iostream>
#include <iomanip>
#include <exception>
#include <string>
#include <sstream>
#include "Poco/Thread.h"
#include "Poco/Format.h"
#include "Poco/DateTimeFormatter.h"
#include "Poco/DateTimeFormat.h"

#include "TICSync/SynchronizationClient.h"

void doWork(std::string serverAddress, double printFreq)
{
    int printPeriod_ms = (1.0e3 / printFreq);

    TICSync::SynchronizationClient timeClient(serverAddress);
    timeClient.start();

    // Wait until some measurements have been gathered.
    while (!timeClient.isStable())
        Poco::Thread::yield();

    while (1)
    {
        try
        {
            // Find out the time at the server
            Poco::Timestamp serverTime(timeClient.serverTimeNow());

            // Print server time in human readable format
            std::cout << Poco::DateTimeFormatter::format(serverTime,
                    Poco::DateTimeFormat::SORTABLE_FORMAT);
            std::cout << " - ";

            // Show the time span currently covered by the filter
            std::cout.setf(std::ios::fixed);
            std::cout << "span: " << std::setprecision(2) << std::setw(8);
            std::cout << static_cast<double> (timeClient.span()) * 1e-6;
            std::cout << " sec, ";

            // Print the offset and skew estimated by the TICSync filter
            // in the Synchronization client.
            // Skew is measured in Parts Per Million
            std::cout << "Est skew: " << std::setprecision(3) << std::setw(10);
            std::cout << timeClient.skew() * 1e6 << " ppm, ";

            std::cout << "Est offset: " << std::setw(7);
            std::cout << timeClient.mostRecentOffset() << " us";
            std::cout << std::endl;

            // Wait for a bit before printing the next output
            Poco::Thread::sleep(printPeriod_ms);
        }
        catch (std::exception& e)
        {
            std::cerr << "Exception: " << e.what() << std::endl;
        }
    }
}

int main(int argc, char** argv)
{

    GetPot cl(argc, argv);

    if (cl.search(2, "-h", "--help"))
    {
        std::cerr << cl[0]
                << " -a <host address> -p <port> -f <print_frequency>\n";
        return 0;
    }

    int port(5555);
    if (cl.search(2, "-p", "--port"))
        port = cl.next(5555);

    double printFreq(1.0);
    if (cl.search(1, "-f"))
        printFreq = cl.next(1.0);

    std::string sHost;
    if (cl.search(1, "-a"))
        sHost = cl.next("localhost");

    std::stringstream s;
    s << "tcp://" << sHost << ":" << port;
    std::string serverAddress(s.str());

    // Start the thing running
    doWork(serverAddress, printFreq);

    return 0;
}
