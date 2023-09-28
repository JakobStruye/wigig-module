/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
//
// Copyright (c) 2006 Georgia Tech Research Corporation
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation;
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// Author: George F. Riley<riley@ece.gatech.edu>
//

// ns3 - On/Off Data Source Application class
// George F. Riley, Georgia Tech, Spring 2007
// Adapted from ApplicationOnOff in GTNetS.

#include "ns3/log.h"
#include "ns3/address.h"
#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/packet-socket-address.h"
#include "ns3/node.h"
#include "ns3/nstime.h"
#include "ns3/data-rate.h"
#include "ns3/random-variable-stream.h"
#include "ns3/socket.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/trace-source-accessor.h"
#include "burst-application.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/seq-ts-header.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("BurstApplication");

NS_OBJECT_ENSURE_REGISTERED (BurstApplication);

TypeId
BurstApplication::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::BurstApplication")
    .SetParent<Application> ()
    .SetGroupName("Applications")
    .AddConstructor<BurstApplication> ()
    .AddAttribute ("DataRate", "The data rate in on state.",
                   DataRateValue (DataRate ("500kb/s")),
                   MakeDataRateAccessor (&BurstApplication::m_cbrRate),
                   MakeDataRateChecker ())
    .AddAttribute ("PacketSize", "The size of packets sent in on state",
                   UintegerValue (512),
                   MakeUintegerAccessor (&BurstApplication::m_pktSize),
                   MakeUintegerChecker<uint32_t> (1))
    .AddAttribute ("Remote", "The address of the destination",
                   AddressValue (),
                   MakeAddressAccessor (&BurstApplication::m_peer),
                   MakeAddressChecker ())
    .AddAttribute ("Local",
                   "The Address on which to bind the socket. If not set, it is generated automatically.",
                   AddressValue (),
                   MakeAddressAccessor (&BurstApplication::m_local),
                   MakeAddressChecker ())
    .AddAttribute ("BurstsPerSecond", "A RandomVariableStream used to pick the duration of the 'On' state.",
                   StringValue ("ns3::ConstantRandomVariable[Constant=60]"),
                   MakePointerAccessor (&BurstApplication::m_burstsPerSecond),
                   MakePointerChecker <RandomVariableStream>())
    .AddAttribute ("EnableTimestamp", "Whether we add timestamp tag to each outgoing packet or not.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&BurstApplication::m_enableTimestamp),
                   MakeBooleanChecker ())
    .AddAttribute ("MaxBytes", 
                   "The total number of bytes to send. Once these bytes are sent, "
                   "no packet is sent again, even in on state. The value zero means "
                   "that there is no limit.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&BurstApplication::m_maxBytes),
                   MakeUintegerChecker<uint64_t> ())
    .AddAttribute ("MaxPackets",
                   "The total number of packets to send. Once these packets are sent, "
                   "no packet is sent again, even in on state. The value zero means "
                   "that there is no limit.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&BurstApplication::m_maxPackets),
                   MakeUintegerChecker<uint64_t> ())
    .AddAttribute ("Protocol", "The type of protocol to use. This should be "
                   "a subclass of ns3::SocketFactory",
                   TypeIdValue (UdpSocketFactory::GetTypeId ()),
                   MakeTypeIdAccessor (&BurstApplication::m_tid),
                   // This should check for SocketFactory as a parent
                   MakeTypeIdChecker ())
    .AddAttribute ("EnableE2EStats",
                   "Enable E2E statistics (sequences, timestamps)",
                   BooleanValue (false),
                   MakeBooleanAccessor (&BurstApplication::m_enableE2EStats),
                   MakeBooleanChecker ())
    .AddTraceSource ("TagCreated", "A new tag is created and is sent",
                    MakeTraceSourceAccessor (&BurstApplication::m_tagCreated),
                    "ns3::BurstAppliation::TagTracedCallback")
    .AddTraceSource ("Tx", "A new packet is created and is sent",
                     MakeTraceSourceAccessor (&BurstApplication::m_txTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("TxWithAddresses", "A new packet is created and is sent",
                     MakeTraceSourceAccessor (&BurstApplication::m_txTraceWithAddresses),
                     "ns3::Packet::TwoAddressTracedCallback")
    .AddTraceSource ("TxE2EStat", "Statistic sent with the packet",
                     MakeTraceSourceAccessor (&BurstApplication::m_txTraceWithStats),
                     "ns3::PacketSink::E2EStatCallback")
  ;
  return tid;
}


BurstApplication::BurstApplication ()
  : m_socket (0),
    m_connected (false),
    m_residualBits (0),
    m_lastStartTime (Seconds (0)),
    m_totBytes (0),
    m_txPackets (0),
    m_pktsSent(0)
{
  NS_LOG_FUNCTION (this);
}

BurstApplication::~BurstApplication()
{
  NS_LOG_FUNCTION (this);
}

void 
BurstApplication::SetMaxBytes (uint64_t maxBytes)
{
  NS_LOG_FUNCTION (this << maxBytes);
  m_maxBytes = maxBytes;
}

Ptr<Socket>
BurstApplication::GetSocket (void) const
{
  NS_LOG_FUNCTION (this);
  return m_socket;
}

uint64_t
BurstApplication::GetTotalTxPackets (void) const
{
  return m_txPackets;
}

uint64_t
BurstApplication::GetTotalTxBytes (void) const
{
  return m_totBytes;
}

uint32_t
BurstApplication::GetPktsPerBurst(void) const
{
    return m_pktsPerBurst;
}

int64_t 
BurstApplication::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_burstsPerSecond->SetStream (stream);
  return 2;
}

void
BurstApplication::DoDispose (void)
{
  NS_LOG_FUNCTION (this);

  CancelEvents ();
  m_socket = 0;
  // chain up
  Application::DoDispose ();
}

// Application Methods
void BurstApplication::StartApplication () // Called at time specified by Start
{
  NS_LOG_FUNCTION (this);

  // Create the socket if not already
  if (!m_socket)
    {
      m_socket = Socket::CreateSocket (GetNode (), m_tid);
      int ret = -1;

      if (! m_local.IsInvalid())
        {
          NS_ABORT_MSG_IF ((Inet6SocketAddress::IsMatchingType (m_peer) && InetSocketAddress::IsMatchingType (m_local)) ||
                           (InetSocketAddress::IsMatchingType (m_peer) && Inet6SocketAddress::IsMatchingType (m_local)),
                           "Incompatible peer and local address IP version");
          ret = m_socket->Bind (m_local);
        }
      else
        {
          if (Inet6SocketAddress::IsMatchingType (m_peer))
            {
              ret = m_socket->Bind6 ();
            }
          else if (InetSocketAddress::IsMatchingType (m_peer) ||
                   PacketSocketAddress::IsMatchingType (m_peer))
            {
              ret = m_socket->Bind ();
            }
        }

      if (ret == -1)
        {
          NS_FATAL_ERROR ("Failed to bind socket");
        }

      m_socket->Connect (m_peer);
      m_socket->SetAllowBroadcast (true);
      m_socket->ShutdownRecv ();

      m_socket->SetConnectCallback (
        MakeCallback (&BurstApplication::ConnectionSucceeded, this),
        MakeCallback (&BurstApplication::ConnectionFailed, this));
    }
  m_cbrRateFailSafe = m_cbrRate;

  uint32_t bitsPerPkt = m_pktSize * 8 - m_residualBits;
  double bitsPerBurst = static_cast<double>(m_cbrRate.GetBitRate ()) / m_burstsPerSecond->GetValue();
  m_pktsPerBurst = std::ceil(bitsPerBurst / bitsPerPkt);

  // Insure no pending event
  CancelEvents ();
  // If we are not yet connected, there is nothing to do here
  // The ConnectionComplete upcall will start timers at that time
  //if (!m_connected) return;
  ScheduleStartEvent ();
}

void BurstApplication::StopApplication () // Called at time specified by Stop
{
  NS_LOG_FUNCTION (this);

  CancelEvents ();
  if(m_socket != 0)
    {
      m_socket->Close ();
    }
  else
    {
      NS_LOG_WARN ("BurstApplication found null socket to close in StopApplication");
    }
}

void BurstApplication::CancelEvents ()
{
  NS_LOG_FUNCTION (this);

  if (m_sendEvent.IsRunning () && m_cbrRateFailSafe == m_cbrRate )
    { // Cancel the pending send packet event
      // Calculate residual bits since last packet sent
      Time delta (Simulator::Now () - m_lastStartTime);
      int64x64_t bits = delta.To (Time::S) * m_cbrRate.GetBitRate ();
      m_residualBits += bits.GetHigh ();
    }
  m_cbrRateFailSafe = m_cbrRate;
  Simulator::Cancel (m_sendEvent);
  Simulator::Cancel (m_startStopEvent);
}

// Event handlers
void BurstApplication::StartSending ()
{
  m_pktsSent = 0;
  NS_LOG_FUNCTION (this);
  m_lastStartTime = Simulator::Now ();
  ScheduleNextTx ();  // Schedule the send packet event
  ScheduleStartEvent ();
}

void BurstApplication::StopSending ()
{
  NS_LOG_FUNCTION (this);
  CancelEvents ();

}

// Private helpers
void BurstApplication::ScheduleNextTx ()
{
  NS_LOG_FUNCTION (this);

  if ((m_maxBytes == 0 || m_totBytes < m_maxBytes) && (m_maxPackets == 0 || m_txPackets < m_maxPackets))
    {
//      uint32_t bits = m_pktSize * 8 - m_residualBits;
//      NS_LOG_LOGIC ("bits = " << bits);
//      Time nextTime (Seconds (bits /
//                              static_cast<double>(m_cbrRate.GetBitRate ()))); // Time till next packet
//      NS_LOG_LOGIC ("nextTime = " << nextTime);
      m_sendEvent = Simulator::Schedule (MicroSeconds(0),
                                         &BurstApplication::SendPacket, this);
    }
  else
    { // All done, cancel any pending events
      StopApplication ();
    }
}

void BurstApplication::ScheduleStartEvent ()
{  // Schedules the event to start sending data (switch to the "On" state)
  NS_LOG_FUNCTION (this);

  Time offInterval = Seconds (1. / (m_burstsPerSecond->GetValue ()));
  NS_LOG_LOGIC ("start at " << offInterval);
  m_startStopEvent = Simulator::Schedule (offInterval, &BurstApplication::StartSending, this);
}


void BurstApplication::SendPacket ()
{
  NS_LOG_FUNCTION (this);

  NS_ASSERT (m_sendEvent.IsExpired ());

  Ptr<Packet> packet;
  if (m_enableE2EStats)
    {
      Address from, to;
      m_socket->GetSockName (from);
      m_socket->GetPeerName (to);
      E2eStatsHeader header;
      header.SetSeq (m_seq++);
      header.SetSize (m_pktSize);
      NS_ABORT_IF (m_pktSize < header.GetSerializedSize ());
      packet = Create<Packet> (m_pktSize - header.GetSerializedSize ());
      packet->AddHeader (header);
      m_txTraceWithStats (packet, from, to, header);
    }
  else
    {
      packet = Create<Packet> (m_pktSize);
    }

  if (m_enableTimestamp)
    {
      TimestampTag timestamp;
      timestamp.SetTimestamp (Simulator::Now ());
      packet->AddByteTag (timestamp);
      m_tagCreated(timestamp);
    }

  m_txTrace (packet);
  m_socket->Send (packet);
  m_totBytes += m_pktSize;
  m_txPackets++;
  Address localAddress;
  m_socket->GetSockName (localAddress);
  if (InetSocketAddress::IsMatchingType (m_peer))
    {
      NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds ()
                   << "s on-off application sent "
                   <<  packet->GetSize () << " bytes to "
                   << InetSocketAddress::ConvertFrom(m_peer).GetIpv4 ()
                   << " port " << InetSocketAddress::ConvertFrom (m_peer).GetPort ()
                   << " total Tx " << m_totBytes << " bytes");
      m_txTraceWithAddresses (packet, localAddress, InetSocketAddress::ConvertFrom (m_peer));
    }
  else if (Inet6SocketAddress::IsMatchingType (m_peer))
    {
      NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds ()
                   << "s on-off application sent "
                   <<  packet->GetSize () << " bytes to "
                   << Inet6SocketAddress::ConvertFrom(m_peer).GetIpv6 ()
                   << " port " << Inet6SocketAddress::ConvertFrom (m_peer).GetPort ()
                   << " total Tx " << m_totBytes << " bytes");
      m_txTraceWithAddresses (packet, localAddress, Inet6SocketAddress::ConvertFrom(m_peer));
    }
  m_lastStartTime = Simulator::Now ();
  m_residualBits = 0;

  m_pktsSent += 1;
  if (m_pktsSent >= m_pktsPerBurst) {
      return;
  }
  ScheduleNextTx ();
}


void BurstApplication::ConnectionSucceeded (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
  m_connected = true;
}

void BurstApplication::ConnectionFailed (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
  NS_FATAL_ERROR ("Can't connect");
}


} // Namespace ns3
