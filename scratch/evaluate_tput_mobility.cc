/*
 * Copyright (c) 2015-2020 IMDEA Networks Institute
 * Author: Hany Assasa <hany.assasa@gmail.com>
 */
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/energy-module.h"
#include "common-functions.h"
#include <iomanip>


NS_LOG_COMPONENT_DEFINE ("BeamformingCBAP");

using namespace ns3;
using namespace std;

/**  Application Variables **/
uint64_t totalRx = 0;
double throughput = 0;
Ptr<PacketSink> packetSink;
Ptr<BurstApplication> onoff;

/* Network Nodes */
Ptr<WifiNetDevice> apWifiNetDevice;
Ptr<WifiNetDevice> staWifiNetDevice;

NetDeviceContainer staDevices;

Ptr<DmgApWifiMac> apWifiMac;
Ptr<DmgStaWifiMac> staWifiMac;

/* Flow monitor */
Ptr<FlowMonitor> monitor;

DeviceEnergyModelContainer deviceModel;
double startUsage;
double stopUsage;

/*** Access Point Variables ***/
uint8_t stationsTrained = 0;              /* Number of BF trained stations */

/*** Beamforming Service Periods ***/
uint8_t beamformedLinks = 0;              /* Number of beamformed links */

void
SaveStartUsage(void) {
    startUsage = deviceModel.Get(0)->GetTotalEnergyConsumption ();
}

void
SaveStopUsage(void) {
    stopUsage = deviceModel.Get(0)->GetTotalEnergyConsumption ();
}

void
CalculateThroughput (void)
{
  double thr = CalculateSingleStreamThroughput (packetSink, totalRx, throughput);
  std::cout << std::left << std::setw (12) << Simulator::Now ().GetSeconds ()
            << std::left << std::setw (12) << thr << std::endl;
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput);
}

void RedoBeamform (void)
{
  staWifiMac->Perform_TXSS_TXOP(apWifiMac->GetAddress ());
  Simulator::Schedule (MilliSeconds (1000), &RedoBeamform);
}

void
StationAssoicated (Ptr<DmgStaWifiMac> staWifiMac, Mac48Address address, uint16_t aid)
{
  std::cout << "DMG STA " << staWifiMac->GetAddress () << " associated with DMG AP " << address << std::endl;
  std::cout << "Association ID (AID) = " << aid << std::endl;
  staWifiMac->Perform_TXSS_TXOP (apWifiMac->GetAddress ());
}

void
SLSCompleted (Ptr<DmgWifiMac> wifiMac, SlsCompletionAttrbitutes attributes)
{
  if (attributes.accessPeriod == CHANNEL_ACCESS_BHI)
    {
      if (wifiMac == apWifiMac)
        {
          std::cout << "DMG AP " << apWifiMac->GetAddress () <<
                       " completed SLS phase with DMG STA " << attributes.peerStation << std::endl;
        }
      else
        {
          std::cout << "DMG STA " << wifiMac->GetAddress ()
                    << " completed SLS phase with DMG AP " << attributes.peerStation << std::endl;
        }
      std::cout << "Best Tx Antenna Configuration: AntennaID=" << uint16_t (attributes.antennaID)
                << ", SectorID=" << uint16_t (attributes.sectorID) << std::endl;
    }
  else if (attributes.accessPeriod == CHANNEL_ACCESS_DTI)
    {
      beamformedLinks++;
      std::cout << "DMG STA " << wifiMac->GetAddress () << " completed SLS phase with DMG STA " << attributes.peerStation << std::endl;
      std::cout << "The best antenna configuration is AntennaID=" << uint16_t (attributes.antennaID)
                << ", SectorID=" << uint16_t (attributes.sectorID) << std::endl;
      if (beamformedLinks == 2)
        {
          apWifiMac->PrintSnrTable ();
          staWifiMac->PrintSnrTable ();
        }
    }
}

void
ActiveTxSectorIDChanged (Ptr<DmgWifiMac> wifiMac, SectorID oldSectorID, SectorID newSectorID)
{
//  std::cout << "DMG STA: " << wifiMac->GetAddress () << " , SectorID=" << uint16_t (newSectorID) << std::endl;
}

int
main (int argc, char *argv[])
{
  bool activateApp = true;                      /* Flag to indicate whether we activate onoff or bulk App */
  string applicationType = "onoff";              /* Type of the Tx application */
  string socketType = "ns3::UdpSocketFactory";  /* Socket Type (TCP/UDP) */
  uint32_t packetSize = 1448;                   /* Application payload size in bytes. */
  string dataRate = "400Mbps";                  /* Application data rate. */
  string tcpVariant = "NewReno";                /* TCP Variant Type. */
  uint32_t bufferSize = 131072;                 /* TCP Send/Receive Buffer Size. */
  uint32_t maxPackets = 0;                      /* Maximum Number of Packets */
  string msduAggSize = "max";                     /* The maximum aggregation size for A-MSDU in Bytes. */
  string mpduAggSize = "max";                  /* The maximum aggregation size for A-MSPU in Bytes. */
  string queueSize = "4000p";                   /* Wifi MAC Queue Size. */
  string phyMode = "DMG_MCS4";                 /* Type of the Physical Layer. */
  bool verbose = false;                         /* Print Logging Information. */
  double simulationTime = 50;                   /* Simulation time in seconds. */
  bool pcapTracing = false;                     /* PCAP Tracing is enabled or not. */

  /* Command line argument parser setup. */
  CommandLine cmd;
  cmd.AddValue ("activateApp", "Whether to activate data transmission or not", activateApp);
  cmd.AddValue ("applicationType", "Type of the Tx Application: onoff or bulk", applicationType);
  cmd.AddValue ("packetSize", "Application packet size in bytes", packetSize);
  cmd.AddValue ("dataRate", "Application data rate", dataRate);
  cmd.AddValue ("maxPackets", "Maximum number of packets to send", maxPackets);
  cmd.AddValue ("tcpVariant", TCP_VARIANTS_NAMES, tcpVariant);
  cmd.AddValue ("socketType", "Type of the Socket (ns3::TcpSocketFactory, ns3::UdpSocketFactory)", socketType);
  cmd.AddValue ("bufferSize", "TCP Buffer Size (Send/Receive) in Bytes", bufferSize);
  cmd.AddValue ("msduAggSize", "The maximum aggregation size for A-MSDU in Bytes", msduAggSize);
  cmd.AddValue ("mpduAggSize", "The maximum aggregation size for A-MPDU in Bytes", mpduAggSize);
  cmd.AddValue ("verbose", "Turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("simulationTime", "Simulation time in seconds", simulationTime);
  cmd.AddValue ("phyMode", "PHY Mode", phyMode);
  cmd.AddValue ("pcap", "Enable PCAP Tracing", pcapTracing);
  cmd.Parse (argc, argv);

  /* Validate A-MSDU and A-MPDU values */
  ValidateFrameAggregationAttributes (msduAggSize, mpduAggSize);
  /* Configure RTS/CTS and Fragmentation */
  ConfigureRtsCtsAndFragmenatation ();
  /* Wifi MAC Queue Parameters */
  ChangeQueueSize (queueSize);

  /*** Configure TCP Options ***/
  ConfigureTcpOptions (tcpVariant, packetSize, bufferSize);

  /**** DmgWifiHelper is a meta-helper ****/
  DmgWifiHelper wifi;

  /* Basic setup */
  wifi.SetStandard (WIFI_PHY_STANDARD_80211ad);

  /* Turn on logging */
  if (verbose)
    {
      wifi.EnableLogComponents ();
      LogComponentEnable ("BeamformingCBAP", LOG_LEVEL_ALL);
    }

  /**** Set up Channel ****/
  DmgWifiChannelHelper wifiChannel ;
  /* Simple propagation delay model */
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  /* Friis model with standard-specific wavelength */
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (60.48e9));

  /**** Setup physical layer ****/
  DmgWifiPhyHelper wifiPhy = DmgWifiPhyHelper::Default ();
  /* Nodes will be added to the channel we set up earlier */
  wifiPhy.SetChannel (wifiChannel.Create ());
  /* All nodes transmit at 10 dBm == 10 mW, no adaptation */
  wifiPhy.Set ("TxPowerStart", DoubleValue (10.0));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (10.0));
  wifiPhy.Set ("TxPowerLevels", UintegerValue (1));
  /* Set operating channel */
  wifiPhy.Set ("ChannelNumber", UintegerValue (2));
  /* Set default algorithm for all nodes to be constant rate */
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue (phyMode));
//  wifi.SetRemoteStationManager ("ns3::CbtraaDmgWifiManager");

  /* Make four nodes and set them up with the phy and the mac */
  NodeContainer wifiNodes;
  wifiNodes.Create (2);
  Ptr<Node> apNode = wifiNodes.Get (0);
  Ptr<Node> staNode = wifiNodes.Get (1);

  /* Add a DMG upper mac */
  DmgWifiMacHelper wifiMac = DmgWifiMacHelper::Default ();

  /* Install DMG PCP/AP Node */
  Ssid ssid = Ssid ("BeamformingCBAP");
  wifiMac.SetType ("ns3::DmgApWifiMac",
                   "Ssid", SsidValue(ssid),
                   "BE_MaxAmpduSize", StringValue (mpduAggSize),
                   "BE_MaxAmsduSize", StringValue (msduAggSize),
                   "SSSlotsPerABFT", UintegerValue (8), "SSFramesPerSlot", UintegerValue (8),
                   "BeaconInterval", TimeValue (MicroSeconds (102400)));

  /* Set Analytical Codebook for the DMG Devices */
  wifi.SetCodebook ("ns3::CodebookAnalytical",
                    "CodebookType", EnumValue (SIMPLE_CODEBOOK),
                    "Antennas", UintegerValue (1),
                    "Sectors", UintegerValue (8));

  NetDeviceContainer apDevice;
  apDevice = wifi.Install (wifiPhy, wifiMac, apNode);

  /* Install DMG STA Nodes */
  wifiMac.SetType ("ns3::DmgStaWifiMac",
                   "Ssid", SsidValue (ssid), "ActiveProbing", BooleanValue (false),
                   "BE_MaxAmpduSize", StringValue (mpduAggSize),
                   "BE_MaxAmsduSize", StringValue (msduAggSize));

  staDevices = wifi.Install (wifiPhy, wifiMac, staNode);

  /* Setting mobility model */
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));   /* DMG PCP/AP */
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));   /* DMG STA */

  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (wifiNodes);

  (wifiNodes.Get(0) -> GetObject<ConstantVelocityMobilityModel>()) -> SetVelocity(Vector(0.0, 0.0, 0.0));
  (wifiNodes.Get(1) -> GetObject<ConstantVelocityMobilityModel>()) -> SetVelocity(Vector(0.0, 0.5, 0.0));

  /* energy source */
  BasicEnergySourceHelper basicSourceHelper;
  // configure energy source
  basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (999999999));
  basicSourceHelper.Set ("BasicEnergySupplyVoltageV", DoubleValue(1));
  // install source
  EnergySourceContainer sources = basicSourceHelper.Install (staNode);
  /* device energy model */
  WifiRadioEnergyModelHelper radioEnergyHelper;
  // configure radio energy model
  radioEnergyHelper.Set ("TxCurrentA", DoubleValue (3.9));
  radioEnergyHelper.Set ("RxCurrentA", DoubleValue (4.0));
  radioEnergyHelper.Set ("CcaBusyCurrentA", DoubleValue (2.6));
  radioEnergyHelper.Set ("SwitchingCurrentA", DoubleValue (2.6));
  radioEnergyHelper.Set ("IdleCurrentA", DoubleValue (2.6));
//  radioEnergyHelper.Set ("SleepCurrentA", DoubleValue (2.0));

  // install device model
  deviceModel = radioEnergyHelper.Install (staDevices, sources);

  /* Internet stack*/
  InternetStackHelper stack;
  stack.Install (wifiNodes);

  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.255.255.0");
  Ipv4InterfaceContainer apInterface;
  apInterface = address.Assign (apDevice);
  Ipv4InterfaceContainer staInterfaces;
  staInterfaces = address.Assign (staDevices);

  /* Populate routing table */
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  /* We do not want any ARP packets */
  PopulateArpCache ();

  if (activateApp)
    {
      /* Install Simple UDP Server on the DMG AP */
      PacketSinkHelper sinkHelper (socketType, InetSocketAddress (Ipv4Address::GetAny (), 9999));
      ApplicationContainer sinkApp = sinkHelper.Install (staNode);
      packetSink = StaticCast<PacketSink> (sinkApp.Get (0));
      sinkApp.Start (Seconds (0.0));

      /* Install TCP/UDP Transmitter on the DMG STA */
      Address dest (InetSocketAddress (staInterfaces.GetAddress (0), 9999));
      ApplicationContainer srcApp;
      if (applicationType == "onoff")
        {
          BurstHelper src (socketType, dest);
          src.SetAttribute ("MaxPackets", UintegerValue (maxPackets));
          src.SetAttribute ("PacketSize", UintegerValue (packetSize));
          src.SetAttribute ("BurstsPerSecond", StringValue ("ns3::ConstantRandomVariable[Constant=100]"));
          src.SetAttribute ("DataRate", DataRateValue (DataRate (dataRate)));
          srcApp = src.Install (apNode);
          onoff = StaticCast<BurstApplication> (srcApp.Get (0));
        }
      srcApp.Start (Seconds (2.0));
      srcApp.Stop (Seconds (simulationTime));
    }

  /* Enable Traces */
  if (pcapTracing)
    {
      wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
      wifiPhy.EnablePcap ("Traces/AccessPoint", apDevice, false);
      wifiPhy.EnablePcap ("Traces/StaNode", staDevices.Get (0), false);
    }

  /* Stations */
  apWifiNetDevice = StaticCast<WifiNetDevice> (apDevice.Get (0));
  staWifiNetDevice = StaticCast<WifiNetDevice> (staDevices.Get (0));

  apWifiMac = StaticCast<DmgApWifiMac> (apWifiNetDevice->GetMac ());
  staWifiMac = StaticCast<DmgStaWifiMac> (staWifiNetDevice->GetMac ());

  /** Connect Traces **/
  staWifiMac->TraceConnectWithoutContext ("Assoc", MakeBoundCallback (&StationAssoicated, staWifiMac));
  apWifiMac->TraceConnectWithoutContext ("SLSCompleted", MakeBoundCallback (&SLSCompleted, apWifiMac));
  staWifiMac->TraceConnectWithoutContext ("SLSCompleted", MakeBoundCallback (&SLSCompleted, staWifiMac));

  apWifiMac->GetCodebook ()->TraceConnectWithoutContext ("ActiveTxSectorID", MakeBoundCallback (&ActiveTxSectorIDChanged, apWifiMac));
  staWifiMac->GetCodebook ()->TraceConnectWithoutContext ("ActiveTxSectorID", MakeBoundCallback (&ActiveTxSectorIDChanged, staWifiMac));

  FlowMonitorHelper flowmon;
  if (activateApp)
    {
      /* Install FlowMonitor on all nodes */
      monitor = flowmon.InstallAll ();

      /* Print Output*/
      std::cout << std::left << std::setw (12) << "Time [s]"
                << std::left << std::setw (12) << "Throughput [Mbps]" << std::endl;

      /* Schedule Throughput Calulcations */
      Simulator::Schedule (Seconds (2.1), &CalculateThroughput);
      Simulator::Schedule (Seconds (2.1), &RedoBeamform);
    }

  Simulator::Schedule(Seconds(2.0), &SaveStartUsage);
  Simulator::Schedule(Seconds(simulationTime), &SaveStopUsage);

  Simulator::Stop (Seconds (simulationTime + 0.101));
  Simulator::Run ();

  Simulator::Destroy ();

  if (activateApp)
    {
      PrintFlowMonitorStatistics (flowmon, monitor, simulationTime - 1);

      /* Print Application Layer Results Summary */
      std::cout << "\nApplication Layer Statistics:" << std::endl;
      if (applicationType == "onoff")
        {
          std::cout << "  Tx Packets: " << onoff->GetTotalTxPackets () << std::endl;
          std::cout << "  Tx Bytes:   " << onoff->GetTotalTxBytes () << std::endl;
        }
    }

  std::cout << "  Rx Packets: " << packetSink->GetTotalReceivedPackets () << std::endl;
  std::cout << "  Rx Bytes:   " << packetSink->GetTotalRx () << std::endl;
  std::cout << "  Throughput: " << packetSink->GetTotalRx () * 8.0 / ((simulationTime - 1) * 1e6) << " Mbps" << std::endl;

  std::cout << "  Total usage: " << (stopUsage - startUsage) << "J" << std::endl;
  std::cout << "   Average wattage: " << (stopUsage - startUsage) / (simulationTime - 2.0) << "W" << std::endl;

  return 0;
}
