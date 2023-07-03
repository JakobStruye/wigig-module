/*
 * Copyright (c) 2015-2020 IMDEA Networks Institute
 * Author: Hany Assasa <hany.assasa@gmail.com>
 */
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "common-functions.h"
#include "ns3/spectrum-module.h"

NS_LOG_COMPONENT_DEFINE ("Jakob");

using namespace ns3;
using namespace std;

/**  Application Variables **/
Ptr<PacketSink> packetSink;

/* Network Nodes */
Ptr<Node> apWifiNode, staWifiNode;
Ptr<WifiNetDevice> staWifiNetDevice, apWifiNetDevice;
Ptr<DmgWifiPhy> staWifiPhy, apWifiPhy;
Ptr<WifiRemoteStationManager> staRemoteStationManager;

int
main (int argc, char *argv[])
{
    uint32_t packetSize = 1472;                   /* Application payload size in bytes. */
    string socketType = "ns3::UdpSocketFactory";  /* Socket Type (TCP/UDP) */
    string tcpVariant = "NewReno";                /* TCP Variant Type. */
    uint32_t bufferSize = 131072;                 /* TCP Send/Receive Buffer Size. */
    string msduAggSize = "max";                   /* The maximum aggregation size for A-MSDU in Bytes. */
    string mpduAggSize = "max";                   /* The maximum aggregation size for A-MPDU in Bytes. */
    string queueSize = "4000p";                   /* Wifi MAC Queue Size. */
    bool enableRts = false;                       /* Flag to indicate if RTS/CTS handskahre is enabled or disabled. */
    uint32_t rtsThreshold = 0;                    /* RTS/CTS handshare threshold. */
    double simulationTime = 1;                    /* Simulation time in seconds per MCS. */

    /* Command line argument parser setup. */
    CommandLine cmd;
    cmd.AddValue ("packetSize", "Application packet size in bytes", packetSize);
    cmd.AddValue ("tcpVariant", TCP_VARIANTS_NAMES, tcpVariant);
    cmd.AddValue ("socketType", "Type of the Socket (ns3::TcpSocketFactory, ns3::UdpSocketFactory)", socketType);
    cmd.AddValue ("msduAggSize", "The maximum aggregation size for A-MSDU in Bytes", msduAggSize);
    cmd.AddValue ("mpduAggSize", "The maximum aggregation size for A-MPDU in Bytes", mpduAggSize);
    cmd.AddValue ("enableRts", "Enable or disable RTS/CTS handshake", enableRts);
    cmd.AddValue ("queueSize", "The maximum size of the Wifi MAC Queue", queueSize);
    cmd.AddValue ("simulationTime", "Simulation time in Seconds per MCS", simulationTime);
    cmd.Parse (argc, argv);

    /* Validate WiGig standard value */
    WifiPhyStandard wifiStandard = WIFI_PHY_STANDARD_80211ay;
    string wifiModePrefix = "EDMG_SC";
    uint mcs = 12;        /* The maximum MCS index. */

    ValidateFrameAggregationAttributes (msduAggSize, mpduAggSize, wifiStandard);
    ConfigureRtsCtsAndFragmenatation (enableRts, rtsThreshold);
    ChangeQueueSize (queueSize);

    AsciiTraceHelper ascii;       /* ASCII Helper. */
    Ptr<OutputStreamWrapper> outputFile = ascii.CreateFileStream ("Throughput_Jakob.csv");
    *outputFile->GetStream () << "MODE,THROUGHPUT" << std::endl;
    uint8_t channel = 2;//2, 9, 17, 25}; // 2.16, 4.32, 6.48, 8.64 GHz.

    DmgWifiHelper wifi;
    wifi.SetStandard (wifiStandard);

    DmgWifiChannelHelper wifiChannel ;
    wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (60.48e9));

    /**** Setup physical layer ****/
    DmgWifiPhyHelper wifiPhy = DmgWifiPhyHelper::Default ();
    wifiPhy.SetChannel (wifiChannel.Create ());
    wifiPhy.Set ("TxPowerStart", DoubleValue (10.0));
    wifiPhy.Set ("TxPowerEnd", DoubleValue (10.0));
    wifiPhy.Set ("TxPowerLevels", UintegerValue (1));
    /* Set operating channel */
    EDMG_CHANNEL_CONFIG config = FindChannelConfiguration (channel);
    wifiPhy.Set ("ChannelNumber", UintegerValue (config.chNumber));
    wifiPhy.Set ("PrimaryChannelNumber", UintegerValue (config.primayChannel));
    /* Add support for the OFDM PHY */
    wifiPhy.Set ("SupportOfdmPhy", BooleanValue (false));
    wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode",
                                  StringValue (wifiModePrefix + "_MCS" + std::to_string (mcs)));
    wifiPhy.SetErrorRateModel ("ns3::DmgErrorModel",
                               "FileName", StringValue ("WigigFiles/ErrorModel/LookupTable_1458_ay.txt"));

    /* Make two nodes and set them up with the PHY and the MAC */
    NodeContainer wifiNodes;
    wifiNodes.Create (2);
    apWifiNode = wifiNodes.Get (0);
    staWifiNode = wifiNodes.Get (1);

    /* Add a DMG upper mac */
    DmgWifiMacHelper wifiMac = DmgWifiMacHelper::Default ();
    Ssid ssid = Ssid ("test");

    /* Set Analytical Codebook for the WiGig Devices */
    wifi.SetCodebook ("ns3::CodebookAnalytical",
                      "CodebookType", EnumValue (SIMPLE_CODEBOOK),
                      "Antennas", UintegerValue (1),
                      "Sectors", UintegerValue (8));

    /* Create Wifi Network Devices (WifiNetDevice) */
    wifiMac.SetType ("ns3::DmgApWifiMac",
                     "Ssid", SsidValue (ssid),
                     "BE_MaxAmpduSize", StringValue (mpduAggSize),
                     "BE_MaxAmsduSize", StringValue (msduAggSize),
                     "SSSlotsPerABFT", UintegerValue (8), "SSFramesPerSlot", UintegerValue (8),
                     "BeaconInterval", TimeValue (MicroSeconds (102400)),
                     "EDMGSupported", BooleanValue (true));

    NetDeviceContainer apDevice;
    apDevice = wifi.Install (wifiPhy, wifiMac, apWifiNode);

    wifiMac.SetType ("ns3::DmgStaWifiMac",
                     "Ssid", SsidValue (ssid), "ActiveProbing", BooleanValue (false),
                     "BE_MaxAmpduSize", StringValue (mpduAggSize),
                     "BE_MaxAmsduSize", StringValue (msduAggSize),
                     "EDMGSupported", BooleanValue (true));

    NetDeviceContainer staDevice;
    staDevice = wifi.Install (wifiPhy, wifiMac, staWifiNode);
    wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
//                wifiPhy.SetSnapshotLength (snapshotLength);
    wifiPhy.EnablePcap ("Traces/AccessPoint", apDevice, false);
    wifiPhy.EnablePcap ("Traces/STA", staDevice, false);

    /* Setting mobility model */
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
    positionAlloc->Add (Vector (0.0, 0.0, 0.0));        /* WiGig PCP/AP */
    positionAlloc->Add (Vector (2.0, 0.0, 2.0));        /* WiGig STA */

    mobility.SetPositionAllocator (positionAlloc);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (wifiNodes);

    /* Internet stack*/
    InternetStackHelper stack;
    stack.Install (wifiNodes);

    Ipv4AddressHelper address;
    address.SetBase ("10.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterface;
    apInterface = address.Assign (apDevice);
    Ipv4InterfaceContainer staInterface;
    staInterface = address.Assign (staDevice);

    /* Populate routing table */
    Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

    /* We do not want any ARP packets */
    PopulateArpCache ();

    /* Install Simple UDP Server on the WiGig PCP/AP */
    PacketSinkHelper sinkHelper (socketType, InetSocketAddress (Ipv4Address::GetAny (), 9999));
    ApplicationContainer sinkApp = sinkHelper.Install (apWifiNode);
    packetSink = StaticCast<PacketSink> (sinkApp.Get (0));
    sinkApp.Start (Seconds (0.0));

    /* Get the nominal PHY rate and use it as the data rate of the application */
    WifiMode mode = WifiMode (wifiModePrefix + "_MCS" + to_string (mcs));
    uint64_t dataRate = mode.GetPhyRate () * config.NCB;

    /* Install TCP/UDP Transmitter on the WiGig STA */
    ApplicationContainer srcApp;
    Address dest (InetSocketAddress (apInterface.GetAddress (0), 9999));
    if (socketType == "ns3::UdpSocketFactory")
    {
        OnOffHelper src (socketType, dest);
        src.SetAttribute ("MaxPackets", UintegerValue (0));
        src.SetAttribute ("PacketSize", UintegerValue (packetSize));
        src.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1e6]"));
        src.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
        src.SetAttribute ("DataRate", DataRateValue (DataRate (dataRate)));
        srcApp = src.Install (staWifiNode);
    }
    else
    {
        /*** Configure TCP Options ***/
        bufferSize = 8e-3 * dataRate/8; // We assume RTT = 10ms, the data rate is given in bits.
        ConfigureTcpOptions (tcpVariant, packetSize, bufferSize);

        BulkSendHelper src (socketType, dest);
        srcApp = src.Install (staWifiNode);
    }
    srcApp.Start (Seconds (0.2)); //Don't send until connected
    srcApp.Stop (Seconds (simulationTime + 0.2));

    Simulator::Stop (Seconds (simulationTime + 0.2));
    Simulator::Run ();
    Simulator::Destroy ();

    *outputFile->GetStream () << wifiModePrefix << "," << mcs << "," << uint16_t (config.NCB) << ","
                              << packetSink->GetTotalRx () * (double) 8/1e6 / simulationTime << std::endl;


    return 0;
}
