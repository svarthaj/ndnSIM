/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Copyright (c) 2011-2015  Regents of the University of California.
 *
 * This file is part of ndnSIM. See AUTHORS for complete list of ndnSIM authors and
 * contributors.
 *
 * ndnSIM is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * ndnSIM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ndnSIM, e.g., in COPYING.md file.  If not, see <http://www.gnu.org/licenses/>.
 **/

// ndn-simple.cpp

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ndnSIM-module.h"

#include "ns3/mobility-module.h"
#include <ns3/ndnSIM/utils/ndn-catalog.hpp>
#include "model/ndn-l3-protocol.hpp"
#include "ns3/ndnSIM/helper/ndn-link-control-helper.hpp"

using namespace std;

namespace ns3 {

void
CacheEntryRemoved(std::string context, Ptr<const ndn::cs::Entry> entry, Time lifetime)
{
    cout << "Cache entry removed: " << context << " " << entry->GetName() << " " << lifetime.ToDouble(Time::S) << "s" << std::endl;
}

int
main(int argc, char* argv[])
{
  // setting default parameters for PointToPoint links and channels
  Config::SetDefault("ns3::PointToPointNetDevice::DataRate", StringValue("1Mbps"));
  Config::SetDefault("ns3::PointToPointChannel::Delay", StringValue("10ms"));
  Config::SetDefault("ns3::DropTailQueue::MaxPackets", StringValue("100"));

  uint32_t n; 
  uint32_t ha;
  uint32_t obj;
  uint32_t cs;
  float req;

  // Read optional command-line parameters (e.g., enable visualizer with ./waf --run=<> --visualize
  CommandLine cmd;
  cmd.AddValue("n", "number of nodes", n);
  cmd.AddValue("ha", "home agent position", ha);
  cmd.AddValue("obj", "number of objects", obj);
  cmd.AddValue("cs", "cache size", cs);
  cmd.AddValue("req", "requisitions per object", req);
  cmd.Parse(argc, argv);

  // Setting simulation time
  //float simulation_time = 30000.0 / (n-1);
  float simulation_time = 250;
  
  // Creating nodes
  
  NodeContainer nodes;
  nodes.Create(n);

  NodeContainer consumers;
  for (int i = 0; i < n-1; i++) {
    consumers.Add(nodes.Get(i));
  }
  Ptr<Node> producer = nodes.Get(n-1); 
  Ptr<Node> homeagent = nodes.Get(ha);

  // Connecting nodes using two links
  PointToPointHelper p2p;
  Ptr<ns3::ndn::Catalog> catalog = Create<ns3::ndn::Catalog>();
  for (uint32_t i = 0; i < consumers.GetN()-1; i++) {
    p2p.Install(consumers.Get(i), consumers.Get(i+1));
  }

  for (uint32_t i = 0; i < consumers.GetN(); i++) {
    p2p.Install(consumers.Get(i), producer);
    catalog->addRouter(nodes.Get(i));
  }

  // Install NDN stack on all nodes
  ndn::StackHelper ndnHelper;
  //ndnHelper.SetOldContentStore("ns3::ndn::cs::Nocache");
  ndnHelper.SetOldContentStore("ns3::ndn::cs::Stats::Lru", "MaxSize", to_string(cs));
  ndnHelper.SetDefaultRoutes(false);
  ndnHelper.InstallAll();

  //Config::Connect("/NodeList/*/$ns3::ndn::cs::Stats::Lru/WillRemoveEntry", MakeCallback(CacheEntryRemoved));
  
  Ptr<RandomVariableStream> indexes = CreateObject<UniformRandomVariable>();
  indexes->SetAttribute("Min", DoubleValue(0));
  indexes->SetAttribute("Max", DoubleValue(n-1));
  Ptr<RandomPositionAllocator> positionAlloc = CreateObject<RandomPositionAllocator>();
  positionAlloc->SetIndex(indexes);

  // Installing applications

  string prefix = "/prod";
  string haprefix = "/ha0";
  string locprefix = "/loc";

  double freq = req * obj * 60.0 / simulation_time;
  // Probe Consumer
  ndn::AppHelper consumerHelper("ns3::ndn::ProbeConsumer");
  // Consumer will request /prefix/0, /prefix/1, ...
  consumerHelper.SetPrefix(prefix);
  consumerHelper.SetAttribute("Frequency", DoubleValue(freq)); // 6 interests a minute
  //consumerHelper.SetAttribute("Frequency", DoubleValue(6.0)); // 6 interests a minute
  consumerHelper.SetAttribute("Objects", UintegerValue(obj)); // 100 objects
  consumerHelper.Install(consumers).Start(Seconds(2));                     

  ndn::AppHelper agentHelper("ns3::ndn::ProbeAgent");
  consumerHelper.SetPrefix(haprefix);
  agentHelper.Install(homeagent);

  // Producer
  
  ndn::AppHelper producerHelper("ns3::ndn::ProbeProducer");
  // Producer will reply to all requests starting with /prefix
  producerHelper.SetPrefix(prefix);
  producerHelper.SetAttribute("PayloadSize", UintegerValue(1024));
  producerHelper.SetAttribute("Home", VectorValue(Vector(ha,0,0)));
  producerHelper.SetAttribute("Routers", PointerValue(catalog));
  producerHelper.Install(producer); // last node

  for (uint32_t i = 0; i < consumers.GetN(); i++) {
    if (i == ha) ndn::LinkControlHelper::UpLink(consumers.Get(i), producer);
    else ndn::LinkControlHelper::FailLink(consumers.Get(i), producer);
  }

  // Mobility
  MobilityHelper mobility;

  Ptr<RandomVariableStream> frequency;
  frequency = CreateObject<ConstantRandomVariable>();
  frequency->SetAttribute("Constant", DoubleValue(10));
  
  // Initialize positions of nodes
  mobility.SetPositionAllocator(positionAlloc);

  mobility.SetMobilityModel("ns3::RandomMobilityModel",
                            "Frequency", PointerValue(frequency),
                            "PositionAllocator", PointerValue(positionAlloc));

  mobility.Install(producer);

  ndn::GlobalRoutingHelper ndnGlobalRoutingHelper;
  ndnGlobalRoutingHelper.InstallAll();

  ndnGlobalRoutingHelper.AddOrigin(haprefix, homeagent);
  for (uint32_t i = 0; i < consumers.GetN(); i++) {
    ndnGlobalRoutingHelper.AddOrigin(locprefix + to_string(i), consumers.Get(i));
    ndnGlobalRoutingHelper.AddOrigin(locprefix + to_string(i) + prefix, producer);
  }

  // Calculate and install FIBs
  ndn::GlobalRoutingHelper::CalculateRoutes();
  //ndn::GlobalRoutingHelper::PrintFIBs();  
  
  Simulator::Stop(Seconds(simulation_time));

//  ndn::AppDelayTracer::Install(consumers, "saida.txt");
  ndn::AppDelayTracer::InstallAll("chain-indirection-n="+to_string(n)+"-cs="+to_string(cs)+"-ha="+to_string(ha)+"-obj="+to_string(obj)+"-req="+to_string(req)+".dat");

  Simulator::Run();
  Simulator::Destroy();

  return 0;
}

} // namespace ns3

int
main(int argc, char* argv[])
{
  return ns3::main(argc, argv);
}
