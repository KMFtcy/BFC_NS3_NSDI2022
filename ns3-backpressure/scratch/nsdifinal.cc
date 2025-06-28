/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation;
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have receivedr a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#undef PGO_TRAINING
#define PATH_TO_PGO_CONFIG "path_to_pgo_config"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>
#include <time.h>
#include <random>
#include <math.h>
#include "ns3/core-module.h"
#include "ns3/qbb-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/global-route-manager.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/broadcom-node.h"
#include "ns3/packet.h"
#include "ns3/error-model.h"
#include "ns3/qbb-net-device.h"
#include "ns3/qbb-channel.h"
#include <random>
#include <vector>
#include <set>
#include <unordered_map>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("GENERIC_SIMULATION");

bool use_dynamic_pfc_threshold = true, packet_level_ecmp = false, flow_level_ecmp = true;
uint32_t packet_payload_size = 1000, l2_chunk_size = 4000, l2_ack_interval = 256;
double pause_time = 5, simulator_stop_time = 3.01, app_start_time = 1.0, app_stop_time = 9.0;
std::string data_rate, link_delay, topology_file, flow_file, tcp_flow_file, trace_file, trace_output_file;
bool used_port[65536 * 64] = {0};
std::string fct_output_file = "fct.txt";

struct Interface
{
    uint32_t idx;
    bool up;
    uint64_t delay;
    uint64_t bw;

    Interface()
        : idx(0),
          up(false)
    {
    }
};
uint64_t maxRtt = 0, maxBdp = 0;
map<Ptr<Node>, map<Ptr<Node>, vector<Ptr<Node>>>> nextHop;
map<Ptr<Node>, map<Ptr<Node>, uint64_t>> pairDelay;
map<Ptr<Node>, map<Ptr<Node>, uint64_t>> pairTxDelay;
map<uint32_t, map<uint32_t, uint64_t>> pairBw;
map<Ptr<Node>, map<Ptr<Node>, uint64_t>> pairBdp;
map<uint32_t, map<uint32_t, uint64_t>> pairRtt;
map<Ptr<Node>, map<Ptr<Node>, Interface>> nbr2if;

double cnp_interval = 50, alpha_resume_interval = 55, rp_timer, dctcp_gain = 1 / 16, np_sampling_interval = 0, pmax = 1;
uint32_t byte_counter, fast_recovery_times = 5, kmax = 60, kmin = 60;
std::string rate_ai, rate_hai;

bool clamp_target_rate = false, clamp_target_rate_after_timer = false, send_in_chunks = true, l2_back_to_zero = false, l2_test_read = false;
double error_rate_per_link = 0.0;

double x;
double sigma = 2.0;
bool create_incast = false;

uint32_t datarate = 100;
double utilization_factor = 0.6;
bool enable_window = false;
uint32_t packet_size_incast = 20;
uint32_t num_sources = 200;

uint32_t qcount = 128;

bool w_dctcp = false;
bool w_3 = false;
bool w_4 = false;

uint32_t fixed_window_size = 150;

bool enable_buffer_stats = true;

int min_size = 100;

std::vector<uint64_t> enterprise_size_conga{
	250, 270, 286, 297, 312, 327, 342, 356, 368, 383, 404, 427, 447,
	469, 490, 512, 536, 561, 587, 612, 636, 651, 674, 696, 708, 729,
	753, 782, 820, 857, 900, 945, 990, 1036, 1086, 1140, 1200, 1260,
	1311, 1353, 1393, 1452, 1485, 1510, 1540, 1597, 1658, 1721, 1784,
	1873, 1978, 2079, 2160, 2267, 2395, 2536, 2688, 2862, 2960, 3040,
	3163, 3390, 3603, 3836, 4124, 4406, 4540, 4673, 4952, 5260, 5625,
	5938, 6088, 6506, 6976, 7430, 7800, 8332, 8849, 9477, 10344, 11256,
	12396, 13766, 15370, 17417, 19884, 22822, 26285, 30973, 37152, 45274,
	55796, 70997, 94050, 131474, 189792, 344538, 898190, 2221957, 2624209,
	3114668, 3849851, 4904979, 6438497, 8608848, 13392072, 23110447,
	40607470, 44466961, 48564256, 53520618, 59661017, 67292218, 76720988,
	92984411, 121274824, 168707183, 176569410, 184086600, 195504461,
	211698244, 230332119, 249070260, 293849828, 361127656, 440186178,
	442950536, 449947224, 466035021, 482515012, 510665738, 561765744,
	669621992, 738727896, 1022256099, 1773379248};

std::vector<double> enterprise_prob_conga{
	0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1,
	0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18, 0.19, 0.2, 0.21,
	0.22, 0.23, 0.24, 0.25, 0.26, 0.27, 0.28, 0.29, 0.3, 0.31, 0.32,
	0.33, 0.34, 0.35, 0.36, 0.37, 0.38, 0.39, 0.4, 0.41, 0.42, 0.43,
	0.44, 0.45, 0.46, 0.47, 0.48, 0.49, 0.5, 0.51, 0.52, 0.53, 0.54,
	0.55, 0.56, 0.57, 0.58, 0.59, 0.6, 0.61, 0.62, 0.63, 0.64, 0.65,
	0.66, 0.67, 0.68, 0.69, 0.7, 0.71, 0.72, 0.73, 0.74, 0.75, 0.76,
	0.77, 0.78, 0.79, 0.8, 0.81, 0.82, 0.83, 0.84, 0.85, 0.86, 0.87,
	0.88, 0.89, 0.9, 0.91, 0.92, 0.93, 0.94, 0.95, 0.96, 0.97, 0.98,
	0.99, 0.991, 0.992, 0.993, 0.994, 0.995, 0.996, 0.997, 0.998,
	0.999, 0.9991, 0.9992, 0.9993, 0.9994, 0.9995, 0.9996, 0.9997,
	0.9998, 0.9999, 0.99991, 0.99992, 0.99993, 0.99994, 0.99995,
	0.99996, 0.99997, 0.99998, 0.99999, 0.999991, 0.999992, 0.999993,
	0.999994, 0.999995, 0.999996, 0.999997, 0.999998, 0.999999, 1.0};

//DCTCP
std::vector<uint64_t> enterprise_size_dctcp{0, 100, 10000, 20000, 30000, 50000, 80000, 200000, 1000000, 2000000, 5000000, 10000000, 30000000};
std::vector<double> enterprise_prob_dctcp{0.0, 0.0, 0.15, 0.2, 0.3, 0.4, 0.53, 0.6, 0.7, 0.8, 0.9, 0.97, 1.0};

std::vector<uint64_t> enterprise_size;
std::vector<double> enterprise_prob;

std::set<double> probs;
std::unordered_map<double, int> size_map;

std::ifstream topof, flowf, tracef;
NodeContainer n;
// maintain port number for each host pair
std::unordered_map<uint32_t, unordered_map<uint32_t, uint16_t>> portNumder;
// maintain ip to node id
std::unordered_map<uint32_t, uint32_t> ip_to_nodeid;

struct FlowInput
{
    uint32_t src, dst, pg, maxPacketCount, port, dport;
    double start_time;
    uint32_t idx;
};

FlowInput flow_input = {0};
uint32_t flow_num = 0;

// void
// ReadFlowInput()
// {
//     if (flow_input.idx < flow_num)
//     {
//         flowf >> flow_input.src >> flow_input.dst >> flow_input.pg >> flow_input.dport >>
//             flow_input.maxPacketCount >> flow_input.start_time;

//         auto srcNode = n.Get(flow_input.src);
//         auto dstNode = n.Get(flow_input.dst);

//         NS_LOG_DEBUG("[Debug] Flow #" << flow_input.idx << " src=" << flow_input.src
//                   << " (type=" << srcNode->GetNodeType() << ")"
//                   << ", dst=" << flow_input.dst << " (type=" << dstNode->GetNodeType() << ")"
//                   << std::endl);
//         NS_ASSERT(n.Get(flow_input.src)->GetNodeType() == 0 &&
//                   n.Get(flow_input.dst)->GetNodeType() == 0);
//     }
// }

// void
// ScheduleFlowInputs()
// {
//     while (flow_input.idx < flow_num && Seconds(flow_input.start_time) == Simulator::Now())
//     {
//         size_t winSize = (global_t == 1 ? maxBdp : pairBdp[n.Get(flow_input.src)][n.Get(flow_input.dst)]);
//         // has_win is effective in all cases except when use_coding_transport is true and pg != 2.
//         // In other words, when use_coding_transport is true, only pg==2 flows can use has_win.
//         bool isSetWin = has_win && (!use_coding_transport || flow_input.pg == 2);
//         uint32_t port = portNumder[flow_input.src][flow_input.dst]++; // get a new port number
//         RdmaClientHelper clientHelper(
//             flow_input.pg,
//             serverAddress[flow_input.src],
//             serverAddress[flow_input.dst],
//             port,
//             flow_input.dport,
//             flow_input.maxPacketCount,
//             isSetWin ? winSize : 0,
//             global_t == 1 ? maxRtt : pairRtt[flow_input.src][flow_input.dst]);
//         ApplicationContainer appCon = clientHelper.Install(n.Get(flow_input.src));
//         appCon.Start(Time(0));

//         // get the next flow input
//         flow_input.idx++;
//         ReadFlowInput();
//     }

//     // schedule the next time to run this function
//     if (flow_input.idx < flow_num)
//     {
//         Simulator::Schedule(Seconds(flow_input.start_time) - Simulator::Now(), ScheduleFlowInputs);
//     }
//     else
//     { // no more flows, close the file
//         flowf.close();
//     }
// }

void
qp_finish(FILE* fout, std::unordered_map<uint32_t, uint32_t>* ip_to_node_id, uint32_t sip, uint32_t dip, uint32_t sport, uint32_t dport, uint32_t pg, Time start_time, uint32_t num_packets)
{
	// print all parameters
	// NS_LOG_INFO("sip: " << sip << " dip: " << dip << " sport: " << sport << " dport: " << dport << " pg: " << pg << " start_time: " << start_time.GetTimeStep() << " num_packets: " << num_packets);
    uint32_t sid = ip_to_node_id->at(sip);
    uint32_t did = ip_to_node_id->at(dip);
    uint64_t base_rtt = pairDelay[n.Get(sid)][n.Get(did)], b = pairBw[sid][did];
    uint64_t total_bytes =
        num_packets * packet_payload_size + num_packets*4;
    uint64_t standalone_fct = base_rtt + total_bytes * 8000000000lu / b;
	// NS_LOG_INFO("sid: " << sid << " did: " << did << " base_rtt: " << base_rtt << " b: " << b << " total_bytes: " << total_bytes << " standalone_fct: " << standalone_fct);
    // // sip, dip, source node id, destination node id, sport, dport,  priority group, size (B), start_time, fct (ns), standalone_fct (ns)
    fprintf(fout,
            "%08x %08x %u %u %u %u %lu %lu %lu %lu %lu\n",
            sip,
            dip,
            sid,
            did,
            sport,
            dport,
            pg,
            num_packets * packet_payload_size,
            start_time.GetTimeStep(),
            (Simulator::Now() - start_time).GetTimeStep(),
            standalone_fct);
    fflush(fout);

}

void
CalculateRoute(Ptr<Node> host)
{
    // queue for the BFS.
    vector<Ptr<Node>> q;
    // Distance from the host to each node.
    map<Ptr<Node>, int> dis;
    map<Ptr<Node>, uint64_t> delay;
    map<Ptr<Node>, uint64_t> txDelay;
    map<Ptr<Node>, uint64_t> bw;
    // init BFS.
    q.push_back(host);
    dis[host] = 0;
    delay[host] = 0;
    txDelay[host] = 0;
    bw[host] = 0xfffffffffffffffflu;
    // BFS.
    for (int i = 0; i < (int)q.size(); i++)
    {
        Ptr<Node> now = q[i];
        int d = dis[now];
        for (auto it = nbr2if[now].begin(); it != nbr2if[now].end(); it++)
        {
            // skip down link
            if (!it->second.up)
            {
                continue;
            }
            Ptr<Node> next = it->first;
            // If 'next' have not been visited.
            if (dis.find(next) == dis.end())
            {
                dis[next] = d + 1;
                delay[next] = delay[now] + it->second.delay;
                txDelay[next] =
                    txDelay[now] + packet_payload_size * 1000000000lu * 8 / it->second.bw;
                bw[next] = std::min(bw[now], it->second.bw);
                // we only enqueue switch, because we do not want packets to go through host as
                // middle point
                if (next->GetNodeType() == 1)
                {
                    q.push_back(next);
                }
            }
            // if 'now' is on the shortest path from 'next' to 'host'.
            if (d + 1 == dis[next])
            {
                nextHop[next][host].push_back(now);
            }
        }
    }
    for (auto it : delay)
    {
        pairDelay[it.first][host] = it.second;
    }
    for (auto it : txDelay)
    {
        pairTxDelay[it.first][host] = it.second;
    }
    for (auto it : bw)
    {
        pairBw[it.first->GetId()][host->GetId()] = it.second;
    }
}

bool poison_distr = true;
int main(int argc, char *argv[])
{

	clock_t begint, endt;
	begint = clock();
#ifndef PGO_TRAINING
	//if (argc > 1)
	if (true)
#else
	if (true)
#endif
	{
		//Read the configuration file
		std::ifstream conf;
#ifndef PGO_TRAINING
		conf.open(argv[1]);
#else
		conf.open(PATH_TO_PGO_CONFIG);
#endif
		while (!conf.eof())
		{
			std::string key;
			conf >> key;

			//std::cout << conf.cur << "\n";

			if (key.compare("WORKLOAD") == 0)
			{
				std::string v;
				conf >> v;
				if (v == "W3")
				{
					std::cout << "WORKLOAD\t\t\t"
							  << "W3"
							  << "\n";
					w_3 = true;
				}
				else if (v == "W4")
				{
					std::cout << "WORKLOAD\t\t\t"
							  << "W4"
							  << "\n";
					w_4 = true;
				}
				else
				{
					std::cout << "WORKLOAD\t\t\t"
							  << "DCTCP"
							  << "\n";
					w_dctcp = true;
				}
			}
			else if (key.compare("PAUSE_TIME") == 0)
			{
				double v;
				conf >> v;
				pause_time = v;
				std::cout << "PAUSE_TIME\t\t\t" << pause_time << "\n";
			}
			else if (key.compare("DATA_RATE") == 0)
			{
				std::string v;
				conf >> v;
				data_rate = v;
				// std::cout<<"--------------------------------------\n";
				// std::cout<<"NOTE THAT DATARATE IS IGNORED AND THE DEFAULT CHOSEN\n";
				// std::cout<<"--------------------------------------\n";
				std::cout << "DATA_RATE\t\t\t" << data_rate << "\n";
				datarate = uint32_t(std::stoi(data_rate.substr(0, data_rate.size() - 4)));
			}
			else if (key.compare("UTILIZATION_FACTOR") == 0)
			{
				double v;
				conf >> v;
				utilization_factor = v;
				std::cout << "UTILIZATION_FACTOR\t\t\t" << utilization_factor << "\n";
			}
			else if (key.compare("LINK_DELAY") == 0)
			{
				std::string v;
				conf >> v;
				link_delay = v;
				std::cout << "LINK_DELAY\t\t\t" << link_delay << "\n";
			}
			else if (key.compare("L2_TEST_READ") == 0)
			{
				uint32_t v;
				conf >> v;
				l2_test_read = v;
				if (l2_test_read)
					std::cout << "L2_TEST_READ\t\t\t"
							  << "Yes"
							  << "\n";
				else
					std::cout << "L2_TEST_READ\t\t\t"
							  << "No"
							  << "\n";
			}
			else if (key.compare("APP_START_TIME") == 0)
			{
				double v;
				conf >> v;
				app_start_time = v;
				std::cout << "SINK_START_TIME\t\t\t" << app_start_time << "\n";
			}
			else if (key.compare("APP_STOP_TIME") == 0)
			{
				double v;
				conf >> v;
				app_stop_time = v;
				std::cout << "SINK_STOP_TIME\t\t\t" << app_stop_time << "\n";
			}
			else if (key.compare("SIMULATOR_STOP_TIME") == 0)
			{
				double v;
				conf >> v;
				simulator_stop_time = v;
				std::cout << "SIMULATOR_STOP_TIME\t\t" << simulator_stop_time << "\n";
			}
			else if (key.compare("SEND_IN_CHUNKS") == 0)
			{
				uint32_t v;
				conf >> v;
				send_in_chunks = v;
				if (send_in_chunks)
				{
					std::cout << "SEND_IN_CHUNKS\t\t\t"
							  << "Yes"
							  << "\n";
					std::cout << "WARNING: deprecated and not tested. Please consider using L2_WAIT_FOR_ACK";
				}
				else
					std::cout << "SEND_IN_CHUNKS\t\t\t"
							  << "No"
							  << "\n";
			}
			else if (key.compare("POISSON_DISTRIBUTION") == 0)
			{
				uint32_t v;
				conf >> v;
				poison_distr = v;
				if (poison_distr)
					std::cout << "POISSON_DISTRIBUTION\t\t"
							  << "Yes"
							  << "\n";
				else
					std::cout << "POISSON_DISTRIBUTION\t\t"
							  << "No"
							  << "\n";
			}
			else if (key.compare("QCOUNT") == 0)
			{
				uint32_t v;
				conf >> v;
				qcount = v;
				std::cout << "QCOUNT\t\t" << qcount << "\n";
				std::cout << "Note that qcount must be changed in broadcom, broadcom-egress and qbb header files\n";
			}
			else if (key.compare("NUMBER_OF_INCAST_SOURCES") == 0)
			{
				uint32_t v;
				conf >> v;
				num_sources = v;
				std::cout << "NUMBER_OF_INCAST_SOURCES\t\t" << num_sources << "\n";
			}
			else if (key.compare("FLOW_SIZE_IN_INCAST") == 0)
			{
				uint32_t v;
				conf >> v;
				packet_size_incast = v;
				std::cout << "FLOW_SIZE_IN_INCAST\t\t" << packet_size_incast << "\n";
			}
			else if (key.compare("CREATE_INCAST") == 0)
			{
				uint32_t v;
				conf >> v;
				create_incast = v;
				if (create_incast)
				{
					// flow_rate -= 32 * 10 * 1000000000.0*0.05;
					// flows_per_sec = flow_rate/(8.0*avg_flow_size);
					std::cout << "CREATE_INCAST\t\t"
							  << "Yes"
							  << "\n";
				}
				else
				{
					std::cout << "CREATE_INCAST\t\t"
							  << "No"
							  << "\n";
				}
			}
			else if (key.compare("TOPOLOGY_FILE") == 0)
            {
                std::string v;
                conf >> v;
                topology_file = v;
                std::cout << "TOPOLOGY_FILE\t\t\t" << topology_file << "\n";
            }
			else if (key.compare("FLOW_FILE") == 0)
            {
                std::string v;
                conf >> v;
                flow_file = v;
                std::cout << "FLOW_FILE\t\t\t" << flow_file << "\n";
            }
			else if (key.compare("FCT_OUTPUT_FILE") == 0)
            {
                conf >> fct_output_file;
                std::cout << "FCT_OUTPUT_FILE\t\t" << fct_output_file << '\n';
            }
			else if (key.compare("ENABLE_BUFFER_STATS") == 0)
			{
				uint32_t v;
				conf >> v;
				enable_buffer_stats = v;
				if (enable_buffer_stats)
				{
					std::cout << "ENABLE_BUFFER_STATS\t\t"
							  << "Yes"
							  << "\n";
				}
				else
				{
					std::cout << "ENABLE_BUFFER_STATS\t\t"
							  << "No"
							  << "\n";
				}
			}
			fflush(stdout);
		}
		conf.close();
	}
	else
	{
		std::cout << "Error: require a config file\n";
		fflush(stdout);
		return 1;
	}

	bool dynamicth = use_dynamic_pfc_threshold;

	NS_ASSERT(packet_level_ecmp + flow_level_ecmp < 2); //packet level ecmp and flow level ecmp are exclusive
	Config::SetDefault("ns3::Ipv4GlobalRouting::RandomEcmpRouting", BooleanValue(packet_level_ecmp));
	Config::SetDefault("ns3::Ipv4GlobalRouting::FlowEcmpRouting", BooleanValue(flow_level_ecmp));
	Config::SetDefault("ns3::QbbNetDevice::PauseTime", UintegerValue(pause_time));
	Config::SetDefault("ns3::QbbNetDevice::L2BackToZero", BooleanValue(l2_back_to_zero));
	Config::SetDefault("ns3::QbbNetDevice::L2TestRead", BooleanValue(l2_test_read));
	Config::SetDefault("ns3::QbbNetDevice::L2ChunkSize", UintegerValue(l2_chunk_size));
	Config::SetDefault("ns3::QbbNetDevice::L2AckInterval", UintegerValue(l2_ack_interval));

	Config::SetDefault("ns3::QbbNetDevice::EnableWindow", BooleanValue(enable_window));
	Config::SetDefault("ns3::QbbNetDevice::FixedWindowSize", UintegerValue(fixed_window_size));
	SeedManager::SetSeed(time(NULL));

	uint32_t node_num, switch_num, link_num, flow_num, trace_num, tcp_flow_num;

	NS_ASSERT(w_3 + w_dctcp + w_4 < 2);
	//uint8_t dst_port[node_num-switch_num] = { 0 };

	if (w_3)
	{
		std::ifstream workload;
		workload.open("mix/W3.txt");

		if (!workload)
		{
			std::cerr << "Unable to open file W3.txt";
			exit(1); // call system to stop
		}
		int size;
		double cdf = 0.0;
		//std::cout<<"New file--------\n";
		int index = 0;
		while (workload >> size)
		{
			workload >> cdf;
			//std::cout<<size<<" "<<cdf<<"\n";
			if (size > min_size)
			{
				enterprise_size.push_back(size);
				enterprise_prob.push_back(cdf);
			}
			else
			{
				enterprise_size.push_back(size);
				enterprise_prob.push_back(0.0);
			}
			index++;
		}
		workload.close();
	}
	else if (w_4)
	{
		std::ifstream workload;
		workload.open("mix/w4.txt");

		if (!workload)
		{
			std::cerr << "Unable to open file w4.txt";
			exit(1); // call system to stop
		}
		int size;
		double cdf = 0.0;
		//std::cout<<"New file--------\n";
		int index = 0;
		while (workload >> size)
		{
			workload >> cdf;
			//std::cout<<size<<" "<<cdf<<"\n";
			if (size > min_size)
			{
				enterprise_size.push_back(size);
				enterprise_prob.push_back(cdf);
			}
			else
			{
				enterprise_size.push_back(size);
				enterprise_prob.push_back(0.0);
			}
			index++;
		}
		workload.close();
	}
	else if (w_dctcp)
	{
		enterprise_size = enterprise_size_dctcp;
		enterprise_prob = enterprise_prob_dctcp;
	}
	for (int i = 0; i < enterprise_size.size(); i++)
	{
		probs.insert(enterprise_prob[i]);
		size_map[enterprise_prob[i]] = enterprise_size[i];
	}

	double avg_flow_size = 0; //in Bytes
	for (int i = 1; i < enterprise_size.size(); i++)
	{
		avg_flow_size += (enterprise_prob[i] - enterprise_prob[i - 1]) * ((enterprise_size[i] + enterprise_size[i - 1]) / 2.0);
	}

	std::cout << " Avg flow size " << avg_flow_size << "\n";

	std::cout << "Utilization Factor (including incast, if there) " << utilization_factor << "\n";
	double flow_rate = 64 * datarate * 1000000000.0 * utilization_factor; //in Bits/s
																		  //double flow_rate= 0.5 * 10 * 1000000000.0 * 0.6;//in Bits/s
	if (create_incast)
	{
		flow_rate -= 64 * datarate * 1000000000.0 * 0.05;
	}
	double flows_per_sec = flow_rate / (avg_flow_size * 8.0);

	// node_num = 144;
	// switch_num = 16;
	//std::cerr<<node_num<<" "<<switch_num<<"\n";
	// uint64_t dst_port[node_num-switch_num] = { 0 };

	// Create topology here
	topof.open(topology_file.c_str());
	topof >> node_num >> switch_num >> link_num;
	uint32_t end_num = node_num - switch_num;
	std::cout << "Node num: " << node_num << std::endl;
	std::vector<uint32_t> node_type(node_num, 0);
    for (uint32_t i = 0; i < switch_num; i++)
    {
        uint32_t sid;
        topof >> sid;
        node_type[sid] = 1;
    }

	n.Create(node_num);
	for (uint32_t i = 0; i < node_num; i++)
	{
		if (node_type[i] == 1)
		{
			n.Get(i)->SetNodeType(1, dynamicth); //broadcom switch
											   //n.Get(sid)->m_broadcom->SetMarkingThreshold(kmin, kmax, pmax);
			n.Get(i)->m_broadcom->SetNode(n.Get(i), i);
			n.Get(i)->m_broadcom->SetStats(enable_buffer_stats);
		}
	}

	NS_LOG_INFO("Create nodes.");

	InternetStackHelper internet;
	internet.Install(n);

	NS_LOG_INFO("Create channels.");

	FILE* fct_output = fopen(fct_output_file.c_str(), "w");
	//
	// Explicitly create the channels required by the topology.
	//
	Ptr<RateErrorModel> rem = CreateObject<RateErrorModel>();
	Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
	rem->SetRandomVariable(uv);
	uv->SetStream(50);
	rem->SetAttribute("ErrorRate", DoubleValue(error_rate_per_link));
	rem->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));

	QbbHelper qbb;
	Ipv4AddressHelper ipv4;

	// Create servers for all the nodes except the switches whose id is between 0 and switch_num - 1
	for (int i = 0; i < node_num - switch_num; i++)
	{
		UdpServerHelper server0(40000 + i);
		ApplicationContainer apps0s = server0.Install(n.Get(i));
		apps0s.Start(Seconds(app_start_time));
		apps0s.Stop(Seconds(simulator_stop_time));
	}
	for (uint32_t i = 0; i < link_num; i++) //Why is this 96?
	{
		uint32_t src, dst;
		double error_rate;
		topof >> src >> dst >> data_rate >> link_delay >> error_rate;

		Ptr<Node> snode = n.Get(src), dnode = n.Get(dst);

		qbb.SetDeviceAttribute("DataRate", StringValue(data_rate));
		qbb.SetChannelAttribute("Delay", StringValue(link_delay));

		if (error_rate > 0)
		{
			Ptr<RateErrorModel> rem = CreateObject<RateErrorModel>();
			Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
			rem->SetRandomVariable(uv);
			uv->SetStream(50);
			rem->SetAttribute("ErrorRate", DoubleValue(error_rate));
			rem->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
			qbb.SetDeviceAttribute("ReceiveErrorModel", PointerValue(rem));
		}
		else
		{
			qbb.SetDeviceAttribute("ReceiveErrorModel", PointerValue(rem));
		}

		fflush(stdout);
		NetDeviceContainer d = qbb.Install(snode, dnode);
		Ptr<QbbNetDevice> qbb_dev = DynamicCast<QbbNetDevice>(d.Get(0));
		qbb_dev->TraceConnectWithoutContext("QpComplete",
                                             MakeBoundCallback(qp_finish, fct_output, &ip_to_nodeid));
		qbb_dev = DynamicCast<QbbNetDevice>(d.Get(1));
		qbb_dev->TraceConnectWithoutContext("QpComplete",
                                             MakeBoundCallback(qp_finish, fct_output, &ip_to_nodeid));

		// used to create a graph of the topology
        nbr2if[snode][dnode].up = true;
        nbr2if[snode][dnode].delay =
            DynamicCast<QbbChannel>(DynamicCast<QbbNetDevice>(d.Get(0))->GetChannel())
                ->GetDelay()
                .GetTimeStep();
        nbr2if[snode][dnode].bw = DynamicCast<QbbNetDevice>(d.Get(0))->GetDataRate().GetBitRate();
        nbr2if[dnode][snode].up = true;
        nbr2if[dnode][snode].delay =
            DynamicCast<QbbChannel>(DynamicCast<QbbNetDevice>(d.Get(1))->GetChannel())
                ->GetDelay()
                .GetTimeStep();
        nbr2if[dnode][snode].bw = DynamicCast<QbbNetDevice>(d.Get(1))->GetDataRate().GetBitRate();

		char ipstring[16];
		sprintf(ipstring, "10.%d.%d.0", i / 254 + 1, i % 254 + 1);
		ipv4.SetBase(ipstring, "255.255.255.0");
		ipv4.Assign(d);
	}

	Ipv4GlobalRoutingHelper::PopulateRoutingTables();

	// update ip_to_nodeid
	for (uint32_t i = 0; i < node_num; ++i) {
		if (n.Get(i)->GetNodeType() == 0) { // host
			Ptr<Ipv4> ipv4 = n.Get(i)->GetObject<Ipv4>();
			uint32_t ip = ipv4->GetAddress(1, 0).GetLocal().Get();
			ip_to_nodeid[ip] = i;
		}
	}
	
	//
	//
    // get BDP and delay
    //
	for (int i = 0; i < (int)n.GetN(); i++)
    {
        Ptr<Node> node = n.Get(i);
        if (node->GetNodeType() == 0)
        {
            CalculateRoute(node);
        }
    }
	// print all delay and tx delay
	// for (uint32_t i = 0; i < node_num; i++)
    // {
    //     if (n.Get(i)->GetNodeType() != 0)
    //     {
    //         continue;
    //     }
    //     for (uint32_t j = 0; j < node_num; j++)
    //     {
    //         if (n.Get(j)->GetNodeType() != 0)
    //         {
    //             continue;
    //         }
    //         uint64_t delay = pairDelay[n.Get(i)][n.Get(j)];
    //         uint64_t txDelay = pairTxDelay[n.Get(i)][n.Get(j)];
    //         NS_LOG_INFO("Node " << i << " to Node " << j << " delay " << delay << " txDelay " << txDelay);
    //     }
    // }

	NS_LOG_INFO("Create Applications.");
// #ifdef false
	std::mt19937 gen(5489U);						  //Same Seed for all the Simlulations
	std::mt19937 gen2(5489U);						  //Same Seed for all the Simlulations
	std::uniform_real_distribution<> dis(0.0, 1.0);
	// maintain port number for each host
    for (uint32_t i = 0; i < node_num; i++)
    {
        if (n.Get(i)->GetNodeType() == 0)
        {
            for (uint32_t j = 0; j < node_num; j++)
            {
                if (n.Get(j)->GetNodeType() == 0)
                {
                    portNumder[i][j] = 10000; // each host pair use port number from 10000
                }
            }
        }
    }
	// read flow file
	flowf.open(flow_file.c_str());
	flowf >> flow_num;
	for (uint32_t i = 0; i < flow_num; i++)
	{
		uint32_t src_id, dst_id, pg, dport;
		double start_time;
		size_t flow_size;
		size_t flow_packet_count;
		flowf >> src_id >> dst_id >> pg >> dport >> flow_size >> start_time;
		NS_ASSERT(n.Get(src_id)->GetNodeType() == 0 && n.Get(dst_id)->GetNodeType() == 0);
		// get a new port number
        uint32_t port = portNumder[src_id][dst_id]++;
		// get server address
		Ptr<Ipv4> ipv4 = n.Get(dst_id)->GetObject<Ipv4>();
		Ipv4Address serverAddress = ipv4->GetAddress(1, 0).GetLocal(); //GetAddress(0,0) is the loopback 127.0.0.1
		// get data rate of client device
		Ptr<Node> node = n.Get(src_id); // 或 n.Get(dst_id) 或其他你关心的节点
		uint32_t nDevices = node->GetNDevices();
		Ptr<NetDevice> dev = n.Get(src_id)->GetDevice(1); // server only has 2 devices:   
		// Device 0: ns3::LoopbackNetDevice
		//Device 1: ns3::QbbNetDevice
		if (dev == nullptr){
			NS_LOG_INFO("get error at dev");
		}
		Ptr<QbbNetDevice> qbb_dev = DynamicCast<QbbNetDevice>(dev);
		// check if it is null ptr
		if (qbb_dev == nullptr){
			NS_LOG_INFO("get error at qbb_dev");
		}
		DataRate datarateobj = qbb_dev->GetDataRate();
		uint64_t datarate = qbb_dev->GetDataRate().GetBitRate();
		// get packet interval
		double interarrival = (packet_payload_size * 8) / datarate;
		Time interPacketInterval = Seconds(interarrival); //800ns assuming 
		// get flow packet count
		size_t flow_packet_size;
		if (flow_size <= packet_payload_size)
		{
			flow_packet_size = flow_size;
			flow_packet_count = 1;
		}
		else
		{
			flow_packet_size = packet_payload_size;
			flow_packet_count = int(flow_size / packet_payload_size) + 1;
		}

		pg = 5 * qcount + uint32_t(dis(gen2) * 1000000000);
		UdpClientHelper client0(serverAddress, dst_id + 40000, pg, qcount, datarate / 1e9); //Add Priority
		client0.SetAttribute("MaxPackets", UintegerValue(flow_packet_count));
		client0.SetAttribute("Interval", TimeValue(interPacketInterval));
		// NS_LOG_INFO("index " << i << " flow_packet_size " << flow_packet_size);
		// NS_ASSERT(flow_packet_size > 99);
		client0.SetAttribute("PacketSize", UintegerValue(flow_packet_size));
		ApplicationContainer apps0c = client0.Install(n.Get(src_id));
		apps0c.Start(Seconds(start_time));
		// apps0c.Stop(Seconds(simulator_stop_time));
	}
// #endif

#ifdef false
	NS_LOG_INFO("Setup Flows Info");
	uint32_t packetSize = packet_payload_size;
	//Time interPacketInterval = Seconds(0.0000005 / 2);
	double interarrival = (packetSize * 8) / (1000000000.0 * datarate);
	//std::cout<<interarrival<<"in\n";
	Time interPacketInterval = Seconds(interarrival); //800ns assuming 10Gbps link and 1000 Byte Payload
	std::mt19937 gen(5489U);						  //Same Seed for all the Simlulations
	std::mt19937 gen2(5489U);						  //Same Seed for all the Simlulations
	std::uniform_real_distribution<> dis(0.0, 1.0);
	std::exponential_distribution<double> exp_dis(flows_per_sec);
	double m = -(sigma * sigma / 2) - log(flows_per_sec);
	std::lognormal_distribution<double> lognormal_dis(m, sigma);
	double start_time = app_start_time;
	//for (uint32_t i = 0; i < flow_num; i++)
	uint32_t flownum = 0;

	uint32_t num_of_incasts = (app_stop_time - app_start_time) * 64 * datarate * 1000000000.0 * 0.05 / (packet_size_incast * packetSize * num_sources * 8.0);
	uint32_t incasts_done = 0;
	double incast_interval = (app_stop_time - app_start_time) / num_of_incasts;
	if (create_incast)
	{
		std::cout << "Incast info- "
				  << "Packet size " << packet_size_incast << "Num of sources " << num_sources << "\n";
	}

	// Create flow at here
	NS_LOG_INFO("Create Flows");
	NS_LOG_INFO("start_time " << start_time << " app_stop_time " << app_stop_time);
	while (start_time < app_stop_time)
	{
		flownum += 1;
		uint32_t src, dst, pg, maxPacketCount, port;
		src = uint32_t(dis(gen) * end_num);
		//src = 1;
		while (true)
		{
			dst = uint32_t(dis(gen) * end_num);
			if (dst != src)
				break;
		}
		/*if (flownum==1) {
 		src = 80;
 		dst = 33;
 	} else {
		dst = 33;
		src = 34 + src%14;
	}*/
		NS_ASSERT(dst < end_num);
		NS_ASSERT(src < end_num);
		//pg = 2 + uint32_t(dis(gen2) * (qcount-8));
		pg = 5 * qcount + uint32_t(dis(gen2) * 1000000000);
		//} //priority between 1 & 125, 0, 126 & 127 reserved
		double flow_size_helper = dis(gen);
		int start_ind = 0, end_ind = enterprise_size.size();
		auto higher_prob = probs.upper_bound(flow_size_helper);
		double higher_probability = *(higher_prob);
		higher_prob--;
		double lower_probability = *(higher_prob);
		int higher_size = size_map[higher_probability];
		int size = size_map[lower_probability];
		// std::cout<<"Hs "<<higher_size<<"size "<<size<<"\n";
		int flow_size = size + (higher_size - size) * ((higher_probability - flow_size_helper) / (higher_probability - lower_probability));
		// std::cout<<"Flow size "<<flow_size<<"\n";
		int flow_packet_size;
		if (flow_size <= packetSize)
		{
			flow_packet_size = flow_size;
			maxPacketCount = 1;
		}
		else
		{
			flow_packet_size = packetSize;
			maxPacketCount = int(flow_size / packetSize) + 1;
		}
		/*if (flownum==1) {
                flow_packet_size = packetSize;
                maxPacketCount=10000000;
        }*/
		if (maxPacketCount == packet_size_incast)
		{
			maxPacketCount += 1;
		}

		// std::cout << "New Flow Created Src " << src << " Dst " << dst << " Size " << maxPacketCount * (flow_packet_size) << " Time " << std::setprecision(10) << start_time << "\n";
		// NS_LOG_INFO("New Flow Created Src " << src << " Dst " << dst << " Size " << maxPacketCount * (flow_packet_size) << " Time " << std::setprecision(10) << start_time);
		//if(flownum%1000==0) std::cout<<"New Flow Created Src "<<src<<" Dst "<<dst<<" FlowNum "<<flownum<<"Packets "<<maxPacketCount<<" Priority "<<pg<<"\n";
		NS_ASSERT(n.Get(src)->GetNodeType() == 0 && n.Get(dst)->GetNodeType() == 0);
		Ptr<Ipv4> ipv4 = n.Get(dst)->GetObject<Ipv4>();
		Ipv4Address serverAddress = ipv4->GetAddress(1, 0).GetLocal(); //GetAddress(0,0) is the loopback 127.0.0.1

		// NS_LOG_INFO("Create client - server address: " << serverAddress << " port: " << dst + 40000 << " pg: " << pg << " qcount: " << qcount << " datarate: " << datarate << " maxPacketCount: " << maxPacketCount << " interPacketInterval: " << interPacketInterval << " flow_packet_size: " << flow_packet_size);
		UdpClientHelper client0(serverAddress, dst + 40000, pg, qcount, datarate); //Add Priority
		client0.SetAttribute("MaxPackets", UintegerValue(maxPacketCount));
		client0.SetAttribute("Interval", TimeValue(interPacketInterval));
		NS_ASSERT(flow_packet_size > 99);
		client0.SetAttribute("PacketSize", UintegerValue(flow_packet_size));
		ApplicationContainer apps0c = client0.Install(n.Get(src));
		apps0c.Start(Seconds(start_time));
		apps0c.Stop(Seconds(simulator_stop_time));
		if (create_incast)
		{
			if ((uint32_t)((start_time - app_start_time) / (incast_interval)) > incasts_done)
			{
				uint32_t dst2 = dst;
				while (true)
				{
					dst = uint32_t(dis(gen) * end_num);
					if (dst != dst2)
						break;
				}
				incasts_done++;
				//std::cout<<"Incast done "<<incasts_done<<" \n";
				//dst = uint32_t(dis(gen) * 64);
				for (int k = 0; k < num_sources; k++)
				{
					while (true)
					{
						src = uint32_t(dis(gen) * end_num);
						if (dst != src)
							break;
					}
					maxPacketCount = packet_size_incast;
					pg = 5 * qcount + uint32_t(dis(gen2) * 1000000000);
					std::cout << "New Flow Created Src " << src << " Dst " << dst << " Size " << maxPacketCount * (packetSize) << " Time " << std::setprecision(10) << start_time << "\n";

					NS_ASSERT(n.Get(src)->GetNodeType() == 0 && n.Get(dst)->GetNodeType() == 0);
					Ptr<Ipv4> ipv4 = n.Get(dst)->GetObject<Ipv4>();
					Ipv4Address serverAddress = ipv4->GetAddress(1, 0).GetLocal(); //GetAddress(0,0) is the loopback 127.0.0.1
					NS_ASSERT(flow_packet_size > 99);

					UdpClientHelper client0(serverAddress, dst + 40000, pg, qcount, datarate); //Add Priority
					client0.SetAttribute("MaxPackets", UintegerValue(maxPacketCount));
					client0.SetAttribute("Interval", TimeValue(interPacketInterval));
					client0.SetAttribute("PacketSize", UintegerValue(packetSize));
					ApplicationContainer apps0c = client0.Install(n.Get(src));
					apps0c.Start(Seconds(start_time));
					apps0c.Stop(Seconds(simulator_stop_time));
				}
			}
		}
		if (poison_distr)
		{
			x = exp_dis(gen);
			start_time += x;
		}
		else
		{
			x = lognormal_dis(gen);
			start_time += x;
		}
		//std::cout<<x<<"\n";
	}
	std::cout << "Number of flows " << flownum << "\n";
	if (create_incast)
	{
		std::cout << "Number of incasts done " << incasts_done << "\n";
	}
#endif

	std::cout << "Running Simulation.\n";
	fflush(stdout);
	NS_LOG_INFO("Run Simulation.");
	Simulator::Stop(Seconds(simulator_stop_time));
	Simulator::Run();
	Simulator::Destroy();
	NS_LOG_INFO("Done.");
	//fclose(trace_output);

	endt = clock();
	std::cout << (double)(endt - begint) / CLOCKS_PER_SEC << "\n";
}
