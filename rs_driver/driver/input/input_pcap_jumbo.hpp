/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#include <rs_driver/driver/input/input.hpp>
#include <rs_driver/driver/input/jumbo.hpp>

#include <sstream>

#ifdef _WIN32
#ifndef WIN32
#define WIN32
#endif
#else  //__linux__
#endif

#include <pcap.h>

namespace robosense
{
namespace lidar
{
class InputPcapJumbo : public Input
{
public:
  InputPcapJumbo(const RSInputParam& input_param, double sec_to_delay)
    : Input(input_param), pcap_(NULL), pcap_offset_(ETH_HDR_LEN), pcap_tail_(0), difop_filter_valid_(false), 
    msec_to_delay_((uint64_t)(sec_to_delay / input_param.pcap_rate * 1000000))
  {
    if (input_param.use_vlan)
    {
      pcap_offset_ += VLAN_HDR_LEN;
    }

    pcap_offset_ += input_param.user_layer_bytes;
    pcap_tail_   += input_param.tail_layer_bytes;

    std::stringstream msop_stream;
    if (input_param_.use_vlan)
    {
      msop_stream << "vlan && ";
    }

    msop_stream << "udp";

    msop_filter_str_ = msop_stream.str();
  }

  virtual bool init();
  virtual bool start();
  virtual ~InputPcapJumbo();

private:
  void recvPacket();

private:
  pcap_t* pcap_;
  size_t pcap_offset_;
  size_t pcap_tail_;
  std::string msop_filter_str_;
  bpf_program msop_filter_;
  bpf_program difop_filter_;
  bool difop_filter_valid_;
  uint64_t msec_to_delay_;

  Jumbo jumbo_;
};

inline bool InputPcapJumbo::init()
{
  if (init_flag_)
    return true;

  char errbuf[PCAP_ERRBUF_SIZE];
  pcap_ = pcap_open_offline(input_param_.pcap_path.c_str(), errbuf);
  if (pcap_ == NULL)
  {
    cb_excep_(Error(ERRCODE_PCAPWRONGPATH));
    return false;
  }

  pcap_compile(pcap_, &msop_filter_, msop_filter_str_.c_str(), 1, 0xFFFFFFFF);

  init_flag_ = true;
  return true;
}

inline bool InputPcapJumbo::start()
{
  if (start_flag_)
    return true;

  if (!init_flag_)
  {
    cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
    return false;
  }

  to_exit_recv_ = false;
  recv_thread_ = std::thread(std::bind(&InputPcapJumbo::recvPacket, this));

  start_flag_ = true;
  return true;
}

inline InputPcapJumbo::~InputPcapJumbo()
{
  stop();

  if (pcap_ != NULL)
  {
    pcap_close(pcap_);
    pcap_ = NULL;
  }
}

inline void InputPcapJumbo::recvPacket()
{
  while (!to_exit_recv_)
  {
    struct pcap_pkthdr* header;
    const uint8_t* pkt_data;
    int ret = pcap_next_ex(pcap_, &header, &pkt_data);
    if (ret < 0)  // reach file end.
    {
      pcap_close(pcap_);
      pcap_ = NULL;

      if (input_param_.pcap_repeat)
      {
        cb_excep_(Error(ERRCODE_PCAPREPEAT));

        char errbuf[PCAP_ERRBUF_SIZE];
        pcap_ = pcap_open_offline(input_param_.pcap_path.c_str(), errbuf);
        continue;
      }
      else
      {
        cb_excep_(Error(ERRCODE_PCAPEXIT));
        break;
      }
    }

    if (pcap_offline_filter(&msop_filter_, header, pkt_data) != 0)
    {
      uint16_t udp_port = 0;
      const uint8_t* udp_data = NULL;
      size_t udp_data_len = 0;
      bool new_pkt = jumbo_.new_fragment(pkt_data, header->len, &udp_port, &udp_data, &udp_data_len);
      if (new_pkt)
      {
        if ((udp_port == input_param_.msop_port) || (udp_port == input_param_.difop_port))
        {
          std::shared_ptr<Buffer> pkt = cb_get_pkt_(IP_LEN);
          memcpy(pkt->data(), udp_data, udp_data_len);
          pkt->setData(0, udp_data_len);
          pushPacket(pkt);
        }
      }
    }
    else
    {
      continue;
    }

    std::this_thread::sleep_for(std::chrono::microseconds(msec_to_delay_));
  }
}

}  // namespace lidar
}  // namespace robosense
