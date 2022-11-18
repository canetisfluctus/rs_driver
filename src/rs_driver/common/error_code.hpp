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

#include <string>
#include <ctime>

namespace robosense
{
namespace lidar
{
/**
 * @brief Error Code for RoboSense LiDAR Driver.
 * 0x00 For success info
 * 0x01 ~ 0x40 for Infos, some infomation during the program running
 * 0x41 ~ 0x80 for Warning, the program may not work normally
 * 0x81 ~ 0xC0 for Critical Error, the program will exit
 */
enum class ErrCodeType
{
  INFO_CODE,     ///< Common information
  WARNING_CODE,  ///< Program may not work normally
  ERROR_CODE     ///< Program will exit immediately
};

enum ErrCode
{
  // info
  ERRCODE_SUCCESS         = 0x00,  ///< Normal
  ERRCODE_PCAPREPEAT      = 0x01,  ///< Pcap file will play repeatedly
  ERRCODE_PCAPEXIT        = 0x02,  ///< Pcap thread will exit

  // warning
  ERRCODE_MSOPTIMEOUT     = 0x40,  ///< Msop packets receive overtime (1 sec)
  ERRCODE_NODIFOPRECV     = 0x41,  ///< Point cloud decoding process will not start until the difop packet receive
  ERRCODE_WRONGMSOPID     = 0x42,  ///< Packet header is wrong
  ERRCODE_WRONGMSOPLEN    = 0x43,  ///< Packet length is wrong
  ERRCODE_WRONGMSOPBLKID  = 0x44,  ///< Packet header is wrong
  ERRCODE_WRONGDIFOPID    = 0x45,  ///< Packet header is wrong
  ERRCODE_WRONGDIFOPLEN   = 0x46,  ///< Packet length is wrong
  ERRCODE_ZEROPOINTS      = 0x47,  ///< Size of the point cloud is zero
  ERRCODE_PKTBUFOVERFLOW  = 0x48,  ///< Packet buffer is overflow
  ERRCODE_CLOUDOVERFLOW   = 0x49,  ///< Point cloud buffer is overflow

  // error
  ERRCODE_STARTBEFOREINIT = 0x80,  ///< start() function is called before initializing successfully
  ERRCODE_PCAPWRONGPATH   = 0x81,  ///< Input directory of pcap file is wrong
  ERRCODE_POINTCLOUDNULL  = 0x82,  ///< PointCloud buffer is invalid
};

struct Error
{
  ErrCode error_code;
  ErrCodeType error_code_type;

  Error ()
    : error_code(ErrCode::ERRCODE_SUCCESS)
  {
  }

  explicit Error(const ErrCode& code) 
   : error_code(code)
  {
    if (error_code < 0x40)
    {
      error_code_type = ErrCodeType::INFO_CODE;
    }
    else if (error_code < 0x80)
    {
      error_code_type = ErrCodeType::WARNING_CODE;
    }
    else
    {
      error_code_type = ErrCodeType::ERROR_CODE;
    }
  }
  std::string toString() const
  {
    switch (error_code)
    {
      // info
      case ERRCODE_PCAPREPEAT:
        return "Info_PcapRepeat";
      case ERRCODE_PCAPEXIT:
        return "Info_PcapExit";

      // warning
      case ERRCODE_MSOPTIMEOUT:
        return "ERRCODE_MSOPTIMEOUT";
      case ERRCODE_NODIFOPRECV:
        return "ERRCODE_NODIFOPRECV";
      case ERRCODE_WRONGMSOPID:
        return "ERRCODE_WRONGMSOPID";
      case ERRCODE_WRONGMSOPLEN:
        return "ERRCODE_WRONGMSOPLEN";
      case ERRCODE_WRONGMSOPBLKID:
        return "ERRCODE_WRONGMSOPBLKID";
      case ERRCODE_WRONGDIFOPID:
        return "ERRCODE_WRONGDIFOPID";
      case ERRCODE_WRONGDIFOPLEN:
        return "ERRCODE_WRONGDIFOPLEN";
      case ERRCODE_ZEROPOINTS:
        return "ERRCODE_ZEROPOINTS";
      case ERRCODE_PKTBUFOVERFLOW:
        return "ERRCODE_PKTBUFOVERFLOW";
      case ERRCODE_CLOUDOVERFLOW:
        return "ERRCODE_CLOUDOVERFLOW";

      // error
      case ERRCODE_STARTBEFOREINIT:
        return "ERRCODE_STARTBEFOREINIT";
      case ERRCODE_PCAPWRONGPATH:
        return "ERRCODE_PCAPWRONGPATH";
      case ERRCODE_POINTCLOUDNULL:
        return "ERRCODE_POINTCLOUDNULL";

      //default
      default:
        return "ERRCODE_SUCCESS";
    }
  }
};

#define LIMIT_CALL(func, sec)   \
{                               \
  static time_t prev_tm = 0;    \
  time_t cur_tm = time(NULL);   \
  if ((cur_tm - prev_tm) > sec) \
  {                             \
    func;                       \
    prev_tm = cur_tm;           \
  }                             \
}

#define DELAY_LIMIT_CALL(func, sec)   \
{                                     \
  static time_t prev_tm = time(NULL); \
  time_t cur_tm = time(NULL);   \
  if ((cur_tm - prev_tm) > sec) \
  {                             \
    func;                       \
    prev_tm = cur_tm;           \
  }                             \
}

}  // namespace lidar
}  // namespace robosense
