/*
 ;    Project:       Open Vehicle Monitor System
 ;    Date:          1th October 2018
 ;
 ;    Changes:
 ;    1.0  Initial release
 ;
 ;    (C) 2018       Martin Graml
 ;    (C) 2019       Thomas Heuer
 ;
 ; Permission is hereby granted, free of charge, to any person obtaining a copy
 ; of this software and associated documentation files (the "Software"), to deal
 ; in the Software without restriction, including without limitation the rights
 ; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 ; copies of the Software, and to permit persons to whom the Software is
 ; furnished to do so, subject to the following conditions:
 ;
 ; The above copyright notice and this permission notice shall be included in
 ; all copies or substantial portions of the Software.
 ;
 ; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 ; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 ; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 ; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 ; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 ; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 ; THE SOFTWARE.
 ;
 ; Most of the CAN Messages are based on https://github.com/MyLab-odyssey/ED_BMSdiag
 */

#ifndef __VEHICLE_SMARTEQ_H__
#define __VEHICLE_SMARTEQ_H__

#include <atomic>

#include "can.h"
#include "vehicle.h"

#include "ovms_log.h"
#include "ovms_config.h"
#include "ovms_metrics.h"
#include "ovms_command.h"
#include "freertos/timers.h"
#ifdef CONFIG_OVMS_COMP_WEBSERVER
#include "ovms_webserver.h"
#endif

// CAN buffer access macros: b=byte# 0..7 / n=nibble# 0..15
#define CAN_BYTE(b)     data[b]
#define CAN_UINT(b)     (((UINT)CAN_BYTE(b) << 8) | CAN_BYTE(b+1))
#define CAN_UINT24(b)   (((uint32_t)CAN_BYTE(b) << 16) | ((UINT)CAN_BYTE(b+1) << 8) | CAN_BYTE(b+2))
#define CAN_UINT32(b)   (((uint32_t)CAN_BYTE(b) << 24) | ((uint32_t)CAN_BYTE(b+1) << 16)  | ((UINT)CAN_BYTE(b+2) << 8) | CAN_BYTE(b+3))
#define CAN_NIBL(b)     (data[b] & 0x0f)
#define CAN_NIBH(b)     (data[b] >> 4)
#define CAN_NIB(n)      (((n)&1) ? CAN_NIBL((n)>>1) : CAN_NIBH((n)>>1))

using namespace std;


class OvmsVehicleSmartEQ : public OvmsVehicle
{
  public:
    OvmsVehicleSmartEQ();
    ~OvmsVehicleSmartEQ();

  public:
    void IncomingFrameCan1(CAN_frame_t* p_frame) override;
    void IncomingPollReply(const OvmsPoller::poll_job_t &job, uint8_t* data, uint8_t length) override;
    void HandleEnergy();

  public:
    virtual vehicle_command_t CommandClimateControl(bool enable);
    virtual vehicle_command_t CommandHomelink(int button, int durationms=1000);
    virtual vehicle_command_t CommandWakeup();

  public:
#ifdef CONFIG_OVMS_COMP_WEBSERVER
    void WebInit();
    void WebDeInit();
    static void WebCfgFeatures(PageEntry_t& p, PageContext_t& c);
    static void WebCfgBattery(PageEntry_t& p, PageContext_t& c);
#endif
    void ConfigChanged(OvmsConfigParam* param) override;
    bool SetFeature(int key, const char* value);
    const std::string GetFeature(int key);
    uint64_t swap_uint64(uint64_t val);

  private:
    unsigned int m_candata_timer;
    unsigned int m_candata_poll;

  protected:
    void Ticker1(uint32_t ticker) override;
    void GetDashboardConfig(DashboardConfig& cfg);
    
    void PollReply_BMS_BattVolts(const char* reply_data, uint16_t reply_len, uint16_t start);
    void PollReply_BMS_BattTemps(const char* reply_data, uint16_t reply_len);
    void PollReply_BCB_OBC(const char* reply_data, uint16_t reply_len);
    void PollReply_HVAC(const char* reply_data, uint16_t reply_len);
    void PollReply_TDB(const char* reply_data, uint16_t reply_len);

  protected:
    bool m_enable_write;                    // canwrite

    #define DEFAULT_BATTERY_CAPACITY 17600
    #define MAX_POLL_DATA_LEN 126
    #define CELLCOUNT 96
    #define SQ_CANDATA_TIMEOUT 10

  protected:
    std::string   m_rxbuf;

  protected:
    OvmsMetricVector<float> *mt_bms_temps;       // BMS temperatures
    OvmsMetricBool          *mt_bus_awake;       // Can Bus active

  protected:
    bool m_booter_start;
};

#endif //#ifndef __VEHICLE_SMARTED_H__
