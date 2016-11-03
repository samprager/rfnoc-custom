/* -*- c++ -*- */
/*
 * Copyright 2016 <+YOU OR YOUR COMPANY+>.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */


#include <uhd/rfnoc/wavegen_block_ctrl.hpp>
#include <uhd/convert.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/types/ranges.hpp>
#include <uhd/types/direction.hpp>
#include <uhd/types/stream_cmd.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>
#include "time_core_3000.hpp"
#include <math.h>

using namespace uhd;
using namespace uhd::rfnoc;


class wavegen_block_ctrl_impl : public wavegen_block_ctrl
{
public:
    static const boost::uint32_t SR_CH_COUNTER_ADDR = 200;
    static const boost::uint32_t SR_CH_TUNING_COEF_ADDR = 201;
    static const boost::uint32_t SR_CH_FREQ_OFFSET_ADDR = 202;
    static const boost::uint32_t SR_AWG_CTRL_WORD_ADDR = 203;

    static const boost::uint32_t SR_PRF_INT_ADDR = 204;
    static const boost::uint32_t SR_PRF_FRAC_ADDR = 205;
    static const boost::uint32_t SR_ADC_SAMPLE_ADDR = 206;

    static const boost::uint32_t SR_RADAR_CTRL_POLICY = 207;
    static const boost::uint32_t SR_RADAR_CTRL_COMMAND = 208;
    static const boost::uint32_t SR_RADAR_CTRL_TIME_HI = 209;
    static const boost::uint32_t SR_RADAR_CTRL_TIME_LO = 210;
    static const boost::uint32_t SR_RADAR_CTRL_CLEAR_CMDS = 211;
    static const boost::uint32_t SR_AWG_RELOAD = 212;
    static const boost::uint32_t SR_AWG_RELOAD_LAST = 213;

    /* Control readback registers */

    static const boost::uint32_t RB_AWG_LEN              = 5;
    static const boost::uint32_t RB_ADC_LEN              = 6;
    static const boost::uint32_t RB_AWG_CTRL             = 7;
    static const boost::uint32_t RB_AWG_PRF              = 8;
    static const boost::uint32_t RB_AWG_POLICY           = 9;
    static const boost::uint32_t RB_AWG_STATE            = 10;

     /* Constant settings values */
    static const boost::uint32_t CTRL_WORD_SEL_CHIRP = 0x00000010;
    static const boost::uint32_t CTRL_WORD_SEL_AWG = 0x00000310;

    static const boost::uint32_t RADAR_POLICY_AUTO = 0;
    static const boost::uint32_t RADAR_POLICY_MANUAL = 1;

    /*Waveform Data Upload Header Command Identifier */
    static const boost::uint16_t WAVEFORM_WRITE_CMD = 0x5744;

    UHD_RFNOC_BLOCK_CONSTRUCTOR(wavegen_block_ctrl),
        _item_type("sc16") // We only support sc16 in this block
    {
        wfrm_header.cmd = WAVEFORM_WRITE_CMD;
        wfrm_header.id = 0;
        wfrm_header.ind = 0;
        wfrm_header.len = 0;
    }

    void set_waveform(const std::vector<boost::uint32_t> &samples)
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::set_waveform()" << std::endl;
        wfrm_header.cmd = WAVEFORM_WRITE_CMD;
        wfrm_header.ind = 0;
        wfrm_header.len = boost::uint16_t(samples.size());

        sr_write(SR_AWG_RELOAD, *((boost::uint32_t *)&wfrm_header+1));
        sr_write(SR_AWG_RELOAD, *((boost::uint32_t *)&wfrm_header));

        for (size_t i = 0; i < samples.size() - 1; i++) {
            sr_write(SR_AWG_RELOAD, samples[i]);
        }
        sr_write(SR_AWG_RELOAD_LAST, boost::uint32_t(samples.back()));

        /* Each waveform upload must have unique ID */
        wfrm_header.id ++;
    }


    void set_waveform(const std::vector<boost::uint32_t> &samples, int spp)
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::set_waveform()" << std::endl;
        wfrm_header.cmd = WAVEFORM_WRITE_CMD;
        wfrm_header.ind = 0;
        wfrm_header.len = boost::uint16_t(samples.size());

        int round_pkts = (int)floor((double)samples.size() / spp);
        int partial_spp = (int)samples.size() % spp;

        for(int j=0; j<round_pkts;j++){
            sr_write(SR_AWG_RELOAD, *((boost::uint32_t *)&wfrm_header+1));
            sr_write(SR_AWG_RELOAD, *((boost::uint32_t *)&wfrm_header));
            for (int i = 0; i < spp - 1; i++) {
                sr_write(SR_AWG_RELOAD, samples[j*spp+i]);
            }
            sr_write(SR_AWG_RELOAD_LAST, samples[j*spp+spp-1]);
            wfrm_header.ind ++;
        }
        if(partial_spp > 0){
            sr_write(SR_AWG_RELOAD, *((boost::uint32_t *)&wfrm_header+1));
            sr_write(SR_AWG_RELOAD, *((boost::uint32_t *)&wfrm_header));
            for (int i = 0; i < partial_spp - 1; i++) {
                sr_write(SR_AWG_RELOAD, samples[round_pkts*spp+i]);
            }
            sr_write(SR_AWG_RELOAD_LAST, samples.back());
        }
        /* Each waveform upload must have unique ID */
        wfrm_header.id ++;
    }
    void issue_stream_cmd(const uhd::stream_cmd_t &stream_cmd, const size_t)
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block_ctrl::issue_stream_cmd() " << char(stream_cmd.stream_mode) << std::endl;

        //setup the mode to instruction flags
        typedef boost::tuple<bool, bool, bool, bool> inst_t;
        static const uhd::dict<stream_cmd_t::stream_mode_t, inst_t> mode_to_inst = boost::assign::map_list_of
                                                                //reload, chain, samps, stop
            (stream_cmd_t::STREAM_MODE_START_CONTINUOUS,   inst_t(true,  true,  false, false))
            (stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS,    inst_t(false, false, false, true))
            (stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE, inst_t(false, false, true,  false))
            (stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE, inst_t(false, true,  true,  false))
        ;

        //setup the instruction flag values
        bool inst_reload, inst_chain, inst_samps, inst_stop;
        boost::tie(inst_reload, inst_chain, inst_samps, inst_stop) = mode_to_inst[stream_cmd.stream_mode];

        //calculate the word from flags and length
        boost::uint32_t cmd_word = 0;
        cmd_word |= boost::uint32_t((stream_cmd.stream_now)? 1 : 0) << 31;
        cmd_word |= boost::uint32_t((inst_chain)?            1 : 0) << 30;
        cmd_word |= boost::uint32_t((inst_reload)?           1 : 0) << 29;
        cmd_word |= boost::uint32_t((inst_stop)?             1 : 0) << 28;
        cmd_word |= (inst_samps)? stream_cmd.num_samps : ((inst_stop)? 0 : 1);

        //issue the stream command
        const boost::uint64_t ticks = (stream_cmd.stream_now)? 0 : stream_cmd.time_spec.to_ticks(get_rate());
        sr_write(SR_RADAR_CTRL_COMMAND, cmd_word);
        sr_write(SR_RADAR_CTRL_TIME_HI, boost::uint32_t(ticks >> 32));
        sr_write(SR_RADAR_CTRL_TIME_LO, boost::uint32_t(ticks >> 0)); //latches the command

        send_pulse();
    }

    void set_rate(double rate){
      _tick_rate = rate;
    }

    void send_pulse(){
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::send_pulse()" << std::endl;
        boost::uint32_t pulse_cmd_imm = 0x80000000;
        /* Start immediately */
        sr_write(SR_RADAR_CTRL_TIME_HI, pulse_cmd_imm);
        /* Write TIME_LO register to initiate */
        sr_write(SR_RADAR_CTRL_TIME_LO, 0);
    }
    void send_pulse(const boost::uint64_t ticks){
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::send_pulse(ticks)" << std::endl;

        // //setup the instruction flag values
        // bool inst_reload, inst_chain, inst_samps, inst_stop;
        // boost::tie(inst_reload, inst_chain, inst_samps, inst_stop) = mode_to_inst[stream_cmd.stream_mode];
        //
        // //calculate the word from flags and length
        // boost::uint32_t cmd_word = 0;
        // cmd_word |= boost::uint32_t((stream_cmd.stream_now)? 1 : 0) << 31;
        // cmd_word |= boost::uint32_t((inst_chain)?            1 : 0) << 30;
        // cmd_word |= boost::uint32_t((inst_reload)?           1 : 0) << 29;
        // cmd_word |= boost::uint32_t((inst_stop)?             1 : 0) << 28;
        // cmd_word |= (inst_samps)? stream_cmd.num_samps : ((inst_stop)? 0 : 1);
        //issue the stream command
        // const boost::uint64_t ticks = (stream_cmd.stream_now)? 0 : stream_cmd.time_spec.to_ticks(get_rate());

        // Send timed command - Let Radar Pulse Controller handle the details
        boost::uint32_t cmd_word = 0;
        //issue the stream command
        sr_write(SR_RADAR_CTRL_COMMAND, cmd_word);
        sr_write(SR_RADAR_CTRL_TIME_HI, boost::uint32_t(ticks >> 32));
        sr_write(SR_RADAR_CTRL_TIME_LO, boost::uint32_t(ticks >> 0)); //latches the command
    }

    void set_ctrl_word(boost::uint32_t ctrl_word)
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::set_ctrl_word()" << std::endl;
        sr_write(SR_AWG_CTRL_WORD_ADDR, ctrl_word);
    }

    void set_src_awg()
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::set_ctrl_word()" << std::endl;
        sr_write(SR_AWG_CTRL_WORD_ADDR, CTRL_WORD_SEL_AWG);
    }
    void set_src_chirp()
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::set_ctrl_word()" << std::endl;
        sr_write(SR_AWG_CTRL_WORD_ADDR, CTRL_WORD_SEL_CHIRP);
    }

    void set_policy(boost::uint32_t policy)
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::set_policy()" << std::endl;
        sr_write(SR_RADAR_CTRL_POLICY, policy);
    }

    void set_policy_manual()
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::set_policy_manual()" << std::endl;
        sr_write(SR_RADAR_CTRL_POLICY, RADAR_POLICY_MANUAL);
    }
    void set_policy_auto()
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::set_policy_auto()" << std::endl;
        sr_write(SR_RADAR_CTRL_POLICY, RADAR_POLICY_AUTO);
    }
    void set_num_adc_samples(boost::uint32_t n)
    {
        boost::uint32_t sample_count = n-1;
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::set_num_adc_samples()" << std::endl;
        sr_write(SR_ADC_SAMPLE_ADDR, sample_count);
    }

    void set_rx_len(boost::uint32_t rx_len)
    {
        boost::uint32_t wfrm_len = get_waveform_len();
        if (rx_len < wfrm_len) {
            throw uhd::value_error(str(
                boost::format("wavegen_block: Requested rx length %d is less than waveform length %d.\n")
                % rx_len % wfrm_len
            ));
        }
        boost::uint32_t sample_count = rx_len-wfrm_len;
        if (sample_count>0) sample_count -= 1;
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::set_rx_len()" << std::endl;
        sr_write(SR_ADC_SAMPLE_ADDR, sample_count);
    }

    void set_prf_count(boost::uint64_t prf_count)
    {
        boost::uint32_t prf_count_int = boost::uint32_t(prf_count>>32);
        boost::uint32_t prf_count_frac = boost::uint32_t(prf_count);
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::set_prf_count()" << std::endl;
        sr_write(SR_PRF_INT_ADDR, prf_count_int);
        sr_write(SR_PRF_FRAC_ADDR, prf_count_frac);
    }
    void set_chirp_counter(boost::uint32_t chirp_count)
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::set_chirp_counter()" << std::endl;
        sr_write(SR_CH_COUNTER_ADDR, chirp_count);
    }
    void set_chirp_tuning_coef(boost::uint32_t tuning_coef)
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::set_chirp_tuning_coef()" << std::endl;
        sr_write(SR_CH_TUNING_COEF_ADDR, tuning_coef);
    }
    void set_chirp_freq_offset(boost::uint32_t freq_offset)
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::set_chirp_freq_offset()" << std::endl;
        sr_write(SR_CH_FREQ_OFFSET_ADDR, freq_offset);
    }
    void setup_chirp(boost::uint32_t len, boost::uint32_t tuning_coef, boost::uint32_t freq_offset){
        set_chirp_counter(len-1);
        set_chirp_tuning_coef(tuning_coef);
        set_chirp_freq_offset(freq_offset);
    }

    void clear_commands()
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::clear_commands()" << std::endl;
        sr_write(SR_RADAR_CTRL_CLEAR_CMDS, 1);
    }

    boost::uint32_t get_ctrl_word()
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::get_ctrl_word()" << std::endl;
        boost::uint32_t ctrl_word = boost::uint32_t(user_reg_read64(RB_AWG_CTRL));
        UHD_MSG(status) << "wavegen_block::get_ctrl_word() ctrl_word ==" << ctrl_word << std::endl;
        UHD_ASSERT_THROW(ctrl_word);
        return ctrl_word;
    }
    std::string get_src()
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::get_src()" << std::endl;
        boost::uint32_t ctrl_word = boost::uint32_t(user_reg_read64(RB_AWG_CTRL));
        UHD_MSG(status) << "wavegen_block::get_ctrl_word() ctrl_word ==" << ctrl_word << std::endl;
        UHD_ASSERT_THROW(ctrl_word);
        std::string src_str;
        boost::uint32_t mask = 0x00000003;
        boost::uint32_t ctrl_src = mask & (ctrl_word>>8);
        if (mask == ctrl_src) {
            src_str = "AWG";
        }
        else if (mask == (!ctrl_src)) {
            src_str = "CHIRP";
        }
        else {
            src_str = "UNKNOWN:" + boost::lexical_cast<std::string>(ctrl_src) + " DEFAULT CHIRP";
        }
        return src_str;
    }
    boost::uint32_t get_policy_word()
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::get_policy_word()" << std::endl;
        boost::uint32_t policy = boost::uint32_t(user_reg_read64(RB_AWG_POLICY));
        UHD_MSG(status) << "wavegen_block::get_policy_word() policy ==" << policy << std::endl;
        //UHD_ASSERT_THROW(policy);
        UHD_ASSERT_THROW(1);
        return policy;
    }
    std::string get_policy()
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::get_policy()" << std::endl;
        boost::uint32_t policy = boost::uint32_t(user_reg_read64(RB_AWG_POLICY));
        UHD_MSG(status) << "wavegen_block::get_policy() policy ==" << policy << std::endl;
        //UHD_ASSERT_THROW(policy);
        std::string policy_str;
        if (policy == RADAR_POLICY_AUTO) {
            policy_str = "AUTO";
            UHD_ASSERT_THROW(1);
        }
        else if (policy == RADAR_POLICY_MANUAL) {
            policy_str = "MANUAL";
            UHD_ASSERT_THROW(1);
        }
        else {
            policy_str = "UNKNOWN:" + boost::lexical_cast<std::string>(policy) + " DEFAULT MANUAL";
        }
        return policy_str;
    }

    boost::uint32_t get_num_adc_samples()
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::get_num_adc_samples()" << std::endl;
        boost::uint32_t samples = boost::uint32_t(user_reg_read64(RB_ADC_LEN));
        UHD_MSG(status) << "wavegen_block::get_num_adc_samples() samples ==" << samples << std::endl;
        UHD_ASSERT_THROW(samples);
        return samples;
    }
    boost::uint32_t get_rx_len()
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::get_rx_len()" << std::endl;
        boost::uint32_t adc_samples = get_num_adc_samples();
        boost::uint32_t wfrm_len = get_waveform_len();
        boost::uint32_t rx_len = adc_samples+wfrm_len;
        UHD_MSG(status) << "wavegen_block::get_rx_len() rx_len ==" << rx_len << std::endl;
        UHD_ASSERT_THROW(rx_len);
        return rx_len;
    }

    boost::uint32_t get_waveform_len()
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::get_waveform_len()" << std::endl;
        boost::uint32_t len = boost::uint32_t(user_reg_read64(RB_AWG_LEN));
        UHD_MSG(status) << "wavegen_block::get_waveform_len() len ==" << len << std::endl;
        UHD_ASSERT_THROW(len);
        return len;
    }

    boost::uint64_t get_prf_count()
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::get_prf_count()" << std::endl;
        boost::uint64_t prf_count = boost::uint64_t(user_reg_read64(RB_AWG_PRF));
        UHD_MSG(status) << "wavegen_block::get_prf_count() prf_count ==" << prf_count << std::endl;
        UHD_ASSERT_THROW(prf_count);
        return prf_count;
    }
    boost::uint64_t get_state()
    {
        UHD_RFNOC_BLOCK_TRACE() << "wavegen_block::get_state()" << std::endl;
        boost::uint64_t awg_state = boost::uint64_t(user_reg_read64(RB_AWG_STATE));
        UHD_MSG(status) << "wavegen_block::get_state() awg_state ==" << awg_state << std::endl;
        UHD_ASSERT_THROW(awg_state);
        return awg_state;
    }

    double get_rate(){
      return _tick_rate;
    }

private:
    const std::string _item_type;
    double _tick_rate;
    struct waveform_header {
        boost::uint16_t len;
        boost::uint16_t ind;
        boost::uint16_t id;
        boost::uint16_t cmd;
    }wfrm_header;
};

UHD_RFNOC_BLOCK_REGISTER(wavegen_block_ctrl,"wavegen");
