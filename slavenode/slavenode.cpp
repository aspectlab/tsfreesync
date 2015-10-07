/***********************************************************************
 * slave.cpp          
 * 
 * This source file implements a dummy "slave" node in the timestamp free
 * protocol, and activates 2 Tx and one Rx. The receive samples are discarded,
 * and no timing correction is made in this version. The primary use (until
 * further development) is to trigger and test the master node code.
 * 
 * VERSION 09.07.15:1 -- initial version by J.Canfield / M.Overdick / A.G.Klein
 * 
 **********************************************************************/

#include "sinc.hpp"
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/assign/list_of.hpp>
#include <iostream>
#include <csignal>
#include <complex>

// tweakable parameters
#define SAMPRATE 1.0e6       // sampling rate (Hz)
#define CARRIERFREQ 100.0e6  // carrier frequency (Hz)
#define CLOCKRATE 30.0e6     // clock rate (Hz) 
#define TXGAIN 60.0          // Tx frontend gain in dB
#define RXGAIN 0.0           // Rx frontend gain in dB

#define SPB 1000             // samples per buffer
#define TXDELAY 3            // buffers in the future that we schedule transmissions 
#define BW (1.0/50)          // normalized bandwidth of sinc pulse (1 --> Nyquist)
#define OFFSET (1.0/5)       // normalized freq offset of sinc pulse (1 --> Nyquist)
#define PULSE_LENGTH 8       // sinc pulse duration (in half number of lobes... in actual time units, will be 2*PULSE_LENGTH/BW)
#define PERIOD 500           // debug channel clock tick period (in number of buffers... in actual time units, will be PERIOD*SPB/SAMPRATE).
// Note: BW, PULSE_LENGTH, and SPB need to be chosen so that: 
//           + PULSE_LENGTH/BW is an integer
//           + 2*PULSE_LENGTH/BW <= SPB

typedef boost::function<uhd::sensor_value_t (const std::string&)> get_sensor_fn_t;
bool check_locked_sensor(std::vector<std::string> sensor_names, const char* sensor_name, get_sensor_fn_t get_sensor_fn, double setup_time);

/***********************************************************************
 * Signal handlers
 **********************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}

/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    // create a USRP Tx device 
    uhd::usrp::multi_usrp::sptr usrp_tx = uhd::usrp::multi_usrp::make(std::string(""));
    usrp_tx->set_master_clock_rate(CLOCKRATE);                                   // set clock rate
    usrp_tx->set_clock_source(std::string("internal"));                          // lock mboard clocks
    usrp_tx->set_tx_subdev_spec(std::string("A:A A:B"));                         // select the subdevice (2-channel mode)
    usrp_tx->set_tx_rate(SAMPRATE);                                              // set the sample rate
    uhd::tune_request_t tune_request(CARRIERFREQ);                               // validate tune request
    usrp_tx->set_tx_freq(tune_request,0);                                        // set the center frequency of chan0
    usrp_tx->set_tx_freq(tune_request,1);                                        // set the center frequency of chan1
    usrp_tx->set_tx_gain(TXGAIN,0);                                              // set the rf gain of chan0
    usrp_tx->set_tx_gain(TXGAIN,1);                                              // set the rf gain of chan1
    usrp_tx->set_tx_antenna("TX/RX",0);                                          // set the antenna of chan0
    usrp_tx->set_tx_antenna("TX/RX",1);                                          // set the antenna of chan1
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));                  // allow for some setup time

    // set USRP Rx params
    uhd::usrp::multi_usrp::sptr usrp_rx = uhd::usrp::multi_usrp::make(std::string(""));   // create a usrp device
    usrp_rx->set_master_clock_rate(CLOCKRATE);                                            // set clock rate
    usrp_rx->set_clock_source(std::string("internal"));                                   // lock mboard clocks
    usrp_rx->set_rx_subdev_spec(std::string("A:A"));                                      // select the subdevice
    usrp_rx->set_rx_rate(SAMPRATE,0);                                                     // set the sample rate
    usrp_rx->set_rx_freq(tune_request,0);                                                 // set the center frequency
    usrp_rx->set_rx_gain(RXGAIN,0);                                                       // set the rf gain
    usrp_rx->set_rx_antenna(std::string("RX2"),0);                                        // set the antenna
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));                           // allow for some setup time

    // check Ref and LO Lock detect for Tx
    std::vector<std::string> sensor_names;
    sensor_names = usrp_tx->get_tx_sensor_names(0);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp_tx->get_tx_sensor("lo_locked",0);
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    sensor_names = usrp_tx->get_mboard_sensor_names(0);

    // check Ref and LO Lock detect for Rx
    check_locked_sensor(usrp_rx->get_rx_sensor_names(0), "lo_locked", boost::bind(&uhd::usrp::multi_usrp::get_rx_sensor, usrp_rx, _1, 0), 1.0); 

    // create a transmit streamer, set time
    uhd::stream_args_t stream_args_tx("fc32", "sc16");
    stream_args_tx.channels = boost::assign::list_of(0)(1); 
    uhd::tx_streamer::sptr tx_stream = usrp_tx->get_tx_stream(stream_args_tx);
    uhd::tx_metadata_t md_tx;
    md_tx.start_of_burst = true;
    md_tx.end_of_burst = false;
    md_tx.has_time_spec = true;
    usrp_tx->set_time_unknown_pps(uhd::time_spec_t(0.0));

    // create a receive streamer
    uhd::stream_args_t stream_args_rx("fc32", "sc16");
    uhd::rx_streamer::sptr rx_stream = usrp_rx->get_rx_stream(stream_args_rx);
    uhd::rx_metadata_t md_rx;

    // create sinc, zero, and receive buffers
    std::vector< std::complex<float> > sinc(SPB);          // stores precomputed sinc pulse for Tx
    std::vector< std::complex<float> > zero(SPB, (0,0));   // stores all zeros for Tx
    std::vector< std::complex<float> *> txbuffs(2);        // pointer to facilitate 2-chan transmission
    std::vector< std::complex<float> > rxbuff(SPB);        // receive buffer

    // pre-compute the sinc waveform, and fill buffer
    const sinc_table_class sinc_table(0.3, BW, OFFSET, PULSE_LENGTH, SPB);
    for (size_t n = 0; n < SPB; n++){
        sinc[n] = sinc_table(n);
    } 
    
    // report stuff to user (things which may differ from what was requested)
    std::cout << boost::format("Actual TX Rate: %f Msps...") % (usrp_tx->get_tx_rate()/1e6) << std::endl; 
    std::cout << boost::format("Actual time between pulses: %f sec...") % (PERIOD*SPB/SAMPRATE) << std::endl << std::endl;

    // set sigint so user can terminate via Ctrl-C
    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl + C to stop streaming..." << std::endl; 

    // setup receive streaming
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = uhd::time_spec_t(0.25)+usrp_rx->get_time_now();  // tell USRP to start streaming 0.25 seconds in the future
    rx_stream->issue_stream_cmd(stream_cmd);

    // grab initial block of received samples from USRP with nice long timeout (gets discarded)
    size_t num_rx_samps = rx_stream->recv(&rxbuff.front(), SPB, md_rx, 3.0); 

    int count = 0;
    while(not stop_signal_called){
        
        // grab block of received samples from USRP
        size_t num_rx_samps = rx_stream->recv(&rxbuff.front(), SPB, md_rx); 

        // checking for receive errors
        if (md_rx.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            std::cout << boost::format("Timeout while receiving.") << std::endl;
            break;
        }
        if (md_rx.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
            std::string error = str(boost::format("Receiver error: %s") % md_rx.strerror());
            throw std::runtime_error(error);
        }

        // determine if we're transmitting zeroes or a sinc pulse, and set buffer
        if (count == PERIOD-1) {
            txbuffs[0] = &sinc.front();
            txbuffs[1] = &sinc.front();  
            count = 0;
        } else {
            txbuffs[0] = &zero.front();
            txbuffs[1] = &zero.front();  
            count++;
        }

        // send pulse
        md_tx.time_spec = md_rx.time_spec + uhd::time_spec_t((TXDELAY+1)*SPB/SAMPRATE);
        tx_stream->send(txbuffs, SPB, md_tx);
        md_tx.start_of_burst = false;

    }

    // send a mini EOB packet
    md_tx.end_of_burst = true;
    tx_stream->send("", 0, md_tx);
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));
    
    return EXIT_SUCCESS;
}

bool check_locked_sensor(std::vector<std::string> sensor_names, const char* sensor_name, get_sensor_fn_t get_sensor_fn, double setup_time){
    if (std::find(sensor_names.begin(), sensor_names.end(), sensor_name) == sensor_names.end())
        return false;

    boost::system_time start = boost::get_system_time();
    boost::system_time first_lock_time;

    std::cout << boost::format("Waiting for \"%s\": ") % sensor_name;
    std::cout.flush();

    while (true) {
        if ((not first_lock_time.is_not_a_date_time()) and
                (boost::get_system_time() > (first_lock_time + boost::posix_time::seconds(setup_time))))
        {
            std::cout << " locked." << std::endl;
            break;
        }
        if (get_sensor_fn(sensor_name).to_bool()){
            if (first_lock_time.is_not_a_date_time())
                first_lock_time = boost::get_system_time();
            std::cout << "+";
            std::cout.flush();
        }
        else {
            first_lock_time = boost::system_time();	// reset to 'not a date time'

            if (boost::get_system_time() > (start + boost::posix_time::seconds(setup_time))){
                std::cout << std::endl;
                throw std::runtime_error(str(boost::format("timed out waiting for consecutive locks on sensor \"%s\"") % sensor_name));
            }
            std::cout << "_";
            std::cout.flush();
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
    std::cout << std::endl;
    return true;
}
