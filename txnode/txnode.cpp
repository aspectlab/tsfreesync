/***********************************************************************
 * two_tx_test.cpp          
 * 
 * Sends out sinc pulses on two channels.
 *  
 * VERSION 12.10.15:1 -- initial version by A.G.Klein / M.Overdick
 * 
 **********************************************************************/

#include "includes.hpp"

    // Compliation parameters
#define DEBUG           1           // Debug (binary) if 1, debug code compiled

#define WRITESINC       1           // Write Sinc (binary) if 1, template sinc pulse
                                    // is written to file "./sinc.dat"
                            
    // tweakable parameters
#define SAMPRATE        100e3       // sampling rate (Hz)
#define CARRIERFREQ     100.0e6     // carrier frequency (Hz)
#define CLOCKRATE       30.0e6      // clock rate (Hz) 
#define TXGAIN          60.0        // Tx frontend gain in dB

#define SPB             1000        // samples per buffer
#define TXDELAY         3           // buffers in the future that we schedule transmissions (must be odd)
#define BW              (0.75)      // normalized bandwidth of sinc pulse (1 --> Nyquist)
#define CBW             (1.0)       // normalized freq offset of sinc pulse (1 --> Nyquist)
#define PULSE_LENGTH    250         // sinc pulse duration (in half number of lobes... in actual time units, will be 2*PULSE_LENGTH/BW)
#define PULSE_PERIOD    1           // ping tick period (in number of buffers... in actual time units, will be PING_PERIOD*SPB/SAMPRATE).
    // Note: BW, PULSE_LENGTH, and SPB need to be chosen so that: 
    //           + PULSE_LENGTH/BW is an integer
    //           + 2*PULSE_LENGTH/BW <= SPB
    
#define THRESHOLD       1e7         // Threshold of cross correlation

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

    /** Constant Decalartions *****************************************/

    /** Variable Declarations *****************************************/
    
        // create sinc and zero
    std::vector< CINT16 >   sinc1(SPB);                 // stores precomputed sinc pulse debug clock            (adjusted)
    std::vector< CINT16 >   sinc0(SPB);                 // stores precomputed sinc pulse ping to master         (constant)
    std::vector< CINT16 >   zero(SPB, (0,0));           // stores all zeros for Tx
    std::vector< CINT16 *>  txbuffs(2);                 // pointer to facilitate 2-chan transmission

        // Correlation variables
    std::vector<CINT32> xcorr(SPB);                 // Cross correlation
    std::vector<INT32U> normxcorr(SPB);             // Normalized cross correlation
    std::vector<INT32U>::iterator p_normxcorrmax;   // Pointer to max element
    bool threshbroken = false;                      // Threhold detection
    bool calculate    = false;                      // Calculate delay trigger
    
        // Counters
    INT16U pulse_ctr    = 0;            // Counter for transmitting pulses
    INT16U i,j,k;                       // Generic counters
    
    /** Debugging vars ************************************************/
    
    /** Variable Initializations **************************************/

    Sinc_Gen(&sinc0.front(), 2048, BW, CBW, PULSE_LENGTH, SPB, 0.0);
    Sinc_Gen(&sinc1.front(), 2048, BW, CBW, PULSE_LENGTH, SPB, 0.5);
    
    /** Debug code for writing sinc pulse *****************************/

        // Write template sinc pulse to file for debug
    #if ((DEBUG != 0) && (WRITESINC != 0))
    
        std::cout << "Writing Sinc0 to file..." << std::flush;
        writebuff_CINT16("./sinc0.dat", &sinc0.front(), SPB);
        std::cout << "done!" << std::endl;
        
        std::cout << "Writing Sinc1 to file..." << std::flush;
        writebuff_CINT16("./sinc1.dat", &sinc1.front(), SPB);
        std::cout << "done!" << std::endl;
        
    #else
    #endif /* #if ((DEBUG != 0) && (WRITESINC != 0)) */

    /** Main code *****************************************************/

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

        // check Ref and LO Lock detect for Tx
    std::vector<std::string> sensor_names;
    sensor_names = usrp_tx->get_tx_sensor_names(0);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp_tx->get_tx_sensor("lo_locked",0);
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    sensor_names = usrp_tx->get_mboard_sensor_names(0);

        // create a transmit streamer, set time
    uhd::stream_args_t stream_args_tx("sc16", "sc16");
    stream_args_tx.channels = boost::assign::list_of(0)(1);
    uhd::tx_streamer::sptr tx_stream = usrp_tx->get_tx_stream(stream_args_tx);
    uhd::tx_metadata_t md_tx;
    md_tx.start_of_burst = true;
    md_tx.end_of_burst = false;
    md_tx.has_time_spec = true;
    usrp_tx->set_time_unknown_pps(uhd::time_spec_t(0.0));

        // report stuff to user (things which may differ from what was requested)
    std::cout << boost::format("Actual TX Rate: %f Msps...") % (usrp_tx->get_tx_rate()/1e6) << std::endl; 
    std::cout << boost::format("Actual time between pulses: %f sec...") % (PULSE_PERIOD*SPB/SAMPRATE) << std::endl << std::endl;
    
        // set sigint so user can terminate via Ctrl-C
    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl + C to stop streaming..." << std::endl; 

    while(not stop_signal_called){
        /***************************************************************
         * TRANSMITTING block - Transmits debug signal and "ping"
         **************************************************************/
                // Set time spec to be one buffer ahead in time
            md_tx.time_spec = usrp_tx->get_time_now()+uhd::time_spec_t((TXDELAY+1)*(SPB)/SAMPRATE);
            
                // Debug Channel TX
            if (pulse_ctr == PULSE_PERIOD-1) {
                txbuffs[0] = &sinc0.front();  
                txbuffs[1] = &sinc1.front();  
                pulse_ctr = 0;
            } else {
                txbuffs[0] = &zero.front();  
                txbuffs[1] = &zero.front();  
                pulse_ctr++;
            }
                
                // Transmit both buffers
            tx_stream->send(txbuffs, SPB, md_tx);
            md_tx.start_of_burst = false;
            md_tx.has_time_spec = true;
            

    }   /** while(not stop_signal_called) *****************************/

        // send a mini EOB packet
    md_tx.end_of_burst = true;
    tx_stream->send("", 0, md_tx);
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));
    
    return EXIT_SUCCESS;
}   /** main() ********************************************************/

bool check_locked_sensor(std::vector<std::string> sensor_names, const char* sensor_name, get_sensor_fn_t get_sensor_fn, FP64 setup_time){
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
