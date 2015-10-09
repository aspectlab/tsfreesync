/***********************************************************************
 * masternode.cpp          
 * 
 * This source file implements the "master" node in the timestamp free
 * protocol. It uses a rudimentary energy detector to detect the 
 * presence of a pulse, and then plays it back in reverse.
 * 
 * 
 * VERSION 09.07.15:1 -- initial version by M.Overdick / J.Canfield / A.G.Klein
 * VERSION 10.09.15:1 -- version by M.Overdick / J.Canfield / A.G.Klein
 * 
 **********************************************************************/
 
#include "includes.hpp"

#define DEBUG       0       // Debug (binary) if 1, debug code compiled

#define WRITESIZE   2000    // Size of rx buffer to write in samples
                            // OR n seconds x100
                            
#define WRITERX     1       // Enable writing RX buffer to file

// tweakable parameters
#define SAMPRATE 100e3         // sampling rate (Hz)
#define CARRIERFREQ 100.0e6  // carrier frequency (Hz)
#define CLOCKRATE 30.0e6     // clock rate (Hz) 
#define TXGAIN 40.0          // Tx frontend gain in dB
#define RXGAIN 0.0           // Rx frontend gain in dB

#define SPB 1000             // samples per buffer
#define NUMRXBUFFS 3         // number of receive buffers (circular)
#define TXDELAY 3            // buffers in the future that we schedule transmissions (must be odd)
#define BW (1.0/50)          // normalized bandwidth of sinc pulse (1 --> Nyquist)
#define OFFSET (1.0/5)       // normalized freq offset of sinc pulse (1 --> Nyquist)
#define PULSE_LENGTH 8       // sinc pulse duration (in half number of lobes... in actual time units, will be 2*PULSE_LENGTH/BW)
#define PERIOD 500           // debug channel clock tick period (in number of buffers... in actual time units, will be PERIOD*SPB/SAMPRATE).
// Note: BW, PULSE_LENGTH, and SPB need to be chosen so that: 
//           + PULSE_LENGTH/BW is an integer
//           + 2*PULSE_LENGTH/BW <= SPB

#define DETECTION_THRESHOLD 1600 // minimum signal squared magnitude that indicates presence of sinc pulse
#define FLIP_SCALING 300    // scale factor used when re-sending flipped signals... depends heavily on choice of TXGAIN and RXGAIN

typedef boost::function<uhd::sensor_value_t (const std::string&)> get_sensor_fn_t;
bool check_locked_sensor(std::vector<std::string> sensor_names, const char* sensor_name, get_sensor_fn_t get_sensor_fn, double setup_time);

typedef enum {SEARCHING, FLIP3, FLIP2, TRANSMIT} STATES;

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
        // pre-compute the sinc waveform, and fill buffer
    const sinc_table_class sinc_table(2048, BW, OFFSET, PULSE_LENGTH, SPB);
    
    /** Variable Declarations *****************************************/
        // create sinc, zero, and receive buffers
    std::vector< CINT16 >   sinc(SPB);                  // stores precomputed sinc pulse for Tx
    std::vector< CINT16 >   zero(SPB, (0,0));           // stores all zeros for Tx
    std::vector< CINT16 >   flipbuff(3*SPB);   // stores flipped received signals for Tx
    std::vector< CINT16 *>  flipbuffs(3);      // stores flipped received signals for Tx
    std::vector< CINT16 *>  txbuffs(2);                 // pointer to facilitate 2-chan transmission
    std::vector< CINT16 >   rxbuff(NUMRXBUFFS*SPB);     // (circular) receive buffer, keeps most recent 3 buffers
    std::vector< CINT16 *>  rxbuffs(NUMRXBUFFS);        // Vector of pointers to sectons of rx_buff
    
            // Only compiled when debugging
    #if ((DEBUG != 0) && (WRITERX != 0))
        std::vector<CINT16> rx_write(SPB*WRITESIZE);  // 1s Xcorr variable
    #else
    #endif /* #if ((DEBUG != 0) && (WRITEXRX != 0)) */
    
    INT16U i = 0, j = 0, n = 0, k=0,q=0;
    INT16U count = 0, rxbuff_ctr = 0, idx;
    STATES state = SEARCHING;
    
    INT16U num_rx_samps;
    
    /** Variable Initializations **************************************/
        // Initialise rxbuffs (Vector of pointers)
    for(i = 0; i < NUMRXBUFFS; i++){
        rxbuffs[i] = &rxbuff.front() + SPB * i;
    }
    
        // Initialise rxbuffs (Vector of pointers)
    for(i = 0; i < 3; i++){
        flipbuffs[i] = &flipbuff.front() + SPB * i;
    }
    
    for (n = 0; n < SPB; n++){
        sinc[n] = sinc_table(n);
    } 
    
    /** Code **********************************************************/

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
    uhd::stream_args_t stream_args_tx("sc16", "sc16");
    stream_args_tx.channels = boost::assign::list_of(0)(1);
    uhd::tx_streamer::sptr tx_stream = usrp_tx->get_tx_stream(stream_args_tx);
    uhd::tx_metadata_t md_tx;
    md_tx.start_of_burst = true;
    md_tx.end_of_burst = false;
    md_tx.has_time_spec = true;
    usrp_tx->set_time_unknown_pps(uhd::time_spec_t(0.0));

        // create a receive streamer
    uhd::stream_args_t stream_args_rx("sc16", "sc16");
    uhd::rx_streamer::sptr rx_stream = usrp_rx->get_rx_stream(stream_args_rx);
    uhd::rx_metadata_t md_rx;

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
    num_rx_samps = rx_stream->recv(rxbuffs[0], SPB, md_rx, 3.0); 

    while(not stop_signal_called){
        
            // grab block of received samples from USRP
        num_rx_samps = rx_stream->recv(rxbuffs[rxbuff_ctr], SPB, md_rx); 
        
            // Save rxbuff if enabled by defined variables
        #if ((DEBUG != 0) && (WRITERX != 0))
            for(k = 0; k < SPB; k++){
                rx_write[(SPB*q)+k] = rxbuffs[rxbuff_ctr][k];
            }
        #else
        #endif /* #if ((DEBUG != 0) && (WRITERX != 0)) */
        
            // checking for receive errors
        if (md_rx.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            std::cout << boost::format("Timeout while receiving.") << std::endl;
            break;
        }
        if (md_rx.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
            std::string error = str(boost::format("Receiver error: %s") % md_rx.strerror());
            throw std::runtime_error(error);
        }

            // 4-state machine determines what to send on antenna TX/RX-A (chan0)... send either zeros, or flipped receive buffers
        switch (state) {
            case SEARCHING: // state 0 -- transmit zeroes, and search for pulse. if found, flip and save first (of 3) segments
                txbuffs[0] = &zero.front(); 
                for (i = 0; i < SPB; i++){ // rudimentary pulse detector
                    if (std::norm(rxbuffs[rxbuff_ctr][i]) > DETECTION_THRESHOLD) {    
                        std::cout << boost::format("Pulse detected at time: %15.8f sec") % (md_rx.time_spec.get_real_secs()) << std::endl;
                   
                        if(rxbuff_ctr - 1 == -1){
                            idx = NUMRXBUFFS-1;
                        }else{
                            idx = rxbuff_ctr - 1;
                        }
                        
                        for (j = 0; j < SPB; j++) {
                             flipbuffs[2][j].real() = rxbuffs[idx][SPB-1-j].real() * FLIP_SCALING;
                             flipbuffs[2][j].imag() = rxbuffs[idx][SPB-1-j].imag() * FLIP_SCALING;
                             
                        }
                        state = FLIP3;
                        break;
                    }
                }
                break;
            case FLIP3: // state 1 -- flip third segment, and transmit
                for (j = 0; j < SPB; j++) {
                    flipbuffs[0][j].real() = rxbuffs[rxbuff_ctr][SPB-1-j].real() * FLIP_SCALING;
                    flipbuffs[0][j].imag() = rxbuffs[rxbuff_ctr][SPB-1-j].imag() * FLIP_SCALING;
                }
                txbuffs[0] = flipbuffs[0];
                state = FLIP2;
                break;
            case FLIP2:  // state 2 -- flip second segment, and transmit
                
                if(rxbuff_ctr + 1 == 3){
                    idx = 0;
                }else{
                    idx = rxbuff_ctr + 1;
                }
                
                for (j = 0; j < SPB; j++){
                    flipbuffs[1][j].real() = rxbuffs[idx][SPB-1-j].real() * FLIP_SCALING;
                    flipbuffs[1][j].imag() = rxbuffs[idx][SPB-1-j].imag() * FLIP_SCALING;
                    
                }
                txbuffs[0] = flipbuffs[1];
                state = TRANSMIT;
                break;
            default: // state 3 -- transmit flipped first segment
                txbuffs[0] = flipbuffs[2];
                state = SEARCHING;
                break;
        }

            // set up transmit buffers to send sinc pulses (or zeros) on TX/RX-B (chan1)
        if (count == PERIOD-1) {
            txbuffs[1] = &sinc.front();  
            count = 0;
        } else {
            txbuffs[1] = &zero.front();  
            count++;
        }

            // transmit both buffers
        md_tx.time_spec = md_rx.time_spec + uhd::time_spec_t((TXDELAY+1)*SPB/SAMPRATE);
        tx_stream->send(txbuffs, SPB, md_tx);
        md_tx.start_of_burst = false;

            // increment circular receive buffer counter
        rxbuff_ctr++;
        if (rxbuff_ctr >= NUMRXBUFFS) {
            rxbuff_ctr = 0;
        }else{}

        #if ((DEBUG != 0) && (WRITERX != 0))
            if(q >= WRITESIZE){
                std::cout << "Writing rx buffer to file..." << std::flush;
                writebuff_CINT16("./rx.dat", &rx_write.front(), SPB*WRITESIZE);
                std::cout << "done!" << std::endl;
                break;
            }else{}
        #else
        #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */


        q++;
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
