/***********************************************************************
 * slavenode.cpp          
 * 
 * This source file implements the "slave" node in the timestamp free
 * protocol. This can synchronize with the master within one sample of 
 * accuracy. Code is in place for fine delay estimate, but is not
 * implemented in this version
 *  
 * VERSION 09.16.15:1 -- initial version by A.G.Klein / M.Overdick
 * VERSION 10.09.15:1 -- version by A.G.Klein / M.Overdick / J.E.Canfield
 * 
 **********************************************************************/

#include "includes.hpp"

    // Compliation parameters
#define DEBUG       0       // Debug (binary) if 1, debug code compiled

#define WRITESINC   0       // Write Sinc (binary) if 1, template sinc pulse
                            // is written to file "./sinc.dat"
                            
#define WRITESIZE   500     // Length of time to record cross correlation
                            // OR receive buffer in seconds x100
                            
#define WRITEXCORR  1       // Write cross correlation to file (binary)

#define WRITERX     1       // Write receive buffer to file (binary)
                            
#define RECVEOE     0       // Exit on error for receiver function
                            // (binary) If rx_streamer->recv() returns 
                            // an error and this is set, the error will
                            // be printed on the terminal and the code
                            // will exit.
                
    // tweakable parameters
#define SAMPRATE 100e3          // sampling rate (Hz)
#define CARRIERFREQ 100.0e6     // carrier frequency (Hz)
#define CLOCKRATE 30.0e6        // clock rate (Hz) 
#define TXGAIN 60.0             // Tx frontend gain in dB
#define RXGAIN 0.0              // Rx frontend gain in dB

#define SPB 1000                // samples per buffer
#define NUMRXBUFFS 3            // number of receive buffers (circular)
#define TXDELAY 3               // buffers in the future that we schedule transmissions (must be odd)
#define BW (1.0/50)             // normalized bandwidth of sinc pulse (1 --> Nyquist)
#define CBW (1.0/5)             // normalized freq offset of sinc pulse (1 --> Nyquist)
#define PULSE_LENGTH 8          // sinc pulse duration (in half number of lobes... in actual time units, will be 2*PULSE_LENGTH/BW)
#define PERIOD 20               // debug channel clock tick period (in number of buffers... in actual time units, will be PERIOD*SPB/SAMPRATE).
    // Note: BW, PULSE_LENGTH, and SPB need to be chosen so that: 
    //           + PULSE_LENGTH/BW is an integer
    //           + 2*PULSE_LENGTH/BW <= SPB
    
#define THRESHOLD   1e8         // Threshold of cross correlation

typedef boost::function<uhd::sensor_value_t (const std::string&)> get_sensor_fn_t;

typedef struct {
            INT32U val;
            INT32U pos;
            CINT32 corr;
            uhd::time_spec_t ts;
        } MAXES;

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
        // pre-compute the sinc waveform, and fill buffer
	const sinc_table_class sinc_table(64, BW, CBW, PULSE_LENGTH, SPB);

    /** Variable Declarations *****************************************/
    
    
        // create sinc, zero, and receive buffers
    std::vector< CINT16 >   xcorr_sinc(SPB);            // stores precomputed sinc pulse for cross correlation
    std::vector< CINT16 >   sinc(SPB);                  // stores precomputed sinc pulse for Tx
    std::vector< CINT16 >   zero(SPB, (0,0));           // stores all zeros for Tx
    std::vector< CINT16 *>  txbuffs(2);                 // pointer to facilitate 2-chan transmission
    std::vector< CINT16 >   rxbuff(NUMRXBUFFS*SPB);     // (circular) receive buffer, keeps most recent 3 buffers
    std::vector< CINT16 *>  rxbuffs(NUMRXBUFFS);        // Vector of pointers to sectons of rx_buff
        // Empty rx buffer
    std::vector< CINT16 >   rxthrowaway(SPB+1);
    
    
        // Holds the number of received samples returned by rx_stream->recv()
    INT16U num_rx_samps;

        // Correlation variables
    std::vector<CINT32> xcorr(SPB);                 // Cross correlation
    std::vector<INT32U> normxcorr(SPB);             // Normalized cross correlation
    std::vector<INT32U>::iterator p_normxcorrmax;   // Pointer to max element
    bool threshbroken = false;                      // Threhold detection
    bool oddnumbuffs  = false;                      // Odd buffer counter
    bool calculate    = false;                      // Calculate delay trigger
    
        // Delay estimation variables
    MAXES max, prev_max, truemax;       // Xcorr max structures
    INT16U k0;                          // k₀ for delay estmate
    FP32 theta;                         // θ for delay estimate
    
        // Counters
    INT16U tx_ctr    = 0;               // Counter for transmitting pulses
    INT16U xcorr_ctr = NUMRXBUFFS-1;    // Counter for cross correlation
    INT16U rxbuff_ctr;                  // Counter for circular rx buffer
    INT16U i,j,k;                       // Generic counters
    
    
    /** Debugging vars ************************************************/
        // Counter for writting large buffers
    #if ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0)))
        INT16U write_ctr = 0;  
    #else
    #endif /* ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0))) */
    
        // Only compiled when debugging
    #if ((DEBUG != 0) && (WRITEXCORR != 0))
        std::vector<INT32U> normxcorr_write(SPB*WRITESIZE);  // Xcorr variable
    #else
    #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */
    
        // Only compiled when debugging
    #if ((DEBUG != 0) && (WRITERX != 0))
        std::vector<CINT16> rx_write(SPB*WRITESIZE);        // RX variable
    #else
    #endif /* #if ((DEBUG != 0) && (WRITEXRX != 0)) */
    
    /** Variable Initializations **************************************/
    
        // Initialise rxbuffs (Vector of pointers)
    for(i = 0; i < NUMRXBUFFS; i++){
        rxbuffs[i] = &rxbuff.front() + SPB * i;
    }
    
        // Initialize sinc table;
    for (j = 0; j < SPB; j++){
		xcorr_sinc[j] = sinc_table(j);          // Copy sinc_table to a vector.
        
		sinc[j] = sinc_table(j);                // Copy sinc_table to a vector.
		sinc[j].real() = sinc[j].real()*32;    // Scale sinc table
		sinc[j].imag() = sinc[j].imag()*32;    // Scale sinc table
    } 

    /** Debug code for writing sinc pulse *****************************/

        // Write template sinc pulse to file for debug
    #if ((DEBUG != 0) && (WRITESINC != 0))
    
        std::cout << "Writing Sinc to file..." << std::flush;
        writebuff_CINT16("./sinc.dat", &sinc.front(), SPB);
        std::cout << "done!" << std::endl;
        
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
        /***************************************************************
         * SEARCHING block - (cross correlating and pulse detection)
         **************************************************************/
            // Remember current max values
        prev_max = max;
        
            // Cross correlation lags RX by one buffer
        if(xcorr_ctr == NUMRXBUFFS-1){
            rxbuff_ctr = 0;
        }else{
            rxbuff_ctr = xcorr_ctr + 1;
        }
        
            // grab block of received samples from USRP
        num_rx_samps = rx_stream->recv(rxbuffs[rxbuff_ctr], SPB, md_rx); 
        oddnumbuffs = not(oddnumbuffs);
        
        /** CROSS CORRELATION *****************************************/
        for (i = 0; i < SPB; i++) { // compute abs()^2 of cross-correlation at each lag i
            xcorr[i] = 0;
                // Calculate cross correlation
            if (xcorr_ctr == NUMRXBUFFS-1){        // Correlation for cicular buffer
                    // Correlate last section of circular buffer
                for (j = 0; j < SPB-1-i; j++) {
                    xcorr[i] += rxbuffs[NUMRXBUFFS-1][i+j+1] * xcorr_sinc[j];
                }
                for (j = SPB-1-i; j < SPB; j++) {
                    xcorr[i] += rxbuffs[0][-SPB+i+j+1] * xcorr_sinc[j];
                }
            }else{  // Correlate buffers 0 to NUMRXBUFFS-2
                for (j = 0; j < SPB; j++) {
                    xcorr[i] += rxbuffs[xcorr_ctr][i+j+1] * xcorr_sinc[j];
                }
            }
                
                // Compute abs^2 of xcorr divided by 4
            normxcorr[i] = std::norm(CINT32(xcorr[i].real() >> 2,xcorr[i].imag() >> 2));
            
            
            /** Save buffers if enabled by defines ********************/
                // Save normxcorr if enabled by defined variables
            #if ((DEBUG != 0) && (WRITEXCORR != 0))
                for(k = 0; k < SPB; k++){
                    normxcorr_write[(SPB*write_ctr)+k] = normxcorr[k];
                }
            #else
            #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */
            
                // Save rx if enabled by defined variables
            #if ((DEBUG != 0) && (WRITERX != 0))
                for(k = 0; k < SPB; k++){
                    rx_write[(SPB*write_ctr)+k] = rxbuffs[xcorr_ctr][k];
                }
            #else
            #endif /* #if ((DEBUG != 0) && (WRITERX != 0)) */
        }

            // Find max value of cross correlation and save its characteristics
        p_normxcorrmax = std::max_element(normxcorr.begin(), normxcorr.end());  // find largest abs^2
        max.val  = *p_normxcorrmax;                                              // Save the largeset abs^2
        max.pos  = std::distance(normxcorr.begin(), p_normxcorrmax);             // Save the distance
        max.corr = xcorr[max.pos];
        max.ts   = md_rx.time_spec;                                               // Save the current rx time spec  
                
            // Trigger calculation block after extra buffer
        if (threshbroken == true){
            calculate = true;
            threshbroken = false;
            
            // Receive and correlate one extra buffer
        }else if (*p_normxcorrmax >= THRESHOLD){
            threshbroken = true;
        }else{}
        
            // Increment xcorr buffer counter
        xcorr_ctr++;
        if (xcorr_ctr >= NUMRXBUFFS) {
            xcorr_ctr = 0;
        }else{}
        
        
        /***************************************************************
         * CALCULATING block - Finds delay and synchronizes
         **************************************************************/
        if(calculate == true){
                // Figure out which element is the actual peak of
                // the sinc and calculate its time
            if(prev_max.val > max.val){  // previous buffer had largest peak
                truemax = prev_max;
            }else if(max.val >= prev_max.val){
                truemax = max;
            }

                // Display info on the terminal
            std::cout << boost::format("Ping RX Time %10.5f | Val %10i | Pos %3i | ") % (truemax.ts.get_full_secs() + truemax.ts.get_frac_secs()) % truemax.val % truemax.pos << std::flush;
            
            /** Coarse delay estimator **/
            
                // Do when number of buffers elapsed is odd...
            if(oddnumbuffs == true){
                std::cout << "ODD" << std::flush;
                k0 = std::ceil(float(truemax.pos)/2) + (SPB/2);
                
                // Do when number of buffers elapsed is even....
            }else{
                std::cout << "EVE" << std::flush;
                k0 = std::ceil(float(truemax.pos)/2);
            }
            
            /** Fine delay estimator **/
            
                // Calculate theta
            theta = std::atan(truemax.corr.imag()/truemax.corr.real());
            
                // Display info on the terminal
            std::cout << boost::format(" | K₀ %4i | θ %8.4f") % k0 % theta << std::endl << std::endl;
            
            /** Coarse delay compensation **/
                
                // Set TX buffers to zero
            txbuffs[0] = &zero.front();
            txbuffs[1] = &zero.front();
            
                // Only shift if necessary
            if((k0 != 0)||(k0 != 1000)){
                    // Receive k0 worth of samples to synchronize receiver
                num_rx_samps = rx_stream->recv(&rxthrowaway.front(), k0, md_rx);
                
                    // Transmit k0 worth of samples to synchronize transmitter
                md_tx.time_spec = md_rx.time_spec + uhd::time_spec_t((TXDELAY+1)*SPB/SAMPRATE);
                tx_stream->send(txbuffs, num_rx_samps, md_tx);
                md_tx.start_of_burst = false;
                md_tx.has_time_spec = true;
            }else{}
                // Exit calculating mode
            calculate = false;
        }else{}
        
        /***************************************************************
         * TRANSMITTING block - Transmits debug signal and "ping"
         **************************************************************/
                // Set time spec to be one buffer ahead in time
            md_tx.time_spec = md_rx.time_spec + uhd::time_spec_t((TXDELAY+1)*(SPB)/SAMPRATE);
                
                // Set up transmit buffers to send sinc pulses (or zeros) on TX/RX-B (chan1)
            if (tx_ctr == PERIOD-1) {
                //ta = md_tx.time_spec + uhd::time_spec_t((TXDELAY+1)*(SPB)/SAMPRATE);
                txbuffs[0] = &sinc.front();  
                txbuffs[1] = &sinc.front();  
                tx_ctr = 0;
                
                if(tx_ctr == 0){
                    std::cout << boost::format("Ping TX Time %10.5f") % (md_tx.time_spec.get_full_secs() + md_tx.time_spec.get_frac_secs()) << std::endl;
                }else{}
                
                oddnumbuffs = false;
                
            } else {
                txbuffs[0] = &zero.front();
                txbuffs[1] = &zero.front();  
                tx_ctr++;
            }
                
                // Transmit both buffers
            tx_stream->send(txbuffs, SPB, md_tx);
            md_tx.start_of_burst = false;
            md_tx.has_time_spec = true;
            
            
            /** Write buffers to file and exit program ****************/
            #if ((DEBUG != 0) && (WRITERX != 0))
                if(write_ctr >= WRITESIZE){
                    std::cout << "Writing rx buffer to file..." << std::flush;
                    writebuff_CINT16("./rx.dat", &rx_write.front(), SPB*WRITESIZE);
                    std::cout << "done!" << std::endl;
                    
                    #if (WRITEXCORR == 0)
                        break;
                    #else
                    #endif /* #if (WRITEXCORR == 0) */
                    
                }else{}
            #else
            #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */
            
            #if ((DEBUG != 0) && (WRITEXCORR != 0))
                if(write_ctr >= WRITESIZE){
                    std::cout << "Writing normalized correlation to file..." << std::flush;
                    writebuff_INT32U("./xcorr.dat", &normxcorr_write.front(), SPB*WRITESIZE);
                    std::cout << "done!" << std::endl;
                    break;
                }else{}
            #else
            #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */
        
            #if ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0)))
                write_ctr++;  
            #else
            #endif /* ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0))) */
             

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
