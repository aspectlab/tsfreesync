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
 * VERSION 10.13.15:1 -- M.Overdick added code for calculating fine
 *                       delay estimate, fine delay is not corrected. 
 *                       Also cleaned up code some.
 * 
 **********************************************************************/

#include "includes.hpp"

    // Compliation parameters
#define DEBUG           1           // Debug (binary) if 1, debug code compiled

#define WRITESINC       1           // Write Sinc (binary) if 1, template sinc pulse
                                    // is written to file "./sinc.dat"
                            
#define WRITESIZE       500         // Length of time to record cross correlation
                                    // OR receive buffer in seconds x100
                            
#define WRITEXCORR      0           // Write cross correlation to file (binary)

#define WRITERX         0           // Write receive buffer to file (binary)
                            
#define RECVEOE         0           // Exit on error for receiver function
                                    // (binary) If rx_streamer->recv() returns 
                                    // an error and this is set, the error will
                                    // be printed on the terminal and the code
                                    // will exit.
                
    // tweakable parameters
#define SAMPRATE        100e3       // sampling rate (Hz)
#define CARRIERFREQ     100.0e6     // carrier frequency (Hz)
#define CLOCKRATE       30.0e6      // clock rate (Hz) 
#define TXGAIN          60.0        // Tx frontend gain in dB
#define RXGAIN          0.0         // Rx frontend gain in dB

    // Kalman Filter
#define KALGAIN1        1           // Kalman Gain for master clock estimate
#define KALGAIN2        1           // Kalman Gain for master clock estimate

#define SPB             1000        // samples per buffer
#define NUMRXBUFFS      3           // number of receive buffers (circular)
#define TXDELAY         3           // buffers in the future that we schedule transmissions (must be odd)
#define BW              (0.75)      // normalized bandwidth of sinc pulse (1 --> Nyquist)
#define CBW             (1.0)       // normalized freq offset of sinc pulse (1 --> Nyquist)
#define PULSE_LENGTH    250         // sinc pulse duration (in half number of lobes... in actual time units, will be 2*PULSE_LENGTH/BW)
#define PING_PERIOD     20          // ping tick period (in number of buffers... in actual time units, will be PING_PERIOD*SPB/SAMPRATE).
#define DEBUG_PERIOD    1           // debug channel clock tick period (in number of buffers... in actual time units, will be PERIOD*SPB/SAMPRATE).
    // Note: BW, PULSE_LENGTH, and SPB need to be chosen so that: 
    //           + PULSE_LENGTH/BW is an integer
    //           + 2*PULSE_LENGTH/BW <= SPB
    
#define THRESHOLD       1e7         // Threshold of cross correlation

typedef boost::function<uhd::sensor_value_t (const std::string&)> get_sensor_fn_t;

    // Structure for pulse detections
typedef struct {
            INT32U val;
            INT32U pos;
            CINT32 corr;
            uhd::time_spec_t ts;
            INT32U points[3];
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

    /** Variable Declarations *****************************************/
    
        // create sinc, zero, and receive buffers
    std::vector< CINT16 >   xcorr_sinc(SPB);            // stores precomputed sinc pulse for cross correlation  (constant)
    std::vector< CINT16 >   sinc(SPB);                  // stores precomputed sinc pulse debug clock            (adjusted)
    std::vector< CINT16 >   sync_sinc(SPB);             // stores precomputed sinc pulse ping to master         (constant)
    std::vector< CINT16 >   zero(SPB, (0,0));           // stores all zeros for Tx
    std::vector< CINT16 *>  txbuffs(2);                 // pointer to facilitate 2-chan transmission
    std::vector< CINT16 >   rxbuff(NUMRXBUFFS*SPB);     // (circular) receive buffer, keeps most recent 3 buffers
    std::vector< CINT16 *>  rxbuffs(NUMRXBUFFS);        // Vector of pointers to sectons of rx_buff
    std::vector< CINT16 >   rxthrowaway(SPB);           // Empty rx buffer
    
        // Holds the number of received samples returned by rx_stream->recv()
    INT16U num_rx_samps;

        // Correlation variables
    std::vector<CINT32> xcorr(SPB);                 // Cross correlation
    std::vector<INT32U> normxcorr(SPB);             // Normalized cross correlation
    std::vector<INT32U>::iterator p_normxcorrmax;   // Pointer to max element
    bool threshbroken = false;                      // Threhold detection
    bool calculate    = false;                      // Calculate delay trigger
    
        // Delay estimation variables
    MAXES max, prev_max, truemax;       // Xcorr max structures
    FP32 a,b;                           // Interpolator coefficients
    FP32 pkpos;                         // Position of peak
    
        // Kalmann filter variables
    FP32 crnt_master    = 0.0;          // Current time of master
    FP32 time_est       = 0.0;          // Time estimate of master
    FP32 time_pred      = 0.0;          // Time prediction of master
    FP32 rate_est       = 1.0;          // Rate estimate of master
    FP32 rate_pred      = 1.0;          // Rate prediction of master
    FP32 pred_error     = 0.0;          // Prediction error
    
        // Counters
    INT16U ping_ctr     = 0;            // Counter for transmitting pulses
    INT16U debug_ctr    = 0;            // Counter for transmitting pulses
    INT16U rxbuff_ctr   = 0;            // Counter for circular rx buffer
    INT16U i,j,k;                       // Generic counters
    INT32  timer        = 0;            // Timer for time between transmitting and receiving a pulse
    
    
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
    
    Sinc_Gen(&xcorr_sinc.front(),64, BW, CBW, PULSE_LENGTH, SPB, 0.0);
    for (j = 0; j < SPB; j++){
        xcorr_sinc[j] = std::conj(xcorr_sinc[j]);
    }
    
    Sinc_Gen(&sinc.front(), 2048, BW, CBW, PULSE_LENGTH, SPB, 0.0);
    
    Sinc_Gen(&sync_sinc.front(), 2048, BW, CBW, PULSE_LENGTH, SPB, 0.0);

    /** Debug code for writing sinc pulse *****************************/

        // Write template sinc pulse to file for debug
    #if ((DEBUG != 0) && (WRITESINC != 0))
    
        std::cout << "Writing Sinc to file..." << std::flush;
        writebuff_CINT16("./sinc.dat", &sinc.front(), SPB);
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
    std::cout << boost::format("Actual time between pulses: %f sec...") % (PING_PERIOD*SPB/SAMPRATE) << std::endl << std::endl;
    
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
        
            // grab block of received samples from USRP
        num_rx_samps = rx_stream->recv(rxbuffs[rxbuff_ctr], SPB, md_rx); 
        
        /** CROSS CORRELATION *****************************************/
        for (i = 0; i < SPB; i++) { // compute abs()^2 of cross-correlation at each lag i
            xcorr[i] = 0;
                // Cross correlation for circular buffer
            if(rxbuff_ctr == 0){
                for(j = 0; j < SPB-1-i; j++){
                    xcorr[i] += rxbuffs[NUMRXBUFFS-1][i+j+1] * xcorr_sinc[j];
                }
                for(j = SPB-1-i; j < SPB; j++){
                    xcorr[i] += rxbuffs[0][-SPB+1+i+j] * xcorr_sinc[j];
                }
            }else{
                for (j = 0; j < SPB; j++) {
                    xcorr[i] += rxbuffs[rxbuff_ctr-1][i+j+1] * xcorr_sinc[j];
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
                    rx_write[(SPB*write_ctr)+k] = rxbuffs[rxbuff_ctr][k];
                }
            #else
            #endif /* #if ((DEBUG != 0) && (WRITERX != 0)) */
        }

            // Find max value of cross correlation and save its characteristics
        p_normxcorrmax = std::max_element(normxcorr.begin(), normxcorr.end());  // find largest abs^2
        max.val  = *p_normxcorrmax;                                             // Save the largeset abs^2
        max.pos  = std::distance(normxcorr.begin(), p_normxcorrmax);            // Save the distance
        max.corr = xcorr[max.pos];
        max.ts   = md_rx.time_spec;                                             // Save the current rx time spec  
        
            // Save maximum points for interpolation
            // When maximum is on a boundary
        if(max.pos == SPB-1){
            max.points[0] = normxcorr[max.pos-1];           // Save sample before max
            max.points[1] = normxcorr[max.pos];             // Save max 
            max.points[2] = 0;                              // Sample after max comes later
        }else if(max.pos == 0){
                // When current maximum exceeds previous maximum
            if(max.val > prev_max.val){
                max.points[0] = prev_max.points[1];         // Previous max is now sample before max
                max.points[1] = normxcorr[max.pos];         // Save max
                max.points[2] = normxcorr[max.pos+1];       // Save sample after max
                
                // When current maximum is lesser than previous
            }else{
                prev_max.points[2] = normxcorr[max.pos];    // Save sample after max
            }
            
            // When maximum is not on a boundary
        }else{
            max.points[0] = normxcorr[max.pos-1];
            max.points[1] = normxcorr[max.pos];
            max.points[2] = normxcorr[max.pos+1];
        }
        
            // Trigger calculation block after extra buffer
        if (threshbroken == true){
            calculate = true;
            threshbroken = false;
            
            // Receive and correlate one extra buffer
        }else if (*p_normxcorrmax >= THRESHOLD){
            threshbroken = true;
        }else{}
        
            // Increment rx buffer counter
        rxbuff_ctr++;
        if (rxbuff_ctr >= NUMRXBUFFS) {
            rxbuff_ctr = 0;
        }else{}
        
        
        /***************************************************************
         * CALCULATING block - Finds delay and synchronizes
         **************************************************************/
        if(calculate == true){
                // Figure out which element is the actual peak of
                // the sinc and calculate its time
            if(prev_max.val > max.val){  // previous buffer had largest peak
                truemax = prev_max;
                timer -= SPB;            // Decriment timer to correct it
                
            }else if(max.val >= prev_max.val){
                truemax = max;
            }
            
            std::cout << boost::format("%i, %i, %i") % truemax.points[0] % truemax.points[1] % truemax.points[2] << std::endl << std::endl;

                // Display info on the terminal
            std::cout << boost::format("Ping RX Time %10.5f | Val %10i | Pos %3i | Timer %4i") % (truemax.ts.get_full_secs() + truemax.ts.get_frac_secs()) % truemax.val % truemax.pos % timer << std::flush;
            
            /** Delay estimator (Interpolator & Kalman filter)**/
            
                // Calculate coefficients
            a = (truemax.points[0]/2) - truemax.points[1] + (truemax.points[2]/2);
            b = (truemax.points[0]/2) - (truemax.points[2]/2);
            
            pkpos = -b/(2*a);
            
            std::cout << boost::format(" | fine %f") % pkpos << std::endl;
            
            //pkpos = 0;  // Set fine delay to zero for debugging
            
                // Kalman Filter
            crnt_master = (timer - (truemax.pos+pkpos)*0.5) * rate_est;
            pred_error  = crnt_master - time_pred;
            
                // Update Estimates
            time_est  = time_pred + KALGAIN1 * pred_error;
            rate_est  = rate_pred + KALGAIN2 * pred_error;
            
                // Update Predictions
            time_pred = time_est + rate_est;
            rate_pred = rate_est;
            
            
            /** Delay Adjustment **/
                // SPB will change with counter to be implemented
            //Sinc_Gen(&sinc.front(), 2048, BW, CBW, PULSE_LENGTH, SPB, ((truemax.pos+pkpos+SPB)/2));
            Sinc_Gen(&sinc.front(), 2048, BW, CBW, PULSE_LENGTH, SPB, ((truemax.pos+pkpos)/2));
            
                // Exit calculating mode
            calculate = false;
        }else{
            time_est  = time_pred;
            rate_est  = rate_pred;
            time_pred = time_est + rate_est;
        }
        
        /***************************************************************
         * TRANSMITTING block - Transmits debug signal and "ping"
         **************************************************************/
                // Set time spec to be one buffer ahead in time
            md_tx.time_spec = md_rx.time_spec + uhd::time_spec_t((TXDELAY+1)*(SPB)/SAMPRATE);
        
            timer += SPB;
                
                // Ping channel TX
            if (ping_ctr == PING_PERIOD-1) {
                txbuffs[0] = &sync_sinc.front();  
                ping_ctr = 0;
                
                if(ping_ctr == 0){
                    std::cout << boost::format("Ping TX Time %10.5f") % (md_tx.time_spec.get_full_secs() + md_tx.time_spec.get_frac_secs()) << std::endl;
                }else{}
                
                timer = -TXDELAY*SPB;
                
            } else {
                txbuffs[0] = &zero.front();
                ping_ctr++;
            }
            
                // Debug Channel TX
            if (debug_ctr == DEBUG_PERIOD-1) {
                txbuffs[1] = &sinc.front();  
                debug_ctr = 0;
            } else {
                txbuffs[1] = &zero.front();  
                debug_ctr++;
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
