/*******************************************************************************
 * slavenode.cpp
 *
 * This source file implements the "slave" node in the timestamp free
 * protocol. This can synchronize with the master within one sample of
 * accuracy. Code is in place for fine delay estimate, but is not
 * implemented in this version
 *
 ******************************************************************************/

#include "includes.hpp"

    // Compliation parameters
#define DEBUG           0           // Debug (binary) if 1, debug code compiled

#define WRITESINC       1           // Write Sinc (binary) if 1, debug sinc pulse
                                    // is written to file "./sinc.dat"

#define DURATION        2           // Length of time to record in seconds

#define WRITEXCORR      1           // Write cross correlation to file (binary)

#define WRITERX         1           // Write receive buffer to file (binary)

    // Radio Parameters
#define SAMPRATE        100e3       // Sampling Rate (Hz)
#define CARRIERFREQ     900.0e6     // Carrier Frequency (Hz)
#define CLOCKRATE       30.0e6      // Clock Rate (Hz)
#define TXGAIN          60.0        // TX Frontend Gain (dB)
#define RXGAIN          0.0         // RX Frontend Gain (dB)

    // Kalman Filter Gains
#define KALGAIN1        0.9         // Gain for master clock time estimate
#define KALGAIN2        0.00009     // Gain for master clock rate estimate

    // Transmission parameters
#define SPB             1000        // Samples Per Buffer SYNC_PERIOD
#define NRXBUFFS        3           // Number of Receive Buffers (circular)
#define TXDELAY         3           // Number of Buffers to Delay transmission (Must Be Odd)
#define BW              0.75        // Normalized Bandwidth of Sinc pulse (1 --> Nyquist)
#define CBW             1.0         // Normalized Freq Offset of Sinc Pulse (1 --> Nyquist)
#define SYNC_PERIOD     20          // Sync Period (# of buffers)
#define DEBUG_PERIOD    1           // Debug Period (# of buffers)

    // Sinc pulse amplitudes (integer)
#define XCORR_AMP       64          // Peak value of sinc pulse generated for cross correlation (recommended to be 64)
#define DBSINC_AMP      30000       // Peak value of sinc pulse generated for debug channel (max 32768)
#define SYNC_AMP        30000       // Peak value of sinc pulse generated for synchronization (max 32768)

#define THRESHOLD       1e6         // Threshold of cross correlation pulse detection

typedef boost::function<uhd::sensor_value_t (const std::string&)> get_sensor_fn_t;

    // Structure for handling pulse detections
typedef struct {
            INT32U val;             // Maximum value detected pulse
            INT32U pos;             // Position of pulse within buffer
            uhd::time_spec_t ts;    // The timestamp at the time of detection
            INT32U points[3];       // Three points for interpolation
        } MAXES;

bool check_locked_sensor(std::vector<std::string> sensor_names, const char* sensor_name, get_sensor_fn_t get_sensor_fn, double setup_time);

/*******************************************************************************
 * Signal handlers
 ******************************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}

/*******************************************************************************
 * Main function
 ******************************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    /** Constant Decalartions *************************************************/
        // When recording, time specifies the number of buffers to record
    #if ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0)))
        INT16U write_ctr = 0;
        const INT32U time = DURATION*(SAMPRATE/SPB);
    #else
    #endif /* ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0))) */

    /** Variable Declarations *************************************************/
        // NUMRXBUFFS changes depending on recording thr RX buffer or not
    #if ((DEBUG != 0) && (WRITERX != 0))
        #define NUMRXBUFFS time
    #else
        #define NUMRXBUFFS NRXBUFFS
    #endif /* ((DEBUG != 0) && (WRITERX != 0)) */

        // create sinc pulses, zero, and receive buffers
    std::vector< CINT16 >   xcorr_sinc(SPB);            // Stores precomputed sinc pulse for cross correlation  (constant)
    std::vector< CINT16 >   dbug_sinc(SPB);             // Stores precomputed sinc pulse debug clock            (adjusted)
    std::vector< CINT16 >   sync_sinc(SPB);             // Stores precomputed sinc pulse ping to master         (constant)
    std::vector< CINT16 >   zero(SPB, (0,0));           // Stores all zeros for Tx
    std::vector< CINT16 *>  txbuffs(2);                 // Pointer to facilitate 2-chan transmission
    std::vector< CINT16 >   rxbuff(NUMRXBUFFS*SPB);     // Circular receive buffer, keeps most recent 3 buffers
    std::vector< CINT16 *>  rxbuffs(NUMRXBUFFS);        // Vector of pointers to sectons of rx_buff

        // Correlation variables
    CINT32 xcorr;                                   // Cross correlation
    std::vector<INT32U> normxcorr(SPB);             // Normalized cross correlation
    std::vector<INT32U>::iterator p_normxcorrmax;   // Pointer to max element
    bool threshbroken = false;                      // Threhold detection
    bool calculate    = false;                      // Calculate delay trigger

        // Delay estimation variables
    MAXES max, prev_max, truemax;       // Xcorr max structures
    FP32 interp;                        // Position of peak

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
    INT32  kal_timer    = 0;            // Timer for time between transmitting and receiving a pulse
    INT32  max_errors   = 0;            // Counts number of erroneous maximums
    bool   rx_expected  = false;        // Checks whether a returned pulse is expected
    INT32  rx_missing   = 0;            // Counts number of pulses that failed to receive
    INT32  rx_extra     = 0;            // Counts number of pulses that were not expected
    INT32  nping_sent   = 0;            // Number of pings sent


    /** Debugging vars ********************************************************/

        // Only compiled when debugging
    #if ((DEBUG != 0) && (WRITEXCORR != 0))
        std::vector<INT32U> normxcorr_write(SPB*time);  // Xcorr variable
    #else
    #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */

    /** Variable Initializations **********************************************/

        // Initialise rxbuffs (Vector of pointers)
    for(i = 0; i < NUMRXBUFFS; i++){
        rxbuffs[i] = &rxbuff.front() + SPB * i;
    }

    Sinc_Gen(&xcorr_sinc.front(),XCORR_AMP, BW, CBW, SPB, 0.0);
    for (j = 0; j < SPB; j++){
        xcorr_sinc[j] = std::conj(xcorr_sinc[j]);
    }

    Sinc_Gen(&dbug_sinc.front(), DBSINC_AMP, BW, CBW, SPB, 0.0);

    Sinc_Gen(&sync_sinc.front(), SYNC_AMP, BW, CBW, SPB, 0.0);

    /** Debug code for writing sinc pulse *************************************/

        // Write template sinc pulse to file for debug
    #if ((DEBUG != 0) && (WRITESINC != 0))
        std::cout << "Writing debug and xcorr Sinc to file..." << std::flush;
        writebuff_CINT16("./xcorr_sinc.dat", &xcorr_sinc.front(), SPB);
        writebuff_CINT16("./dbug_sinc.dat", &dbug_sinc.front(), SPB);
        std::cout << "done!" << std::endl;
    #else
    #endif /* #if ((DEBUG != 0) && (WRITESINC != 0)) */

    /** Main code *************************************************************/

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
    std::cout << boost::format("Actual time between pulses: %f sec...") % (SYNC_PERIOD*SPB/SAMPRATE) << std::endl << std::endl;

        // set sigint so user can terminate via Ctrl-C
    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

        // setup receive streaming
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = uhd::time_spec_t(0.25)+usrp_rx->get_time_now();  // tell USRP to start streaming 0.25 seconds in the future
    rx_stream->issue_stream_cmd(stream_cmd);

        // grab initial block of received samples from USRP with nice long timeout (gets discarded)
    rx_stream->recv(rxbuffs[0], SPB, md_rx, 3.0);

    while(not stop_signal_called){
        /***********************************************************************
         * SEARCHING block - (cross correlating and pulse detection)
         **********************************************************************/
            // Remember current max values
        prev_max = max;

            // grab block of received samples from USRP
        rx_stream->recv(rxbuffs[rxbuff_ctr], SPB, md_rx);

        /** CROSS CORRELATION *************************************************/
        for (i = 0; i < SPB; i++) { // compute abs()^2 of cross-correlation at each lag i
            xcorr = 0;
                // Cross correlation for circular buffer
            if(rxbuff_ctr == 0){
                for(j = 0; j < SPB-1-i; j++){
                    xcorr += rxbuffs[NUMRXBUFFS-1][i+j+1] * xcorr_sinc[j];
                }
                for(j = SPB-1-i; j < SPB; j++){
                    xcorr += rxbuffs[0][-SPB+1+i+j] * xcorr_sinc[j];
                }
            }else{
                for (j = 0; j < SPB; j++) {
                    xcorr += rxbuffs[rxbuff_ctr-1][i+j+1] * xcorr_sinc[j];
                }
            }

                // Compute abs^2 of xcorr divided by 4
            normxcorr[i] = std::norm(CINT32(xcorr.real() >> 2,xcorr.imag() >> 2));

            /** Save buffers if enabled by defines ****************************/
                // Save normxcorr if enabled by defined variables
            #if ((DEBUG != 0) && (WRITEXCORR != 0))
                normxcorr_write[(SPB*write_ctr)+i] = normxcorr[i];
            #else
            #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */
        }

            // Find max value of cross correlation and save its characteristics
        p_normxcorrmax = std::max_element(normxcorr.begin(), normxcorr.end());  // find largest abs^2
        max.val  = *p_normxcorrmax;                                             // Save the largeset abs^2
        max.pos  = std::distance(normxcorr.begin(), p_normxcorrmax);            // Save the distance
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


        /***********************************************************************
         * CALCULATING block - Finds delay and synchronizes
         **********************************************************************/
        if(calculate == true){
                // Figure out which element is the actual peak of
                // the sinc and calculate its time
            if(prev_max.val > max.val){  // previous buffer had largest peak
                truemax = prev_max;
                kal_timer -= SPB;        // Decriment timer to correct it

            }else if(max.val >= prev_max.val){
                truemax = max;
            }
            if((truemax.points[1] < truemax.points[0])||(truemax.points[1] < truemax.points[2])){
                std::cout << "TRUEMAX WAS INCORRECTLY CALCULATED!" << std::endl;
                max_errors++;
            }
            std::cout << boost::format("%i, %i, %i") % truemax.points[0] % truemax.points[1] % truemax.points[2] << std::endl;
            if (!rx_expected){
                std::cout << "Unexpected pulse received" << std::endl;
                rx_extra++;
            }else{}
            rx_expected = false;
                // Display info on the terminal
            std::cout << boost::format("Ping RX Time %10.5f | Pos %3i | Timer %4i") % (truemax.ts.get_full_secs() + truemax.ts.get_frac_secs()) % truemax.pos % kal_timer << std::flush;

            /** Delay estimator (Interpolator & Kalman filter) ****************/
                // Calculate fractional offset
            interp = -((FP32)(truemax.points[0]/2) - (FP32)(truemax.points[2]/2))/ \
                        (2*((FP32)(truemax.points[0]/2) - truemax.points[1] + (FP32)(truemax.points[2]/2)));

            std::cout << boost::format(" | Interp %f") % interp << std::endl;

                // Kalman Filter
            crnt_master = (kal_timer/SPB - (truemax.pos+interp)*0.5) * rate_est*SPB;
            pred_error  = crnt_master - time_pred;

            while(pred_error >= (SPB/2)){
                pred_error -= SPB;
            }
            while(pred_error < -(SPB/2)){
                pred_error += SPB;
            }

                // Update Estimates
            time_est  = time_pred + KALGAIN1 * pred_error;
            rate_est  = rate_pred + KALGAIN2 * pred_error;

                // Update Predictions
            time_pred = time_est + rate_est;
            rate_pred = rate_est;

            std::cout << boost::format("R Kalman Est.: Time=%f Rate=%f Pred.: Time=%f Rate=%f ERR=%f") % time_est % rate_est % time_pred % rate_pred% pred_error << std::endl << std::endl;

            /** Delay Adjustment **/
                // SPB will change with counter to be implemented
            // Sinc_Gen(&dbug_sinc.front(), 2048, BW, CBW, PULSE_LENGTH, SPB, ((truemax.pos+interp+SPB)/2));
            // Sinc_Gen(&dbug_sinc.front(), 2048, BW, CBW, SPB, ((truemax.pos+interp)/2));

                // Exit calculating mode
            calculate = false;

            // When there is no pulse received, the kalman filter updates the sinc pulse.
        }else{
            // std::cout << boost::format("I Kalman Est.: Time=%f Rate=%f Pred.: Time=%f Rate=%f ERR=%f") % time_est % rate_est % time_pred % rate_pred% pred_error << std::endl << std::endl;
            time_est  = time_pred;
            rate_est  = rate_pred;
            time_pred = time_est + rate_est;
                // Update the sinc pulse
            // Sinc_Gen(&dbug_sinc.front(), 2048, BW, CBW, PULSE_LENGTH, SPB, ((truemax.pos+interp+SPB)/2));
            // Sinc_Gen(&dbug_sinc.front(), 2048, BW, CBW, SPB, ((truemax.pos+interp)/2));
        }

        /***********************************************************************
         * TRANSMITTING block - Transmits debug signal and "ping"
         **********************************************************************/
            // Set time spec to be one buffer ahead in time
        md_tx.time_spec = md_rx.time_spec + uhd::time_spec_t((TXDELAY+1)*(SPB)/SAMPRATE);

        kal_timer += SPB;

            // Sync channel TX
        if (ping_ctr == SYNC_PERIOD-1) {

            if (rx_expected){
                std::cout << "Expected pulse not received" << std::endl << std::endl;
                rx_missing++;
            }else{}

            txbuffs[0] = &sync_sinc.front();
            ping_ctr = 0;

            if(ping_ctr == 0){
                std::cout << boost::format("Ping TX Time %10.5f") % (md_tx.time_spec.get_full_secs() + md_tx.time_spec.get_frac_secs()) << std::endl;
                nping_sent++;
                rx_expected = true;
            }else{}

            kal_timer = -TXDELAY*SPB;

        } else {
            txbuffs[0] = &zero.front();
            ping_ctr++;
        }

            // Debug Channel TX
        if (debug_ctr == DEBUG_PERIOD-1) {
            txbuffs[1] = &dbug_sinc.front();
            debug_ctr = 0;
        } else {
            txbuffs[1] = &zero.front();
            debug_ctr++;
        }

            // Transmit both buffers
        tx_stream->send(txbuffs, SPB, md_tx);
        md_tx.start_of_burst = false;
        md_tx.has_time_spec = true;

        /** Write buffers to file and exit program ****************************/
        #if ((DEBUG != 0) && (WRITERX != 0))
            if(write_ctr >= time){
                std::cout << "Writing rx buffer to file..." << std::flush;
                writebuff_CINT16("./rx.dat", rxbuffs[0], SPB*time);
                std::cout << "done!" << std::endl;

                #if (WRITEXCORR == 0)
                    break;
                #else
                #endif /* #if (WRITEXCORR == 0) */

            }else{}
        #else
        #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */

        #if ((DEBUG != 0) && (WRITEXCORR != 0))
            if(write_ctr >= time){
                std::cout << "Writing normalized correlation to file..." << std::flush;
                writebuff_INT32U("./xcorr.dat", &normxcorr_write.front(), SPB*time);
                std::cout << "done!" << std::endl;
                break;
            }else{}
        #else
        #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */

        #if ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0)))
            write_ctr++;
        #else
        #endif /* ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0))) */

    }   /** while(not stop_signal_called) *************************************/

    std::cout << "Transmission Statistics:" << std::endl;
    std::cout << "    " << max_errors << " Maximums were incorrect." << std::endl;
    std::cout << "    Total Number of Pulses Sent: " << nping_sent << std::endl;
    std::cout << "    Total Pulses Missed: " << rx_missing << " Total Extra Pulses: " << rx_extra << " Total Errors: " << (rx_missing+rx_extra) << std::endl;
    std::cout <<  boost::format("    Percent Loss: %5.2f") % (100*FP32(rx_missing)/nping_sent) << std::endl;
    std::cout <<  boost::format("    Percent Extras: %5.2f") % (100*FP32(rx_extra)/nping_sent) << std::endl;

        // send a mini EOB packet
    md_tx.end_of_burst = true;
    tx_stream->send("", 0, md_tx);
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));

    return EXIT_SUCCESS;
}   /** main() ****************************************************************/

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
