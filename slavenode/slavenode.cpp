/*******************************************************************************
 * slavenode.cpp
 *
 * This source file implements the "slave" node in the timestamp free protocol.
 *
 * M.Overdick, A.G.Klein, and J.Canfield
 * Last Major Revision: 5/12/2016
 ******************************************************************************/

#include "includes.hpp"

    // Compliation parameters
#define DEBUG           1           // Debug (binary) if 1, debug code compiled

#define WRITESINC       1           // Write Sinc (binary) if 1, debug sinc pulse
                                    // is written to file "./sinc.dat"

#define DURATION        30          // Length of time to record in seconds

#define WRITEXCORR      1           // Write cross correlation to file (binary)

#define WRITERX         1           // Write receive buffer to file (binary)

#define WRITEKAL        1           // Write Kalman filter components to file

    // Radio Parameters
#define SAMPRATE        100e3       // Sampling Rate (Hz)
#define CARRIERFREQ     900.0e6     // Carrier Frequency (Hz)
#define CLOCKRATE       30.0e6      // Clock Rate (Hz)
#define TXGAIN0         50.0        // TX frontend gain, Ch 0 (dB)
#define TXGAIN1         60.0        // TX frontend gain, Ch 1 (dB)
#define RXGAIN          0.0         // RX Frontend Gain (dB)

    // Kalman Filter Gains
#define KALGAIN1        0.99999     // Gain for master clock time estimate (set to 1.0 to prevent Kalman update)
#define KALGAIN2        0.00001     // Gain for master clock rate estimate (set to 0.0 to prevent Kalman update)
// #define KALGAIN1        1.0         // Gain for master clock time estimate (set to 1.0 to prevent Kalman update)
// #define KALGAIN2        0.0         // Gain for master clock rate estimate (set to 0.0 to prevent Kalman update)
#define CLKRT           0.0         // Clockrate estimate

    // Transmission parameters
#define SPB             1000        // Samples Per Buffer SYNC_PERIOD
#define NRXBUFFS        3           // Number of Receive Buffers (circular)
#define TXDELAY         3           // Number of Buffers to Delay transmission (Must Be Odd)
#define BW              0.1         // Normalized Bandwidth of Sinc pulse (1 --> Nyquist)
#define CBW             0.5         // Normalized Freq Offset of Sinc Pulse (1 --> Nyquist)
#define SYNC_PERIOD     15          // Sync Period (# of buffers, 11 is safe minimum)


#define SINC_PRECISION  10000       // Precision of sinc pulse delays relative to SAMPRATE
                                    // Precision in seconds = 1/(SAMPRATE*SINC_PRECISION)

    // Sinc pulse amplitudes (integer)
#define XCORR_AMP       128         // Peak value of sinc pulse generated for cross correlation
#define DBSINC_AMP      0x7FFF      // Peak value of sinc pulse generated for debug channel (max 32768)
#define SYNC_AMP        0x7FFF      // Peak value of sinc pulse generated for synchronization (max 32768)

#define THRESHOLD       1e8         // Threshold of cross correlation pulse detection
#define XCORR_SHIFT     3           // How many times to divide the cross correlation by 2 before normalizing

    // Structure for handling pulse detections
typedef struct {
            INT32U center_pos;      // Position of pulse within buffer
            INT32U center;          // Maximum value detected pulse
            INT32U left;            // Value to left of max
            INT32U right;           // Value to right of max
        } MAXES;

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
    #if ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0)||(WRITEKAL != 0)))
        INT16U write_ctr = 0;
        const INT32U time = DURATION*(SAMPRATE/SPB);
    #else
    #endif /* ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0))) */

        // NUMRXBUFFS changes depending on recording thr RX buffer or not
    #if ((DEBUG != 0) && (WRITERX != 0))
        #define NUMRXBUFFS time
    #else
        #define NUMRXBUFFS NRXBUFFS
    #endif /* ((DEBUG != 0) && (WRITERX != 0)) */

    /** Debugging Variables Initialization ************************************/

    #if ((DEBUG != 0) && (WRITEKAL != 0))
            // Kalman filter variables
        std::vector< FP32 > clockoffset_rec(time);  // stores clockoffset
        std::vector< FP32 > crnt_master_rec(time);  // stores crnt_master
        std::vector< FP32 > pred_error_rec(time);   // stores pred_error

        std::vector< FP32 > time_est_rec(time);     // stores time_est
        std::vector< FP32 > rate_est_rec(time);     // stores rate_est
        std::vector< FP32 > time_pred_rec(time);    // stores time_pred
        std::vector< FP32 > rate_pred_rec(time);    // stores rate_pred

        std::vector< FP32 > interp_rec(time);    // stores interp
    #else
    #endif /* #if ((DEBUG != 0) && (WRITEKAL != 0)) */

        // Only compiled when debugging
    #if ((DEBUG != 0) && (WRITEXCORR != 0))
        std::vector<INT32U> normxcorr_write(SPB*time);  // Xcorr variable
    #else
    #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */

    /** Variable Declarations *************************************************/

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
    INT32U normxcorr_last   = 0;                    // Saves the last element in normxcorr
    bool threshbroken = false;                      // Threhold detection
    bool calculate    = false;                      // Calculate delay trigger

        // Delay estimation variables
    MAXES crnt_max, prev_max, exact_max;    // Xcorr max structures
    FP32 interp         = 0.0;              // Position of peak (in fractional samples)
    FP32 clockoffset    = 0.0;              // Calculated offset of master to slave (in samples)
    FP32 clkrt_ctr      = 0.0;              // Calculated clockrate (manually set)

        // Kalman filter variables
    FP32  crnt_master   = 0.0;          // Current time of master
    FP32  time_est      = 0.0;          // Time estimate of master
    FP32  time_pred     = 0.0;          // Time prediction of master
    FP32  rate_est      = 1.0;          // Rate estimate of master
    FP32  rate_pred     = 1.0;          // Rate prediction of master
    FP32  pred_error    = 0.0;          // Prediction error
    INT32 buff_timer    = 0;            // Timer for time between transmitting and receiving a pulse

        // Counters
    INT16U ping_ctr     = 0;            // Counter for transmitting pulses
    INT16U rxbuff_ctr   = 0;            // Counter for circular rx buffer
    INT16U i,j,k;                       // Generic counters

    /** Variable Initializations **********************************************/

        // Initialise rxbuffs (Vector of pointers)
    for(i = 0; i < NUMRXBUFFS; i++){
        rxbuffs[i] = &rxbuff.front() + SPB * i;
    }

        // Initialize oversampled sinc pulse
    Sinc_Init(BW, CBW, SPB, SINC_PRECISION);

        // Generate sinc pulse for cross correlation
    Sinc_Gen(&xcorr_sinc.front(), XCORR_AMP, SPB, 0.0);

        // Conjugate correlation sinc pulse
    for (j = 0; j < SPB; j++){
        xcorr_sinc[j] = std::conj(xcorr_sinc[j]);
    }

        // Generate synchronization and debug sinc pulses
    Sinc_Gen(&sync_sinc.front(), SYNC_AMP, SPB, 0.0);
    Sinc_Gen(&dbug_sinc.front(), DBSINC_AMP, SPB, 0.0);

    /** Debug code for writing sinc pulse *************************************/

        // Write template sinc pulse to file for debug
    #if ((DEBUG != 0) && (WRITESINC != 0))
        std::cout << "Writing debug and xcorr Sinc to file..." << std::flush;
        writebuff("./xcorr_sinc.dat", &xcorr_sinc.front(), SPB);
        writebuff("./dbug_sinc.dat", &dbug_sinc.front(), SPB);
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
    usrp_tx->set_tx_gain(TXGAIN0,0);                                             // set the rf gain of chan0
    usrp_tx->set_tx_gain(TXGAIN1,1);                                             // set the rf gain of chan1
    usrp_tx->set_tx_antenna("TX/RX",0);                                          // set the antenna of chan0
    usrp_tx->set_tx_antenna("TX/RX",1);                                          // set the antenna of chan1
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));                  // allow for some setup time

        // set USRP Rx params
    uhd::usrp::multi_usrp::sptr usrp_rx = uhd::usrp::multi_usrp::make(std::string(""));   // create a usrp device
    usrp_rx->set_master_clock_rate(CLOCKRATE);                                            // set clock rate
    usrp_rx->set_clock_source(std::string("internal"));                                   // lock mboard clocks
    usrp_rx->set_rx_subdev_spec(std::string("A:B"));                                      // select the subdevice
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
            // Increment number of frames elapsed since last pulse was received from master
        buff_timer++;

            // Remember current max values
        prev_max = crnt_max;

            // grab block of received samples from USRP
        rx_stream->recv(rxbuffs[rxbuff_ctr], SPB, md_rx);

        /** CROSS CORRELATION, FIND PEAK **************************************/
        crnt_max.center = 0;
        for (i = 0; i < SPB; i++) { // compute abs()^2 of cross-correlation at each lag i, and keep track of largest
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
            normxcorr[i] = std::norm(CINT64(xcorr.real() >> XCORR_SHIFT,xcorr.imag() >> XCORR_SHIFT));

            // Find max value of cross correlation and save its characteristics
            if(normxcorr[i] > crnt_max.center){
                crnt_max.center = normxcorr[i];          // Save crnt_max value
                crnt_max.center_pos = i;                 // Save crnt_max position
            }else{}

            /** Save buffers if enabled by defines ****************************/
                // Save normxcorr if enabled by defined variables
            #if ((DEBUG != 0) && (WRITEXCORR != 0))
                normxcorr_write[(SPB*write_ctr)+i] = normxcorr[i];
            #else
            #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */
        }

            // Find the points before and after the maximum for the interpolator
            // Check for case when crnt_max.center_pos is on rightmost boundary
        if(crnt_max.center_pos == SPB-1){
            crnt_max.right = 0;     // Sample after max get set later, so set to zero for now
        }else{
            crnt_max.right = normxcorr[crnt_max.center_pos+1];    // Save sample after max
        }

            // Check for case when crnt_max.center_pos is on leftmost boundary
        if(crnt_max.center_pos == 0){
            crnt_max.left = normxcorr_last;                  // normxcorr_last is now sample before max
        }else{
            crnt_max.left = normxcorr[crnt_max.center_pos-1];     // Save sample before max
        }
        normxcorr_last = normxcorr[SPB-1];              // Save the last element of normxcorr

            // If peak was on rightmost boundary in previous frame,
            // value to right of max is first element in current frame
        if(prev_max.center_pos == SPB-1){
            prev_max.right = normxcorr[0];              // Save sample after max
        }

            // Trigger calculation block after extra buffer
        if (threshbroken == true){
            calculate = true;
            threshbroken = false;
            // Receive and correlate one extra buffer
        }else if (crnt_max.center >= THRESHOLD){
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
        std::cout << boost::format("Ping RX Time %10.5f") % (md_rx.time_spec.get_full_secs() + md_rx.time_spec.get_frac_secs()) << std::endl;
                // Figure out which element is the actual peak of
                // the sinc and calculate its time
            if(prev_max.center > crnt_max.center){  // previous buffer had largest peak
                exact_max = prev_max;
                buff_timer--;           // decrement timer to correct it
            }else{
                exact_max = crnt_max;
            }

            // std::cout << boost::format("%i, %i, %i") % exact_max.left % exact_max.center % exact_max.right << std::endl;

            /** Delay estimator (Interpolator & Kalman filter) ****************/
                // Calculate fractional offset
            interp = ((FP32)(exact_max.left) - (FP32)(exact_max.right))/ \
                     (2*((FP32)(exact_max.left) - 2*exact_max.center + (FP32)(exact_max.right)));

            // actual roundtrip time is buff_time*SPB+(exact_max.center_pos-999)+interp.  Divided by 2, and modulo 1000, this becomes --
            if(buff_timer & 1){
                clockoffset = (exact_max.center_pos+interp+1+SPB)/2;
            }else{
                clockoffset = (exact_max.center_pos+interp+1)/2;
            }

                // Display info on the terminal
            // std::cout << boost::format("Pos %3i | Timer %4i | Interp %f") % exact_max.center_pos % buff_timer % interp << std::endl;
            // std::cout << boost::format("Offset %f") % clockoffset << std::endl;

                // Kalman Filter
            crnt_master = (clockoffset) * rate_est;
            pred_error  = crnt_master - time_pred;

                // Make sure pred_error is within +/- spb/2
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
            time_pred = time_est + (rate_est - 1) * SPB;
            rate_pred = rate_est;

            // std::cout << boost::format("R Kalman Est.: Time=%f Rate=%f Pred.: Time=%f Rate=%f ERR=%f") % time_est % rate_est % time_pred % rate_pred% pred_error << std::endl << std::endl;

                // Exit calculating mode
            calculate = false;

            // When there is no pulse received, the update KF using predicted values
        }else{
            // std::cout << boost::format("I Kalman Est.: Time=%f Rate=%f Pred.: Time=%f Rate=%f ERR=%f") % time_est % rate_est % time_pred % rate_pred% pred_error << std::endl << std::endl;
            time_est  = time_pred;
            rate_est  = rate_pred;
            time_pred = time_est + (rate_est - 1) * SPB;
        }

        /***********************************************************************
         * TRANSMITTING block - Transmits debug signal and "ping"
         **********************************************************************/
            // Set time spec to be one buffer ahead in time
        md_tx.time_spec = md_rx.time_spec + uhd::time_spec_t((TXDELAY+1)*(SPB)/SAMPRATE);

            // Sync channel TX
        if (ping_ctr >= SYNC_PERIOD-1) {

            txbuffs[0] = &sync_sinc.front();
            buff_timer = -TXDELAY;

            ping_ctr = 0;

            if(ping_ctr == 0){
                std::cout << boost::format("Ping TX Time %10.5f") % (md_tx.time_spec.get_full_secs() + md_tx.time_spec.get_frac_secs()) << std::endl;
            }else{}

        } else {
            txbuffs[0] = &zero.front();
            ping_ctr++;
        }

            // Generate appropriately delayed sinc pulse
        // clkrt_ctr = clkrt_ctr + CLKRT;   // Experimentally derived offset, produces flat segments of 20 samples
        //
        // Sinc_Gen(&dbug_sinc.front(), DBSINC_AMP, SPB, clkrt_ctr);
        // std::cout << "MORE SHIT " << (-time_est - TXDELAY * (rate_est - 1) * SPB - 0.5) << std::endl;
        Sinc_Gen(&dbug_sinc.front(), DBSINC_AMP, SPB,  -time_est - TXDELAY * (rate_est - 1) * SPB - 500.5);
        // Sinc_Gen(&dbug_sinc.front(), DBSINC_AMP, SPB,  0.0);
        // Sinc_Gen(&dbug_sinc.front(), DBSINC_AMP, SPB,  time_est + TXDELAY * (rate_est - 1) * SPB);

            // Debug Channel TX
        txbuffs[1] = &dbug_sinc.front();

            // Transmit both buffers
        tx_stream->send(txbuffs, SPB, md_tx, 0.0);
        md_tx.start_of_burst = false;

        /** Copy Kalman variables *********************************************/
        #if ((DEBUG != 0) && (WRITEKAL != 0))
            clockoffset_rec[write_ctr]  =   clockoffset;    // stores clockoffset
            crnt_master_rec[write_ctr]  =   crnt_master;    // stores crnt_master
            pred_error_rec[write_ctr]   =   pred_error;     // stores pred_error

            time_est_rec[write_ctr]     =   time_est;       // stores time_est
            rate_est_rec[write_ctr]     =   rate_est;       // stores rate_est
            time_pred_rec[write_ctr]    =   time_pred;      // stores time_pred
            rate_pred_rec[write_ctr]    =   rate_pred;      // stores rate_pred

            interp_rec[write_ctr]       = interp;           // stores interp
        #else
        #endif /* #if ((DEBUG != 0) && (WRITEKAL != 0)) */

        /** Exit program to write buffers  ************************************/
        #if ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0)||(WRITEKAL != 0)))
            if(write_ctr >= time){
                break;
            }else{
                write_ctr++;
            }
        #else
        #endif /* #if ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0)||(WRITEKAL != 0))) */
    }   /** while(not stop_signal_called) *************************************/

        // Write template sinc pulse to file for debug
    #if ((DEBUG != 0) && (WRITESINC != 0))
        std::cout << "Writing debug Sinc to file..." << std::flush;
        writebuff("./dbug_sinc_1.dat", &dbug_sinc.front(), SPB);
        std::cout << "done!" << std::endl;
    #else
    #endif /* #if ((DEBUG != 0) && (WRITESINC != 0)) */

    /** Write buffers *****************************************************/
    #if ((DEBUG != 0) && (WRITERX != 0))
        std::cout << "Writing rx buffer to file..." << std::flush;
        writebuff("./rx.dat", rxbuffs[0], SPB*time);
        std::cout << "done!" << std::endl;
    #else
    #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */

    #if ((DEBUG != 0) && (WRITEXCORR != 0))
        std::cout << "Writing normalized correlation to file..." << std::flush;
        writebuff("./xcorr.dat", &normxcorr_write.front(), SPB*time);
        std::cout << "done!" << std::endl;
    #else
    #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */

    #if ((DEBUG != 0) && (WRITEKAL != 0))
        std::cout << "Writing clockoffset to file..." << std::flush;
        writebuff("./clockoffset.dat", &clockoffset_rec.front(), time);
        std::cout << "done!" << std::endl;

        std::cout << "Writing crnt_master to file..." << std::flush;
        writebuff("./crnt_master.dat", &crnt_master_rec.front(), time);
        std::cout << "done!" << std::endl;

        std::cout << "Writing pred_error to file..." << std::flush;
        writebuff("./pred_error.dat", &pred_error_rec.front(), time);
        std::cout << "done!" << std::endl << std::endl;



        std::cout << "Writing time_est to file..." << std::flush;
        writebuff("./time_est.dat", &time_est_rec.front(), time);
        std::cout << "done!" << std::endl;

        std::cout << "Writing rate_est to file..." << std::flush;
        writebuff("./rate_est.dat", &rate_est_rec.front(), time);
        std::cout << "done!" << std::endl;

        std::cout << "Writing time_pred to file..." << std::flush;
        writebuff("./time_pred.dat", &time_pred_rec.front(), time);
        std::cout << "done!" << std::endl;

        std::cout << "Writing rate_pred to file..." << std::flush;
        writebuff("./rate_pred.dat", &rate_pred_rec.front(), time);
        std::cout << "done!" << std::endl << std::endl;



        std::cout << "Writing interp to file..." << std::flush;
        writebuff("./interp.dat", &interp_rec.front(), time);
        std::cout << "done!" << std::endl;
    #else
    #endif /* #if ((DEBUG != 0) && (WRITEKAL != 0)) */


        // send a mini EOB packet
    md_tx.end_of_burst = true;
    tx_stream->send("", 0, md_tx);
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));

    return EXIT_SUCCESS;
}   /** main() ****************************************************************/
