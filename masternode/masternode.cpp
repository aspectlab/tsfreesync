/*******************************************************************************
 * masternode.cpp
 *
 * This source file implements the "master" node in the timestamp free protocol.
 *
 * M.Overdick, A.G.Klein, and J.Canfield
 * Last Major Revision: 8/5/2016
 ******************************************************************************/

#include "includes.hpp"

    // Compilation Parameters
#define DEBUG           0           // Debug (binary) if 1, debug code compiled

#define DURATION        100         // Duration of recording (s)

#define WRITESINC       1           // Write template sinc pulses (binary)

#define WRITERX         1           // Enable writing RX buffer to file (binary)

#define WRITEXCORR      1           // Write normalized cross correlation to file (binary)

    // Radio Parameters
#define SAMPRATE        150e3       // Sampling rate (Hz)
#define CARRIERFREQ     900.0e6     // Carrier frequency (Hz)
#define CLOCKRATE       30.0e6      // Clock rate (Hz)
#define TXGAIN0         56.5        // TX frontend gain, Ch 0 (dB)
#define TXGAIN1         60.0        // TX frontend gain, Ch 1 (dB)
#define RXGAIN          0.0         // RX frontend gain (dB)

    // Transmission Parameters
#define SPB             1000        // Samples Per Buffer
#define NRXBUFFS        3           // Number of receive buffers (circular)
#define TXDELAY         2           // Buffers in the future that we schedule transmissions (must be even)
#define BW              0.4         // Normalized bandwidth of sinc pulse (1 --> Nyquist)
#define CBW             0.5         // Normalized freq offset of sinc pulse (1 --> Nyquist)
#define DEBUG_PERIOD    1           // Debug Period (# of buffers)

#define SINC_PRECISION  10000       // Precision of sinc pulse delays relative to FS
                                    // Precision in seconds = 1/(SAMPRATE*SINC_PRECISION)

    // Sinc pulse amplitudes (integer)
#define XCORR_AMP       64          // Peak value of sinc pulse generated for cross correlation (recommended to be 64)
#define DBSINC_AMP      30000       // Peak value of sinc pulse generated for debug channel (max 32768)

#define THRESHOLD       5e6         // Threshold of cross correlation pulse detection
#define FLIP_SCALING    50          // scale factor used when re-sending flipped signals... depends heavily on choice of TXGAIN and RXGAIN

typedef enum {SEARCHING, FLIP2, FLIP1, FLIP0, TRANSMIT} STATES;

/*******************************************************************************
* Signal handlers
*******************************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}

/*******************************************************************************
* Main function
*******************************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    /** Variable Declarations *************************************************/
        // Counter for writting large buffers
    #if ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0)))
        INT16U write_ctr = 0;
        const INT32U time = DURATION*(SAMPRATE/SPB);
        // Configure number of RX buffs for recording or not recording
    #else
    #endif /* ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0))) */

    #if ((DEBUG != 0) && (WRITERX != 0))
        #define NUMRXBUFFS time
    #else
        #define NUMRXBUFFS NRXBUFFS
    #endif /* ((DEBUG != 0) && (WRITERX != 0)) */

        // create sinc, zero, and receive buffers
    std::vector< INT32U >   normxcorr(SPB);             // Normalized cross correlation
    std::vector< CINT16 >   xcorr_sinc(SPB);            // stores precomputed sinc pulse for cross correlation
    std::vector< CINT16 >   dbug_sinc(SPB);             // stores precomputed sinc pulse for Tx
    std::vector< CINT16 >   zero(SPB, (0,0));           // stores all zeros for Tx
    std::vector< CINT16 >   flipbuff(3*SPB);            // stores flipped received signals for Tx
    std::vector< CINT16 *>  flipbuffs(3);               // stores flipped received signals for Tx
    std::vector< CINT16 *>  txbuffs(2);                 // pointer to facilitate 2-chan transmission
    std::vector< CINT16 >   rxbuff(NUMRXBUFFS*SPB);     // (circular) receive buffer, keeps most recent 3 buffers
    std::vector< CINT16 *>  rxbuffs(NUMRXBUFFS);        // Vector of pointers to sectons of rx_buff

        // Only compiled when debugging
    #if ((DEBUG != 0) && (WRITEXCORR != 0))
        std::vector<INT32U> normxcorr_write(SPB*time);  // Xcorr variable
    #else
    #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */

    INT16U i = 0, j = 0;                            // Unsigned general counters
    INT16U count = 0, rxbuff_ctr = 0 , idx = 0;     // Specific purpose counters
    CINT32 xcorr = 0;                               // Cross correlation variable
    STATES state = SEARCHING;                       // State memory
    INT16U flip_ctr = 0;                            // Counter for transmitting flipped buffers

    /** Variable Initializations **********************************************/
        // Initialise rxbuffs (Vector of pointers)
    for(i = 0; i < NUMRXBUFFS; i++){
        rxbuffs[i] = &rxbuff.front() + SPB * i;
    }

        // Initialise rxbuffs (Vector of pointers)
    for(i = 0; i < 3; i++){
        flipbuffs[i] = &flipbuff.front() + SPB * i;
    }
        // Initialize oversampled sinc pulse
    Sinc_Init(BW, CBW, SPB, SINC_PRECISION);



        // Generate sinc pulse for cross correlation
    Sinc_Gen(&xcorr_sinc.front(), XCORR_AMP, SPB, 0.0);

        // Conjugate correlation sinc pulse
    for (j = 0; j < SPB; j++){
        xcorr_sinc[j] = std::conj(xcorr_sinc[j]);
    }

        // Generate debug sinc pulse
    Sinc_Gen(&dbug_sinc.front(), DBSINC_AMP, SPB, 0.0);

    /** Debug code for writing sinc pulse *************************************/

        // Write template sinc pulse to file for debug
    #if ((DEBUG != 0) && (WRITESINC != 0))
        std::cout << "Writing Sinc to file..." << std::flush;
        writebuff("./xcorr_sinc.dat", &xcorr_sinc.front(), SPB);
        writebuff("./dbug_sinc.dat", &dbug_sinc.front(), SPB);
        std::cout << "done!" << std::endl;
    #else
    #endif /* #if ((DEBUG != 0) && (WRITESINC != 0)) */

    /** Code ******************************************************************/

        // create a USRP Tx device
    uhd::usrp::multi_usrp::sptr usrp_tx = uhd::usrp::multi_usrp::make(std::string(""));
    usrp_tx->set_master_clock_rate(CLOCKRATE);                                   // set clock rate
    usrp_tx->set_clock_source(std::string("internal"));                          // lock mboard clocks
    // usrp_tx->set_time_source("external");                                        // Use PPS signal

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
    uhd::usrp::multi_usrp::sptr usrp_rx = uhd::usrp::multi_usrp::make(std::string(""));     // create a usrp device
    usrp_rx->set_master_clock_rate(CLOCKRATE);                                              // set clock rate
    usrp_rx->set_clock_source(std::string("internal"));                                     // lock mboard clocks
    // usrp_rx->set_time_source("external");                                                   // Use PPS signal
    usrp_rx->set_rx_subdev_spec(std::string("A:A"));                                        // select the subdevice
    usrp_rx->set_rx_rate(SAMPRATE,0);                                                       // set the sample rate
    usrp_rx->set_rx_freq(tune_request,0);                                                   // set the center frequency
    usrp_rx->set_rx_gain(RXGAIN,0);                                                         // set the rf gain
    usrp_rx->set_rx_antenna(std::string("RX2"),0);                                          // set the antenna
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));                             // allow for some setup time

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
    std::cout << boost::format("Actual time between pulses: %f sec...") % (DEBUG_PERIOD*SPB/SAMPRATE) << std::endl << std::endl;

        // set sigint so user can terminate via Ctrl-C
    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

        // setup receive streaming
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.stream_now = false;
    uhd::time_spec_t starttime = uhd::time_spec_t(0.25)+usrp_rx->get_time_now();
    stream_cmd.time_spec = starttime;  // tell USRP to start streaming 0.25 seconds in the future
    rx_stream->issue_stream_cmd(stream_cmd);

        // grab initial block of received samples from USRP with nice long timeout (gets discarded)
    rx_stream->recv(rxbuffs[0], SPB, md_rx, 3.0);

    while(not stop_signal_called){

            // grab block of received samples from USRP
        rx_stream->recv(rxbuffs[rxbuff_ctr], SPB, md_rx);

        if(state == FLIP0){
                // Flip current buffer to transmit 1st
            for (j = 0; j < SPB; j++){
                flipbuffs[0][j] = std::conj(rxbuffs[rxbuff_ctr][SPB-1-j]) * CINT16(FLIP_SCALING, 0);
            }
                // Initialize flip_ctr to 0
            flip_ctr = 0;

                // Once flipped, enter TRANSMIT state to transmit buffer
            state = TRANSMIT;
        }else{}

        /** CROSS CORRELATION *************************************************/
        for (i = 0; i < SPB; i++){
            xcorr = 0;  // Initialize xcorr variable

                // Cross correlation for circular buffer
            if(rxbuff_ctr == 0){
                for(j = 0; j < SPB-1-i; j++){
                    xcorr += (CINT32)rxbuffs[NUMRXBUFFS-1][i+j+1] * (CINT32)xcorr_sinc[j];
                }
                for(j = SPB-1-i; j < SPB; j++){
                    xcorr += (CINT32)rxbuffs[0][-SPB+1+i+j] * (CINT32)xcorr_sinc[j];
                }
            }else{
                for (j = 0; j < SPB; j++) {
                    xcorr += (CINT32)rxbuffs[rxbuff_ctr-1][i+j+1] * (CINT32)xcorr_sinc[j];
                }
            }

                // Compute abs^2 of xcorr divided by 2^3
            normxcorr[i] = std::norm(CINT32(xcorr.real() >> 3,xcorr.imag() >> 3));

            /** Save buffers if enabled by defines ****************************/
                // Save normxcorr if enabled by defined variables
            #if ((DEBUG != 0) && (WRITEXCORR != 0))
                normxcorr_write[(SPB*write_ctr)+i] = normxcorr[i];
            #else
            #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */

                // Trigger calculation block after extra buffer
            if ((normxcorr[i] >= THRESHOLD)&&(state == SEARCHING)){
                std::cout << "ps " << i << std::endl;
                std::cout << boost::format("Pulse detected at time: %15.8f sec") % (md_rx.time_spec.get_real_secs()) << std::endl << std::endl;

                    // Compute which rx buffer is the previously recorded buffer
                if(rxbuff_ctr - 1 == -1){
                    idx = NUMRXBUFFS-1;
                }else{
                    idx = rxbuff_ctr - 1;
                }
                    // Flip prevous buffer to transmit 3rd
                for (j = 0; j < SPB; j++) {
                    flipbuffs[2][j] = std::conj(rxbuffs[idx][SPB-1-j]) * CINT16(FLIP_SCALING, 0);
                }

                    // Flip current buffer to transmit 2nd
                for (j = 0; j < SPB; j++) {
                    flipbuffs[1][j] = std::conj(rxbuffs[rxbuff_ctr][SPB-1-j]) * CINT16(FLIP_SCALING, 0);
                }

                    // Change state to remember to flip next buffer
                state = FLIP0;
            }else{}
        }

            // set up transmit buffers to send sinc pulses (or zeros) on TX/RX-B (chan1)
        if (count == DEBUG_PERIOD-1) {
            txbuffs[1] = &dbug_sinc.front();
            count = 0;
        } else {
            txbuffs[1] = &zero.front();
            count++;
        }

            // When state is transmit, send flipped buffers
        if(state == TRANSMIT){
            txbuffs[0] = flipbuffs[flip_ctr];
            flip_ctr++;
                // After third buffer is queued, exit TRANSMIT state
            if(flip_ctr >= 3){
                state = SEARCHING;
            }else{}
        }else{
            txbuffs[0] = &zero.front();
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

        #if ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0)))
                // Increment write_ctr and exit when done recording
            if(write_ctr >= time){
                break;
            }else{
                write_ctr++;
            }
        #else
        #endif /* ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0))) */
    }

    #if ((DEBUG != 0) && (WRITEXCORR != 0))
        std::cout << std::endl;
        std::cout << "Writing normalized correlation to file..." << std::flush;
        writebuff("./xcorr.dat", &normxcorr_write.front(), SPB*write_ctr);
        std::cout << "done!" << std::endl;
    #else
    #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) rxbuffs*/

    #if ((DEBUG != 0) && (WRITERX != 0))
        std::cout << std::endl;
        std::cout << "Writing rx buffer to file..." << std::endl;
        writebuff("./rx.dat", rxbuffs[0], SPB*write_ctr);
        std::cout << "done!" << std::endl;
    #else
    #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */

        // send a mini EOB packet
    md_tx.end_of_burst = true;
    tx_stream->send("", 0, md_tx);
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));

    return EXIT_SUCCESS;
}   /** main() ****************************************************************/
