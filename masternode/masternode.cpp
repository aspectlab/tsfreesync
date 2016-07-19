/*******************************************************************************
 * masternode.cpp
 *
 * This source file implements the "master" node in the timestamp free protocol.
 *
 * M.Overdick, A.G.Klein, and J.Canfield
 * Last Major Revision: 5/12/2016
 ******************************************************************************/

#include "includes.hpp"

    // Compilation Parameters
#define DEBUG           0           // Debug (binary) if 1, debug code compiled

#define DURATION        10           // Duration of recording (s)

#define WRITESINC       1           // Write template sinc pulses (binary)

#define WRITERX         1           // Enable writing RX buffer to file (binary)

#define WRITEXCORR      1           // Write normalized cross correlation to file (binary)

    // Radio Parameters
#define SAMPRATE        100e3       // Sampling rate (Hz)
#define CARRIERFREQ     900.0e6     // Carrier frequency (Hz)
#define CLOCKRATE       30.0e6      // Clock rate (Hz)
#define TXGAIN0         50.0        // TX frontend gain, Ch 0 (dB)
#define TXGAIN1         60.0        // TX frontend gain, Ch 1 (dB)
#define RXGAIN          0.0         // RX frontend gain (dB)

    // Transmission Parameters
#define SPB             1000        // Samples Per Buffer
#define NRXBUFFS        3           // Number of receive buffers (circular)
#define TXDELAY         3           // Buffers in the future that we schedule transmissions (must be odd)
#define BW              0.1         // Normalized bandwidth of sinc pulse (1 --> Nyquist)
#define CBW             0.5         // Normalized freq offset of sinc pulse (1 --> Nyquist)
#define DEBUG_PERIOD    1           // Debug Period (# of buffers)

    // Sinc pulse amplitudes (integer)
#define SINC_PRECISION  1e-9        // Precision of pre-generated sinc pulse (in s)
#define XCORR_AMP       64          // Peak value of sinc pulse generated for cross correlation (recommended to be 64)
#define DBSINC_AMP      30000       // Peak value of sinc pulse generated for debug channel (max 32768)

#define THRESHOLD       1e8         // Threshold of cross correlation pulse detection
#define FLIP_SCALING    50          // scale factor used when re-sending flipped signals... depends heavily on choice of TXGAIN and RXGAIN

typedef enum {SEARCHING, FLIP3, FLIP2, TRANSMIT} STATES;

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
    std::vector< CINT16 >   zero2(SPB, (0,0));           // stores all zeros for Tx
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

    INT16U i = 0, j = 0, k=0;
    INT16 m = 0, n = 0;
    INT16U count = 0, rxbuff_ctr = 0 , idx = 0;
    CINT32 xcorr = 0;
    STATES state = SEARCHING;



    /** Variable Initializations **********************************************/
        // Initialise rxbuffs (Vector of pointers)
    for(i = 0; i < NUMRXBUFFS; i++){
        rxbuffs[i] = &rxbuff.front() + SPB * i;
    }

        // Initialise rxbuffs (Vector of pointers)
    for(i = 0; i < 3; i++){
        flipbuffs[i] = &flipbuff.front() + SPB * i;
    }

    Sinc_Init(BW, CBW, SPB, SINC_PRECISION, SAMPRATE);

    Sinc_Gen(&xcorr_sinc.front(), XCORR_AMP, SPB, 0.0);
    for (j = 0; j < SPB; j++){
        xcorr_sinc[j] = std::conj(xcorr_sinc[j]);
    }

    Sinc_Gen(&dbug_sinc.front(), DBSINC_AMP, SPB, 0.0);

    /** Debug code for writing sinc pulse *************************************/

        // Write template sinc pulse to file for debug
    #if ((DEBUG != 0) && (WRITESINC != 0))
        std::cout << "Writing Sinc to file..." << std::flush;
        writebuff_CINT16("./xcorr_sinc.dat", &xcorr_sinc.front(), SPB);
        writebuff_CINT16("./dbug_sinc.dat", &dbug_sinc.front(), SPB);
        std::cout << "done!" << std::endl;
    #else
    #endif /* #if ((DEBUG != 0) && (WRITESINC != 0)) */

    /** Code ******************************************************************/

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

            // 4-state machine determines what to send on antenna TX/RX-A (chan0)... send either zeros, or flipped receive buffers
        txbuffs[0] = &zero.front();

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

                // Compute abs^2 of xcorr divided by 4
            normxcorr[i] = std::norm(CINT32(xcorr.real() >> 3,xcorr.imag() >> 3));

            /** Save buffers if enabled by defines ****************************/
                // Save normxcorr if enabled by defined variables
            #if ((DEBUG != 0) && (WRITEXCORR != 0))
                normxcorr_write[(SPB*write_ctr)+i] = normxcorr[i];
            #else
            #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */

                // Trigger calculation block after extra buffer
            if ((normxcorr[i] >= THRESHOLD)&&(state == SEARCHING)){
                std::cout << boost::format("Pulse detected at time: %15.8f sec") % (md_rx.time_spec.get_real_secs()) << std::endl;

                if(rxbuff_ctr - 1 == -1){
                    idx = NUMRXBUFFS-1;
                }else{
                    idx = rxbuff_ctr - 1;
                }

                for (j = 0; j < SPB; j++) {
                    flipbuffs[2][j] = std::conj(rxbuffs[idx][SPB-1-j]) * CINT16(FLIP_SCALING, 0);
                }
                state = FLIP3;

            }else{}
        }

        switch (state) {
            case FLIP3: // state 1 -- flip third segment, and transmit
                for (j = 0; j < SPB; j++) {
                    flipbuffs[0][j] = std::conj(rxbuffs[rxbuff_ctr][SPB-1-j]) * CINT16(FLIP_SCALING, 0);
                }
                txbuffs[0] = flipbuffs[0];
                // txbuffs[0] = &xcorr_sinc.front();
                state = FLIP2;
                break;

            case FLIP2:  // state 2 -- flip second segment, and transmit
                for (j = 0; j < SPB; j++){
                    flipbuffs[1][j] = std::conj(rxbuffs[rxbuff_ctr][SPB-1-j]) * CINT16(FLIP_SCALING, 0);

                }
                txbuffs[0] = flipbuffs[1];
                // txbuffs[0] = &xcorr_sinc.front();
                state = TRANSMIT;
            break;

            case SEARCHING:
                // DO NOTHING
            break;

            default: // state 3 -- transmit flipped first segment
                txbuffs[0] = flipbuffs[2];
                // txbuffs[0] = &xcorr_sinc.front();
                state = SEARCHING;
            break;
        }

            // set up transmit buffers to send sinc pulses (or zeros) on TX/RX-B (chan1)
        if (count == DEBUG_PERIOD-1) {
            txbuffs[1] = &dbug_sinc.front();
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


        #if ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0)))
                // Report progress to terminal
            std::cout << boost::format("\r\t Recording... %2i Percent Complete") % (write_ctr*100/time) << std::flush;
        #else
        #endif /* ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0))) */

        #if ((DEBUG != 0) && (WRITEXCORR != 0))
            if(write_ctr >= time){
                std::cout << std::endl;
                std::cout << "Writing normalized correlation to file..." << std::flush;
                writebuff_INT32U("./xcorr.dat", &normxcorr_write.front(), SPB*time);
                std::cout << "done!" << std::endl;

                // Don't break if we want to record RX too
                #if (WRITERX == 0)
                break;
                #endif /* (WRITERX != 0) */
            }else{}
        #else
        #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) rxbuffs*/

        #if ((DEBUG != 0) && (WRITERX != 0))
            if(write_ctr >= time){
                std::cout << std::endl;
                std::cout << "Writing rx buffer to file..." << std::endl;
                writebuff_CINT16("./rx.dat", rxbuffs[0], SPB*time);
                std::cout << "done!" << std::endl;
                break;
            }else{}
        #else
        #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */

        #if ((DEBUG != 0) && (WRITEXCORR != 0))
            write_ctr++;
        #else
        #endif /* ((DEBUG != 0) && (WRITEXCORR != 0)) */
    }

        // send a mini EOB packet
    md_tx.end_of_burst = true;
    tx_stream->send("", 0, md_tx);
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));

    return EXIT_SUCCESS;
}   /** main() ****************************************************************/
