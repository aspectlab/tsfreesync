/*******************************************************************************
 * rxnode.cpp
 *
 * This source file implements a two channel receiver on the USRP E310.
 *
 * M.Overdick, A.G.Klein, and J.Canfield
 * Last Major Revision: 5/12/2016
 ******************************************************************************/

#include "includes.hpp"

    // tweakable parameters
#define DURATION        20          // Length of time to record in seconds
#define SAMPRATE        3e6         // sampling rate (Hz)
#define CARRIERFREQ     900.0e6     // carrier frequency (Hz)
#define CLOCKRATE       30.0e6      // clock rate (Hz)
#define RXGAIN          16.0        // Rx frontend gain in dB
#define SPB             1000        // samples per buffer

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
    const INT32U time = DURATION*(SAMPRATE/SPB);

    /** Variable Declarations *************************************************/

    // (circular) receive buffers
    std::vector< CINT16 >   ch0_rxbuff(time*SPB);   // Ch 0 is RX2-A
    std::vector< CINT16 >   ch1_rxbuff(time*SPB);   // Ch 1 is RX2-B

    // Vector of pointers to sectons of rx_buff
    std::vector< std::vector< CINT16 *> >   rxbuffs(time*2, std::vector< CINT16 *>(2));

        // Holds the number of received samples returned by rx_stream->recv()
    INT16U num_rx_samps;

        // Counters
    INT16U i = 0,j = 0,k = 0;               // Generic counters
    INT32U rx_ctr = 0;                      // Counts loops through main while()

    /** Variable Initializations **********************************************/
        // Initialise rxbuffs (Vector of pointers)
    for(i = 0; i < time; i++){
        rxbuffs[i][0] = &ch0_rxbuff.front() + SPB * i;
        rxbuffs[i][1] = &ch1_rxbuff.front() + SPB * i;
    }

    /** Main code *************************************************************/

        // set USRP Rx params
    uhd::usrp::multi_usrp::sptr usrp_rx = uhd::usrp::multi_usrp::make(std::string("")); // create a usrp device
    uhd::tune_request_t tune_request(CARRIERFREQ);                                      // validate tune request
    usrp_rx->set_master_clock_rate(CLOCKRATE);                                          // set clock rate
    usrp_rx->set_clock_source(std::string("internal"));                                 // lock mboard clocks
    usrp_rx->set_rx_subdev_spec(std::string("A:A A:B"));                                // select the subdevice
    usrp_rx->set_rx_rate(SAMPRATE,0);                                                   // set the sample rate (Ch 0)
    usrp_rx->set_rx_rate(SAMPRATE,1);                                                   // set the sample rate (Ch 1)
    usrp_rx->set_rx_freq(tune_request,0);                                               // set the center frequency (Ch 0)
    usrp_rx->set_rx_freq(tune_request,1);                                               // set the center frequency (Ch 1)
    usrp_rx->set_rx_gain(RXGAIN,0);                                                     // set the rf gain (Ch 0)
    usrp_rx->set_rx_gain(RXGAIN,1);                                                     // set the rf gain (Ch 1)
    usrp_rx->set_rx_antenna(std::string("TX/RX"),0);                                    // set the antenna (Ch 0)
    usrp_rx->set_rx_antenna(std::string("TX/RX"),1);                                    // set the antenna (Ch 1)
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));                         // allow for some setup time

        // check Ref and LO Lock detect for Rx
    check_locked_sensor(usrp_rx->get_rx_sensor_names(0), "lo_locked", boost::bind(&uhd::usrp::multi_usrp::get_rx_sensor, usrp_rx, _1, 0), 1.0);

        // create a receive streamer
    uhd::stream_args_t stream_args_rx("sc16", "sc16");
     stream_args_rx.channels = boost::assign::list_of(0)(1);
    uhd::rx_streamer::sptr rx_stream = usrp_rx->get_rx_stream(stream_args_rx);
    uhd::rx_metadata_t md_rx;

        // report stuff to user (things which may differ from what was requested)
    std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp_rx->get_rx_rate()/1e6) << std::endl;

        // set sigint so user can terminate via Ctrl-C
    std::signal(SIGINT, &sig_int_handler);
    std::cout << boost::format("Recording RX CH 0 and CH 1 for %i seconds") % DURATION << std::endl << std::endl;
    std::cout << "Press Enter to start recording..." << std::endl << std::endl;

        // Wait for "ENTER" key to be pressed
    while(std::cin.get() != '\n'){}

    std::cout << "Press Ctrl + C to stop recording..." << std::endl;

        // setup receive streaming
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = uhd::time_spec_t(0.25)+usrp_rx->get_time_now();  // tell USRP to start streaming 0.25 seconds in the future
    rx_stream->issue_stream_cmd(stream_cmd);

        // grab initial block of received samples from USRP with nice long timeout (gets discarded)
    num_rx_samps = rx_stream->recv(rxbuffs[0], SPB, md_rx, 3.0);

    while(not stop_signal_called){
            // grab block of received samples from USRP
        num_rx_samps = rx_stream->recv(rxbuffs[rx_ctr], SPB, md_rx);

            // Increment counter
        rx_ctr++;

            // Check if full time has passed
        if(rx_ctr == time){
            break;
        }else{}

            // Report progress to terminal
        std::cout << boost::format("\r\t%2i Percent Complete      ") % (rx_ctr*100/time) << std::flush;

    }   /** while(not stop_signal_called) *************************************/

    if(stop_signal_called){
        std::cout << std::endl << "Cancelled." << std::endl;
    }else{
            // Report progress to terminal
        std::cout << "\r\tdone!               " << std::endl << std::endl;

            // Write buffers to file
        std::cout << "Writing buffers to file (this may take awhile)..." << std::endl;

        std::cout << "    Channel 0 (RX2-A)..." << std::flush;
        writebuff("./RX2-A.dat", &ch0_rxbuff.front(), time*SPB);
        std::cout << "done!" << std::endl;

        std::cout << "    Channel 1 (RX2-B)..." << std::flush;
        writebuff("./RX2-B.dat", &ch1_rxbuff.front(), time*SPB);
        std::cout << "done!" << std::endl;
    }

    return EXIT_SUCCESS;
}   /** main() ****************************************************************/
