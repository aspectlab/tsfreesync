/*******************************************************************************
 * txnode.cpp
 *
 * Sends out sinc pulses on two channels. Equipped with various configurations
 * for testing the functionality of the USRP.
 *
 * M.Overdick, A.G.Klein, and J.Canfield
 * Last Major Revision: 5/12/2016
 ******************************************************************************/

#include "includes.hpp"
#include <boost/program_options.hpp>

    // Compliation parameters
#define DEBUG           1           // Debug (binary) if 1, debug code compiled

#define WRITESINC       1           // Write Sinc (binary) if 1, template sinc pulse
                                    // is written to file "./sincX.dat"

    // Radio Parameters
#define SAMPRATE        100e3       // sampling rate (Hz)
#define CARRIERFREQ     900.0e6     // carrier frequency (Hz)
#define CLOCKRATE       30.0e6      // clock rate (Hz)
#define TXGAIN          60.0        // Tx frontend gain in dB

    // Transmission Parameters
#define SPB             1000        // samples per buffer
#define TXDELAY         3           // buffers in the future that we schedule transmissions (must be odd)
#define BW              0.1         // normalized bandwidth of sinc pulse (1 --> Nyquist)
#define CBW             0.5         // normalized freq offset of sinc pulse (1 --> Nyquist)

#define CLKRT           4.0190e-4   // Experimentally derived clock rate offset

#define CLKRT_ENABLE    0           // Enable clock rate compensation  (binary)

#define CH0_ENABLE      1           // Enable CH0 pulse transmission
                                    // 0 - Send only 0s, 1 - send pulses at CH0_PERIOD

#define CH1_ENABLE      1           // Enable CH1 pulse transmission
                                    // 0 - Send only 0s, 1 - send pulses at CH1_PERIOD

#define CH0_AMPLITUDE   30000       // Peak value of sinc pulse generated for debug channel (max 32768)

#define CH1_AMPLITUDE   30000       // Peak value of sinc pulse generated for debug channel (max 32768)

#define SINC_PRECISION  10000       // Precision of sinc pulse delays relative to SAMPRATE
                                    // Precision in seconds = 1/(SAMPRATE*SINC_PRECISION)

#define CH0_PERIOD      1           // ping tick period (in number of buffers... in actual time units, will be PING_PERIOD*SPB/SAMPRATE).
#define CH1_PERIOD      1           // ping tick period (in number of buffers... in actual time units, will be PING_PERIOD*SPB/SAMPRATE).

/*******************************************************************************
 * Signal handlers
 ******************************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}
namespace po = boost::program_options;

/*******************************************************************************
 * Main function
 ******************************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    /** Constant Decalartions *************************************************/

    /** Variable Declarations *************************************************/
        //variables to be set by program option
    FP32 ch0_delay;
    FP32 ch1_delay;

        // create sinc and zero
    std::vector< CINT16 >   sinc1(SPB);         // stores sinc pulse for CH0
    std::vector< CINT16 >   sinc0(SPB);         // stores sinc pulse for CH1
    std::vector< CINT16 >   zero(SPB, (0,0));   // stores all zeros for TX
    std::vector< CINT16 *>  txbuffs(2);         // pointer to facilitate 2-chan transmission

        // Counters
    INT16U ch0_ctr  = 0;            // Counter for transmitting pulses on CH0
    INT16U ch1_ctr  = 0;            // Counter for transmitting pulses on CH1
    FP32 ch0_clkos  = 0;            // Initial clock offset of CH0
    FP32 ch1_clkos  = 0;            // Initial clock offset of CH1

    /** Debugging vars ********************************************************/

    /** Argument Parsing ******************************************************/
        //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        #if (CLKRT_ENABLE)
            ("clkrt", po::value<FP32>(&ch1_delay)->default_value(CLKRT), "Clockrate offset")
        #endif /* #if (CLKRT != 0) */
        ("d0", po::value<FP32>(&ch0_delay)->default_value(0.0), "Channel 0 delay")
        ("d1", po::value<FP32>(&ch1_delay)->default_value(0.0), "Channel 1 delay")
    ;
    po::variables_map var_map;
    po::store(po::parse_command_line(argc, argv, desc), var_map);
    po::notify(var_map);


        // print the help message
    if (var_map.count("help")){
        std::cout << boost::format("TXnode %s") % desc << std::endl;
        return ~0;
    }

    /** Variable Initializations **********************************************/
    Sinc_Init(BW, CBW, SPB, SINC_PRECISION);

        // Initialize clock offset
    ch0_clkos = ch0_delay;
    ch1_clkos = ch1_delay;

        // Generate sinc pulses
    Sinc_Gen(&sinc0.front(), CH0_AMPLITUDE, SPB, ch0_clkos);
    Sinc_Gen(&sinc1.front(), CH1_AMPLITUDE, SPB, ch1_clkos);

    /** Debug code for writing sinc pulse *************************************/

        // Write template sinc pulse to file for debug
    #if ((DEBUG != 0) && (WRITESINC != 0))

        std::cout << "Writing Sinc0 to file...(TRX-A)" << std::flush;
        writebuff("./sinc0.dat", &sinc0.front(), SPB);
        std::cout << "done!" << std::endl;

        std::cout << "Writing Sinc1 to file...(TRX-B)" << std::flush;
        writebuff("./sinc1.dat", &sinc1.front(), SPB);
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
    std::cout << boost::format("Actual time between pulses CH 0 %f sec...") % (CH0_PERIOD*SPB/SAMPRATE) << std::endl;
    std::cout << boost::format("Actual time between pulses CH 1: %f sec...") % (CH1_PERIOD*SPB/SAMPRATE) << std::endl;
    std::cout << boost::format("CH 0 Delay: %f Samples...") % (ch0_delay) << std::endl;
    std::cout << boost::format("CH 1 Delay: %f Samples...") % (ch1_delay) << std::endl << std::endl;

        // set sigint so user can terminate via Ctrl-C
    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

    //md_tx.time_spec = usrp_tx->get_time_now();
    md_tx.time_spec = usrp_tx->get_time_now()+uhd::time_spec_t(2*SPB/SAMPRATE);

    #if (CH0_ENABLE)
        txbuffs[0] = &zero.front();
    #endif /* #if (CH0_ENABLE) */

    #if (CH1_ENABLE)
        txbuffs[1] = &zero.front();
    #endif /* #if (CH0_ENABLE) */

    while(not stop_signal_called){
            // Set time spec to be one buffer ahead in time
        // md_tx.time_spec = md_tx.time_spec+uhd::time_spec_t((TXDELAY+1)*(SPB)/SAMPRATE);
        md_tx.time_spec = md_tx.time_spec+uhd::time_spec_t(SPB/SAMPRATE);


            // CH0 TX
        #if (CH0_ENABLE)
            #if (CLKRT_ENABLE)
                ch0_clkos = ch0_clkos+CLKRT;    // Compensate for clock offset to achieve flat segments

                    // Generate new sinc pulse for CH0
                Sinc_Gen(&sinc0.front(), CH0_AMPLITUDE, SPB, ch0_clkos);
            #endif /* #if (CLKRT != 0) */

            if (ch0_ctr == CH0_PERIOD-1) {
                txbuffs[0] = &sinc0.front();
                ch0_ctr = 0;
            } else {
                txbuffs[0] = &zero.front();
                ch0_ctr++;
            }
        #endif /* #if (CH0_ENABLE) */

            // CH1 TX
        #if (CH1_ENABLE)
            #if (CLKRT_ENABLE)
                ch1_clkos = ch1_clkos+CLKRT;    // Compensate for clock offset to achieve flat segments

                    // Generate new sinc pulse for CH1
                Sinc_Gen(&sinc1.front(), CH0_AMPLITUDE, SPB, ch1_clkos);
            #endif /* #if (CLKRT != 0) */

            if (ch1_ctr == CH1_PERIOD-1) {
                txbuffs[1] = &sinc1.front();
                ch1_ctr = 0;
            } else {
                txbuffs[1] = &zero.front();
                ch1_ctr++;
            }
        #endif /* #if (CH0_ENABLE) */

            // Transmit both buffers
        tx_stream->send(txbuffs, SPB, md_tx);
        md_tx.start_of_burst = false;

    }   /** while(not stop_signal_called) *************************************/

        // send a mini EOB packet
    md_tx.end_of_burst = true;
    tx_stream->send("", 0, md_tx);
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));

    return EXIT_SUCCESS;
}   /** main() ****************************************************************/
