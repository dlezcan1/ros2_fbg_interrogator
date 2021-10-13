#ifndef _INTERROGATOR_H
#define _INTERROGATOR_H

#include <vector>
#include <array>
#include <boost/asio.hpp>
// #include <Eigen/Dense>
#include <iostream>

/*
#include <cisstOSAbstraction/osaSocket.h>
#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
*/ 

// Interrogator options
#define INTERROGATOR_SM130 1
#define INTERROGATOR_SI155 2 

#if INTERROGATOR == INTERROGATOR_SM130
  #define HOST_IP  "192.168.1.11"
  #define HOST_PORT 1842
#elif INTERROGATOR == INTERROGATOR_SI155
  #define HOST_IP "10.0.0.55" 
  #define HOST_PORT 51692 // TODO
#else
  #define HOST_IP ""
  #define HOST_POST 0
#endif

/*!
  \brief Interface for the FBG interrogator

*/

typedef std::vector<double> Peaks;
typedef std::array< Peaks, 4 > PeakContainer;
using boost::asio::ip::tcp;

class Interrogator { //: mtsTaskPeriodic {
  // attributes
protected:
  /// TCP socket
  // osaSocket socket;
  boost::asio::io_service io_service;
  tcp::socket socket{io_service};

  // interleaves for the interrogator
  unsigned int interleave;
  unsigned int numAverages;
  
  bool streaming;

  // max length of buffer is:
  // = 88 (22* 32bit of header) + 500 (num sensors) * 4 (bytes/sensor) + 8 (8 bytes at end for synchronization) 
  char msgBuffer[2096];

  int timeout; // in milliseconds

  // 16-bit status indiciating number of peaks per channel
  union dutStatus {
    char raw[2];
    unsigned short val;
  };

  // 32-bit status indicating peaks on each channel
  union peaks {
    char raw[4];
    int val;
  };

  PeakContainer peaks; // 4 for number of channels 
  std::vector<double> unpackedPeaks; // peaks ravelled out
 
  // methods
public:

  /*!
    Interrogator constructor
    \param nSensorsPerChannel The number of sensors attached to each channel. Default is 3
  */
  Interrogator(const int nSensorsPerChannel = 3);

  /*!
    Specify number of sesnors on each channel
    \param nX Channel 1 sensors
    \param nY Channel 2 sensors
    \param nZ Channel 3 sensors
    \param nW Channel 4 sensors
    */
  //interrogator(const int nX, const int nY, const int nZ, const int nW);
  virtual ~Interrogator();

  /*!
    \brief Connect to the interrogator
  */
  bool connect(const std::string &host, const unsigned short port);

  /*!
    Disconnect the socket
  */
  void disconnect();

  /*!
    Return true if the socket is connected
  */
  bool isConnected() const {return socket.is_open();};
  bool isStreaming() const {return streaming;};

  /*!
    Set the read and write timeout (in s)
  */
  void setTimeout(const int timeout_msec);
  int getTimeout() const {return timeout;};

  /*!
    Set the data interleave
    \param interleave Default to 100 (5ms interval)
  */
  void setDataInterleave(unsigned int interleave);
  unsigned int getDataInterleave() const {return interleave;};

  /*!
    Set the number of averages
    \param averages The data average [100]
  */
  void setNumAverages(unsigned int averages);

  /*!
    Set if data is streaming
    \param streaming If true, stream data. Otherwise must request
  */
  bool setStreaming(bool streaming);

  /*!
    Set the reference values for the FBG sensors
    \param n The number of data points to collect and average
  */
  //void setReferenceValues(const unsigned int n);

  PeakContainer getPeaks() {return peaks;};
  std::vector<double> getUnpackedPeaks();
  /*!
    Read the response
    \return True if a response is read
   */
  bool read();

  bool getDataUnbuffered();

  /*!
    Extract the peaks from the channels
  */
  void extractPeaks();

  /*!
    Set gain on a given channel
   */
  void setGain(int channel, int gain);
  void setThreshold(int threshold);

  /// mts task
  //virtual void Startup() {};
  //virtual void Cleanup() {};
  //virtual void Configure(const std::string &MARKED_AS_UNUSEDfilename = "") {};
  //virtual void Run();

protected:
  /*!
    Get the number of peaks
    \param buf The buffer to use.
  */
  unsigned short getNumPeaks(char *buf);
  
  /*!
    Get the current peak
  */
  double getPeak(char *buf);

  /// Flush the buffer
  void flushBuffer ();

  void send(const std::string &command);

};

#endif // _INTERROGATOR_H
