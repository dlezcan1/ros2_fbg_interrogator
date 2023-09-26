#include "fbg_interrogator/Interrogator.h"

#include <iostream>
#include <sstream>

Interrogator::Interrogator(const int nSensorsPerChannel) :
  //mtsTaskPeriodic("Interrogator", 5.0*cmn_ms),
  // setting interleave to 1 ms to get the maximum data streaming: 1000 Hz, could have some lag effects with the buffer
  // Also Qt gui will not work properly if this is set to very fast values
  interleave(10),
  numAverages(100),
  streaming(false),
  timeout(0.1)
{
  // there are always four channels
  // we assume three sensors to start
  /*
  vctDoubleVec v(nSensorsPerChannel);
  peaks.resize(4, v);
  unpackedPeaks.resize(9);
  */
  // Vectord v;
  
}

Interrogator::~Interrogator()
{
  disconnect();
}

bool Interrogator::connect(const std::string &host, const unsigned short port)
{
  // bool connected = socket.Connect(host.c_str(), port);
  tcp::endpoint endpoint(boost::asio::ip::make_address_v4(host), port);
  socket.connect(endpoint);

  if(this->isConnected()) {
    setDataInterleave(interleave);
    setNumAverages(numAverages);
    setStreaming(false);
    flushBuffer();
  }
  return this->isConnected();
}

void Interrogator::disconnect()
{
  if (this->isConnected()) {
    setStreaming(false);
    flushBuffer();
    socket.close();
  }
  streaming = false;
}

void Interrogator::setDataInterleave(unsigned int interleave)
{
  if(streaming) {
    std::cout << "Interleave cannot be set while streaming." << std::endl;
    return;
  }

  std::stringstream s;
  s << "#SET_DATA_INTERLEAVE " << interleave << std::endl;
  if(this->isConnected())
    send(s.str());
  else
    return;

  this->interleave = interleave;

  read();
}

void Interrogator::setNumAverages(unsigned int averages)
{
  if(streaming) {
    std::cout << "Averages cannot be set while streaming." << std::endl;
    return;
  }

  std::stringstream s;
  s << "#SET_NUM_AVERAGES " << averages << std::endl;
  if(this->isConnected())
    send(s.str());
  else
    return;

  numAverages = averages;
  read();
}

bool Interrogator::setStreaming(bool streaming)
{
  std::stringstream s;
  s << "#SET_STREAMING_DATA " << std::noboolalpha << streaming << std::endl;
  std::cout << s.str() << std::endl;
  if(this->isConnected())
    send(s.str());
  else
    return false;
  
  read();
  std::string valid(msgBuffer);
  this->streaming = streaming;
  return true;
  //return streaming;
}

void Interrogator::flushBuffer()
{
  std::stringstream s;
  s << "#FLUSH_BUFFER" << std::endl;
  if (this->isConnected())
    send (s.str());
  else
    return;

  read();
}

bool Interrogator::getDataUnbuffered()
{
  std::stringstream s;
  s << "#GET_UNBUFFERED_DATA" << std::endl;
  if (this->isConnected())
    send(s.str());
  else
    return false;

  // Read the data after sending the #GET_UNBUFFERED_DATA command to interrogator
  return read();
}

bool Interrogator::read()
{  
  // initialize the msg buffer to 0
  for (int i = 0; i < 132; i++) {
	  msgBuffer[i] = '0';
  }

  // First socket read to find the expected packet length
  char firstReadBuffer[10] = {'0'};
  // int bytesRecv = socket.Receive(firstReadBuffer, 10, timeout);
  int bytesRecv = socket.receive(boost::asio::buffer(firstReadBuffer));

  if (bytesRecv != 10) {
	  std::cout << "Bytes Received less than 10" << std::endl;
  }
  // for (int i = 0; i < 10; i++) {
	// std::cout << msgBuffer[i] << std::endl;
  // }

  if(bytesRecv == 0){
    return false;
  }

  //msgBuffer[bytesRecv] = '\0';

  // set the expected message length
  int msgLenExpected = 0;
  int msgLenReceived = 0;
  std::stringstream s;
  s << firstReadBuffer;
  s >> msgLenExpected;

  // second read to get the message packet
  //msgLenReceived = socket.Receive(msgBuffer, msgLenExpected, timeout);
  msgLenReceived = socket.receive(boost::asio::buffer(msgBuffer, msgLenExpected));
  //std::cout << "msgLenReceived = " << msgLenReceived << std::endl;

  // check if expected and received buffer are the same size
  int diffLen = 0;
  if (msgLenExpected != msgLenReceived) {
	  diffLen = msgLenExpected - msgLenReceived;
	  char* tempBuffer = new char[diffLen];
    // char* tempBuffer = new char[diffLen];
	  // int msgLenDiff = socket.Receive(tempBuffer, diffLen, timeout);
    // socket.Receive(tempBuffer, diffLen, timeout);
    socket.receive( boost::asio::buffer( tempBuffer, diffLen ) );
	  
    for (int i = 0; i < diffLen; i++) {
		  //tempBuffer[i] = 'Y';
		  msgBuffer[msgLenReceived + i] = tempBuffer[i];
	  }
  }
  //msgBuffer[msgLenReceived] = '\0';
  
  // check for synchronized data in streaming mode (Last 8 bytes should be XXXXXXXX)
  // This check only needs to be done if #SET_STREAMING_MODE is set to true
  // NOTE: Using the STREAMING mode will cause lag in the data during long experiments due to buffering
  // Refer to BIGSS Intranet WIKI for more info
  if (streaming) {
	  int checkCount = 0;
	  for (int i = 1; i < 9; i++) {
      std::cout << "Char " << i << " :" << msgBuffer[msgLenReceived + diffLen - i] << std::endl;
		  if (msgBuffer[msgLenReceived + diffLen - i] == 'X')
			  checkCount++;
	  }
    std::cout << std::endl;
	  if (checkCount != 8){
		  std::cout << " Message not synced! " << std::endl;  
      return 0;
    }
		  //read();
  }
  
  
  return msgLenReceived > 0;
}

void Interrogator::extractPeaks()
{
  // The peaks are organized after the header information
  // the header identifies the number of peaks per channel,
  // then these are ordered sequentially
  
  unsigned short nPeaks[4];

  // 16 bytes into the header
  char *dutBuf = msgBuffer + 16;

  // 88 bytes to end of header
  char *peakBuf = msgBuffer + 88;
  
  for (int k = 0; k < m_peaks.size(); k++) 
  {
    nPeaks[k] = getNumPeaks(dutBuf);
    // std::cout << "numPeaks[" << k << "] = " << nPeaks[k] << std::endl;
    dutBuf += 2;
    //std::cout << nPeaks[k] << "\t";

  } // for

  // obtain FBG data for each channel
  for (int i = 0; i < m_peaks.size(); i++)
  {
    m_peaks[i].clear(); // remove all peaks
    for (int j = 0; j < nPeaks[i]; j++) {
      m_peaks[i].push_back( getPeak( peakBuf ) );
      peakBuf += 4;
    } // for
  } // for

} // Interrogator::extractPeaks

unsigned short Interrogator::getNumPeaks(char *buf)
{
  union dutStatus d;

  // swap byte ordering
  d.raw[0] = buf[0];
  d.raw[1] = buf[1];

  return d.val;
}

double Interrogator::getPeak(char *buf)
{
  union peaks p;
  
  // swap byte ordering
  p.raw[0] = buf[0];
  p.raw[1] = buf[1];
  p.raw[2] = buf[2];
  p.raw[3] = buf[3];

  return p.val/1e6;
}

void Interrogator::setGain(int channel, int gain)
{
  std::stringstream s;
  s << "#SET_CH_GAIN_DB " << std::noboolalpha << channel << " " << gain 
    << std::endl;
  std::cout << s.str() << std::endl;

  if (this->isConnected())
    send(s.str());
  else
    return;

  read();
}

void Interrogator::setThreshold(int threshold)
{
  for(int channel=0 ; channel< m_peaks.size() ; channel++)
  {
    std::stringstream s;
    s << "#SET_CH_NOISE_THRESH " << std::noboolalpha << channel << " " << threshold << std::endl;
    std::cout << s.str() << std::endl;

    if(this->isConnected())
      send(s.str());
    else
      return;
  }

  read();

}

std::vector<double> Interrogator::getUnpackedPeaks()
{
  // unpacking channel 1 <- 4
  //                   2 <- 2
  //                   3 <- 3
  // channel 1 seems to have some issues so using channels 2,3,4
  for (std::vector<double> peaks_ch : m_peaks)
  {
    for (double peak : peaks_ch)
      unpackedPeaks.push_back(peak);

  } // for

  return unpackedPeaks;
}

void Interrogator::send(const std::string &command)
{
  socket.send(boost::asio::buffer(command));

} // Interrogator::send

void Interrogator::setTimeout(const int timeout_msec)
{
  timeout = timeout_msec;
  boost::asio::detail::socket_option::integer<SOL_SOCKET, SO_RCVTIMEO> rcv_timeo(timeout);
  boost::asio::detail::socket_option::integer<SOL_SOCKET, SO_SNDTIMEO> snd_timeo(timeout);
  socket.set_option(rcv_timeo);
  socket.set_option(snd_timeo);

} // Interrogator::setTimeout

//void Interrogator::Run()
//{
//  ProcessQueuedCommands();
//  ProcessQueuedEvents();
//
//  // read all things available.
//  if(this->isConnected() && streaming)
//    while(read())
//      extractPeaks();
//}
