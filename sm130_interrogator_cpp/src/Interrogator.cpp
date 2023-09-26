#include "Interrogator.h"
#include <bitset>
#include <stdexcept>

#include <sys/time.h>

/* Helper functions for bit-wise operations */
#define BIT(x) ( 1 << (x) ) // bit-shift operation

uint32_t bitMask(std::vector<u_int> bits)
{
    uint32_t mask = 0;

    for (u_int bit : bits)
        mask |= BIT(bit);

    return mask;

} // bitMask

uint32_t bitMaskRange(u_int lo, u_int hi)
{
    if (lo > hi) // swap
    {
        u_int tmp = lo;
        lo = hi;
        hi = tmp;

    } // if

    std::vector<u_int> bits;
    for (u_int bit = lo; bit <= hi; bit++) // and including
        bits.push_back(bit);

    return bitMask(bits);

} // bitMaskRange

template <typename T>
T flipBits(const T inp)
{
    T aux = 0, temp;
    const uint num_bits = 8*sizeof(T);
    for(int i = 0; i < num_bits; i+=8)
    {
        temp = (inp & BIT(i));
        if (temp)
            aux |= BIT(num_bits-1-i);

    } // for

    return aux;

} // flipBits

/* Interrogator implementations */
namespace Interrogator
{
    Interrogator::Interrogator(const std::string& ip_addr, int port): AbstractInterrogator::AbstractInterrogator(ip_addr, port)
    {
        
        // Connect to the interrogator
        m_clientsock = socket(AF_INET, SOCK_STREAM, 0); // get socket
        
        m_server_addr.sin_family = AF_INET;
        m_server_addr.sin_port = htons(port);

        // convert ip address to binary form
        inet_pton(AF_INET, ip_addr.c_str(), &m_server_addr.sin_addr);

        // set socket options
        struct timeval timeout;
        timeout.tv_sec = 2;
        timeout.tv_usec = 0;

        int to_status = setsockopt(m_clientsock, SOL_SOCKET, SO_RCVTIMEO, (char*) &timeout, sizeof(timeout));
        setsockopt(m_clientsock, SOL_SOCKET, SO_SNDTIMEO, (char*) &timeout, sizeof(timeout));
        INT_LOG_DEBUG("Timeout status: %d\n", to_status);

        // connect to the interrogator's server
        int connection_status = connect(m_clientsock, (struct sockaddr*)&m_server_addr, sizeof(m_server_addr));
        if (connection_status < 0)
            throw std::runtime_error("Interrogator: Connection not established.");

        clearBuffer();

    } // Interrogator::constructor

    Interrogator::~Interrogator()
    {
        // Close connection.
        

    } // Interrogator::destructor

    PeakMessage Interrogator::getData()
    {
        // Get data command
        buffer_t response = sendCommand("#GET_UNBUFFERED_DATA");
        

        // Parse the header
        StatusHeader header = parseHeader((uint32_t*) response, m_bytesReceived);
        header.print();
        
        // prepare peak message
        PeakMessage peak_msg = PeakMessage();
        double peak_value = 0.0;
        peak_msg.header = header;
        uint32_t* response_ui32 = (uint32_t*) (response + 88); // cast to 32-bit for datagrams & cut-out header
        
        // extract the actual peaks
        int index = 0; // header-size offset
        for (int i = 0; i < header.CH1SensorsDetected; i++) // CH 1
        {
            peak_value = ((double) response_ui32[index++])/((double) header.granularity);
            peak_msg.peaks[0].push_back(peak_value);
            
        } // for
        
        for (int i = 0; i < header.CH2SensorsDetected; i++) // CH 2
        {
            peak_value = ((double) response_ui32[index++])/((double) header.granularity);
            peak_msg.peaks[1].push_back(peak_value);
            
        } // for
        
        for (int i = 0; i < header.CH3SensorsDetected; i++) // CH 3
        {
            peak_value = ((double) response_ui32[index++])/((double) header.granularity);
            peak_msg.peaks[2].push_back(peak_value);
            
        } // for
        
        for (int i = 0; i < header.CH4SensorsDetected; i++) // CH 4
        {
            peak_value = ((double) response_ui32[index++])/((double) header.granularity);
            peak_msg.peaks[3].push_back(peak_value);
            
        } // for
                

        return peak_msg;

    }// Interrogator::getData

    StatusHeader Interrogator::parseHeader(const uint32_t* buffer, const size_t buffer_size) const 
    {
        StatusHeader header;

        // handle DWORD0
        header.fullSpectrumRadix = buffer[0] & bitMaskRange(0,7);
        header.fpgaVersion = ((buffer[0] & bitMaskRange(16, 23)) >> 16);
        header.fanSecondary = (buffer[0] & bitMask({28})) >> 27;
        header.fanPrimary = (buffer[0] & bitMask({29})) >> 28;
        header.calibrationFault = (buffer[0] & bitMask({30})) >> 30;

        // handle DWORD1
        header.switchPosition = (buffer[1] & bitMask({16, 17})) >> 16;
        header.muxLevel = (buffer[1] & bitMask({17, 18})) >> 17;
        header.triggerMode = (buffer[1] & bitMask({20, 21})) >> 20;
        header.operatingMode = (buffer[1] & bitMask({22, 23})) >> 22;

        // handle DWORDS 4-5: number of sensors detected)
        header.CH1SensorsDetected = (buffer[4] & bitMaskRange(0,7));
        header.CH2SensorsDetected = (buffer[4] & bitMaskRange(16,31)) >> 16;
        header.CH3SensorsDetected = (buffer[5] & bitMaskRange(0,7));
        header.CH4SensorsDetected = (buffer[5] & bitMaskRange(16,31)) >> 16;

        // handle DWORDS 7-9: serial number and timeStamp
        header.serialNumber = buffer[7];
        // header.serialNumber = flipBits(header.serialNumber);
        header.timeStamp = (float) buffer[8] + ((float) buffer[9])/(1.0e6);

        // handle DWORD 11: error code
        header.errorCode = (buffer[11] & bitMaskRange(24, 31)) >> 24;

        // handle DWORD 12
        header.bufferSize = (buffer[12] & bitMaskRange(0,7));
        header.headerVersion = (buffer[12] & bitMaskRange(8,15)) >> 8;
        header.headerSize = (buffer[12] & bitMaskRange(16,31)) >> 16;

        // handle DWORDS 18-21
        header.granularity = (buffer[18]);
        header.wavelengthStart = ((float) buffer[20])/((float) header.granularity);
        header.wavelengthEnd = ((float) buffer[21])/((float) header.granularity);

        return header;

    } // Interrogator::parseHeader

    buffer_t Interrogator::receive(buffer_t buffer, size_t msg_size)
    {
        // TODO: implement receive function
        return m_buffer;

    } // Interrogator::receive

    buffer_t Interrogator::sendCommand(const char* command)
    {
        // check if command ends with a "\n"
        std::string cmd(command);
        if (cmd.length() == 0)
            cmd = "\n";

        else if (cmd.compare(cmd.length()-2, 1, "\n") != 0)
            {INT_LOG_DEBUG("Appending \\n\n"); cmd += "\n";}
        
        // send command
        INT_LOG_DEBUG("Sending command: %s\n", cmd.c_str());
        send(m_clientsock, cmd.c_str(), cmd.length(),0);

        // receive message length | header
        clearBuffer();
        m_bytesReceived = recv(m_clientsock, m_buffer, 10, MSG_WAITALL);
        INT_LOG_DEBUG("Buffer header: %d | %s\n", (int) strlen(m_buffer), m_buffer);

        if (m_bytesReceived > 0)
        {
            // parse the message size
            int msg_size = std::stoi(m_buffer);
            INT_LOG_DEBUG("Input Message size: %d\n", msg_size);

            // read the message
            clearBuffer();
            m_bytesReceived = recv(m_clientsock, m_buffer, msg_size, MSG_WAITALL);  
            INT_LOG_DEBUG("Read size: %zu-%d\n", m_bytesReceived, (int) strlen(m_buffer));
            
            INT_LOG_DEBUG("Interrogator Response: %s\n", m_buffer);
            INT_LOG_DEBUG("Buffer: ");
            printBuffer();
        
        }// if
        else
            clearBuffer();


        buffer_t return_buffer = new char[INTERROGATOR_BUFFER_SIZE];
        memcpy(return_buffer, m_buffer, INTERROGATOR_BUFFER_SIZE);
        
        return return_buffer; 

    } // Interrogator::sendCommand


} // namespace: Interrogator

