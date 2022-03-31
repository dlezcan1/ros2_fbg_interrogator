#pragma once
#include <ctime>
#include <vector>
#include <array>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <chrono>

#define INTERROGATOR_PORT_SM130 1852
#define INTERROGATOR_MAX_CHANNELS 4

#define INTERROGATOR_BUFFER_SIZE 65624

#define INTERROGATOR_DEBUG
#ifdef INTERROGATOR_DEBUG
    #define INT_LOG_DEBUG(...) printf(__VA_ARGS__)
#else
    #define INT_LOG_DEBUG(...)
#endif

namespace Interrogator
{
    typedef struct std::array<std::vector<double>,INTERROGATOR_MAX_CHANNELS> PeakContainer;
    
    typedef char* buffer_t;

    /* Message structs */
    struct StatusHeader // status Header as laid out in ENLIGHT manual
    {
        int fullSpectrumRadix;        // DWORD 0, bits 0-7
        u_int8_t fpgaVersion;         // DWORD 0, bits 16-23
        bool fanSecondary;            // DWORD 0, bit 27
        bool fanPrimary;              // DWORD 0, bit 28
        bool calibrationFault;        // DWORD 0, bit 30

        u_int8_t switchPosition;      // DWORD 1, bits 16-17
        u_int8_t muxLevel;            // DWORD 1, bits 17-18
        u_int8_t triggerMode;         // DWORD 1, bits 20-21
        u_int8_t operatingMode;       // DWORD 1, bits 22-23
        
        u_int16_t CH1SensorsDetected; // DWORD4, bits 0-15
        u_int16_t CH2SensorsDetected; // DWORD4, bits 16-31
        u_int16_t CH3SensorsDetected; // DWORD5, bits 0-15
        u_int16_t CH4SensorsDetected; // DWORD5, bits 16-31

        u_int32_t serialNumber;       // DWORD 7
        double timeStamp;             // (microsecond) DWORD 8, (seconds) DWORD 9
        
        int errorCode;                // DWORD 11, bits 24-31
        
        u_int8_t bufferSize;          // DWORD 12, bits 0-7
        u_int8_t headerVersion;       // DWORD 12, bits 8-15
        u_int16_t headerSize;         // DWORD 12, bits 16-31

        u_int32_t granularity;        // DWORD 18
        float wavelengthStart;        // DWORD 20
        float wavelengthEnd;          // DWORD 21

        void print(){
            INT_LOG_DEBUG("Header:\n");
            INT_LOG_DEBUG("  bufferSize: %d\n", bufferSize);
            INT_LOG_DEBUG("  calibrationFault: %d\n", calibrationFault);
            INT_LOG_DEBUG("  errorCode: %d\n", errorCode);
            INT_LOG_DEBUG("  fanPrimary: %d\n", fanPrimary);
            INT_LOG_DEBUG("  fanSecondary: %d\n", fanSecondary);
            INT_LOG_DEBUG("  fpgaVersion: %d\n", fpgaVersion);
            INT_LOG_DEBUG("  fullSpectrumRadix: %d\n", fanSecondary);
            INT_LOG_DEBUG("  granularity: %d\n", granularity);
            INT_LOG_DEBUG("  headerSize: %d\n", headerSize);
            INT_LOG_DEBUG("  headerVersion: %d\n", headerVersion);
            INT_LOG_DEBUG("  muxLevel: %d\n", muxLevel);
            INT_LOG_DEBUG("  CH1SensorsDetected: %d\n", CH1SensorsDetected);
            INT_LOG_DEBUG("  CH2SensorsDetected: %d\n", CH2SensorsDetected);
            INT_LOG_DEBUG("  CH3SensorsDetected: %d\n", CH3SensorsDetected);
            INT_LOG_DEBUG("  CH4SensorsDetected: %d\n", CH4SensorsDetected);
            INT_LOG_DEBUG("  operatingMode: %d\n", operatingMode);
            INT_LOG_DEBUG("  serialNumber: %d\n", serialNumber);
            INT_LOG_DEBUG("  switchPosition: %d\n", switchPosition);
            INT_LOG_DEBUG("  switchPosition: %d\n", switchPosition);
            INT_LOG_DEBUG("  timeStamp: %f\n", timeStamp);
            INT_LOG_DEBUG("  triggerMode: %d\n", triggerMode);
            INT_LOG_DEBUG("  wavelengthStart: %f\n", wavelengthStart);
            INT_LOG_DEBUG("  wavelengthEnd:   %f\n", wavelengthEnd);


        }// StatusHeader::print
    };// struct: StatusHeader

    struct PeakMessage // #GET_DATA peak message
    {
        StatusHeader header;
        PeakContainer peaks;
    
    }; // struct: PeakMessage

    /* Interrogator Implementations */
    class AbstractInterrogator
    {
    public:
        // constructor and desctructor
        AbstractInterrogator(const std::string& ip_addr, int port) {};
        virtual ~AbstractInterrogator(){};

        /* commands */
        virtual buffer_t sendCommand(const char* command) = 0;

        virtual PeakMessage getData() = 0;

    // public
    protected:
        size_t m_bytesReceived = 0;
        buffer_t m_buffer = new char[INTERROGATOR_BUFFER_SIZE];
        int m_clientsock; 
        struct sockaddr_in m_server_addr;

        virtual buffer_t receive(buffer_t buffer, size_t msg_size){return buffer_t();}

    }; // class: AbstractInterrogator


    class Interrogator : AbstractInterrogator
    {
	public:
        Interrogator(const std::string& ip_addr, int port); // connect to the interrogator
        virtual ~Interrogator(); // close the connection

        /* commands */
        PeakMessage getData(); // get Peak Data
        buffer_t sendCommand(const char* command);

        void printBuffer()
        {
            INT_LOG_DEBUG("String: ");
            // for(int i = 0; i < m_bytesReceived; i++)
            INT_LOG_DEBUG("%s", m_buffer);

            INT_LOG_DEBUG("\nHex: ");

            for(int i = 0; i < m_bytesReceived; i++)
                INT_LOG_DEBUG("\\x%.2x", m_buffer[i]);


            INT_LOG_DEBUG("\n");

        } // printBuffer

	
	// public
    protected:
        inline void clearBuffer(){ memset(m_buffer, 0, INTERROGATOR_BUFFER_SIZE);
                                   m_bytesReceived = 0; } // wipe the buffer
        buffer_t receive(buffer_t buffer, size_t msg_size);
        StatusHeader parseHeader(const uint32_t* buffer, const size_t buffer_size) const;
        
    // protected

    }; // class: Interrogator
} // namespace: Interrogator
