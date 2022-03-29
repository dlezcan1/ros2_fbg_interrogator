import socket
from typing import List, Union
from dataclasses import dataclass, field

import numpy as np


# ==================================================================================================
# =========================== CONTAINER CLASSES ====================================================
# ==================================================================================================
@dataclass
class StatusHeader:
    HEADER_SIZE = 88
    fullSpectrumRadix: int = None
    serialNumber: int = None
    granularity: float = None

    numCH1Sensors: int = None
    numCH2Sensors: int = None
    numCH3Sensors: int = None
    numCH4Sensors: int = None

    startWavelength: float = None
    endWavelength: float = None

    timeStamp: float = None  # ms

    bufferSize: int = None
    headerSize: int = None
    headerVersion: int = None


# dataclass: StatusHeader

@dataclass
class PeakContainer:
    CH1: List[ float ] = field( default_factory=list )
    CH2: List[ float ] = field( default_factory=list )
    CH3: List[ float ] = field( default_factory=list )
    CH4: List[ float ] = field( default_factory=list )

    @property
    def peaks( self ):
        return [ self.CH1, self.CH2, self.CH3, self.CH4 ]

    # peaks


# dataclass: PeakContainer

@dataclass
class Spectrum:
    wavelengths: List[ float ]
    amplitudes: List[ float ]

    @staticmethod
    def wavelengthsFromStartEnd( start_wl: float, end_wl: float, N: int ):
        return np.linspace( start_wl, end_wl, N ).tolist()

    # wavelengthsFromStartEnd


# dataclass: Spectrum

@dataclass
class SpectrumContainer:
    CH1: Spectrum = field( default_factory=lambda: Spectrum( wavelengths=[ ], amplitudes=[ ] ) )
    CH2: Spectrum = field( default_factory=lambda: Spectrum( wavelengths=[ ], amplitudes=[ ] ) )
    CH3: Spectrum = field( default_factory=lambda: Spectrum( wavelengths=[ ], amplitudes=[ ] ) )
    CH4: Spectrum = field( default_factory=lambda: Spectrum( wavelengths=[ ], amplitudes=[ ] ) )

    @property
    def spectrum( self ):
        return [ self.CH1, self.CH2, self.CH3, self.CH4 ]

    # property: spectrum


# dataclass: SepctrumContainer

@dataclass
class PeakMessage:
    header: StatusHeader = field( default_factory=StatusHeader )
    peak_container: PeakContainer = field( default_factory=PeakContainer )


# PeakMessage

@dataclass
class SpectrumMessage:
    SPECTRUM_ELEMENT_COUNT = 32768  # number of 16-bit integers in the amplitudes
    header: StatusHeader = field( default_factory=StatusHeader )
    spectrum_container: SpectrumContainer = field( default_factory=SpectrumContainer )


# dataclass: SpectrumMessage

# ==================================================================================================
# =========================== INTERROGATOR IMPLEMENTATION ==========================================
# ==================================================================================================

class Interrogator():
    def __init__( self, address, port, timeout: float = 5 ):
        self.sock = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
        self.socketTimeout = timeout # set the socket's timeout
        self.connect( address, port )

    # __init__

    @property
    def socketTimeout( self ):
        return self.sock.gettimeout()

    # property: socketTimeout

    @socketTimeout.setter
    def socketTimeout( self, timeout: float ):
        self.sock.settimeout( timeout )

    # property setter: socketTimeout

    def connect( self, address, port ):
        self.sock.connect( (address, port) )

    # connect

    def getData( self ):
        """ Get Peak data from all channels

            :return PeakMessage object for the 4 channels
        """
        data = self.sendCommand( "#GET_UNBUFFERED_DATA" )

        peak_msg = PeakMessage()
        peak_msg.header = self.parseHeader( data )

        # parse the data
        offset = peak_msg.header.HEADER_SIZE
        for ch in range( 4 ):
            # get the number of peaks in the header
            if ch == 0:
                num_peaks = peak_msg.header.numCH1Sensors
            elif ch == 1:
                num_peaks = peak_msg.header.numCH2Sensors
            elif ch == 2:
                num_peaks = peak_msg.header.numCH3Sensors
            elif ch == 3:
                num_peaks = peak_msg.header.numCH4Sensors
            else:
                continue

            # parse the peaks
            for _ in range( num_peaks ):
                peak_val = int.from_bytes(
                        data[ offset:offset + 4 ], 'little' ) / peak_msg.header.granularity
                offset += 4
                peak_msg.peak_container.peaks[ ch ].append( peak_val )

            # for
        # for

        return peak_msg

    # getData

    def getSpectrum( self, channels: Union[ List[ int ], int ] = None ):
        """
        Get the spectrum of the interrogator for selected Channels
            :param channels: (Default = [1,2,3,4]) integer or list of integers of channel(s) to get spectrum for

            :return: SpectrumMessage filled in with the specified channels
        """
        # argument parsing
        if channels is None:
            channels = [ 1, 2, 3, 4 ]  # all channels default

        elif isinstance( channels, int ):
            channels = [ channels ]

        spectrum_msg = SpectrumMessage()

        # get spectrum setup
        dt = np.dtype( 'int16' )
        dt.newbyteorder( '<' )

        # iterate through channels
        for ch in channels:
            # send command for setting amplitude gathering channel
            response = self.sendCommand( f"#SET_AMP_CH {ch}" ).decode( 'utf-8' )
            if '!Invalid command or argument' in response:  # check if command is valid
                continue

            # if
            data = self.sendCommand( "#GET_SPECTRUM" )
            spectrum_msg.header = self.parseHeader( data )  # update header for latest results

            wavelengths = Spectrum.wavelengthsFromStartEnd(
                    spectrum_msg.header.startWavelength,
                    spectrum_msg.header.endWavelength,
                    spectrum_msg.SPECTRUM_ELEMENT_COUNT )

            spectrum_msg.spectrum_container.spectrum[ ch - 1 ].wavelengths = wavelengths
            spectrum_msg.spectrum_container.spectrum[ ch - 1 ].amplitudes = np.frombuffer(
                    data, offset=spectrum_msg.header.HEADER_SIZE, dtype=dt ).tolist()

        # for

        return spectrum_msg

    # getSpectrum

    @staticmethod
    def parseHeader( data: bytes ):
        header = StatusHeader()
        header.fullSpectrimRadix = int.from_bytes( data[ 0:8 ], 'little' )
        header.serialNumber = int.from_bytes( data[ 7 * 4:7 * 4 + 4 ], 'little' )

        header.granularity = int.from_bytes( data[ 18 * 4:18 * 4 + 4 ], 'little' )

        header.numCH1Sensors = int.from_bytes( data[ 4 * 4:4 * 4 + 2 ], 'little' )
        header.numCH2Sensors = int.from_bytes( data[ 4 * 4 + 2:4 * 4 + 4 ], 'little' )
        header.numCH3Sensors = int.from_bytes( data[ 5 * 4:5 * 4 + 2 ], 'little' )
        header.numCH4Sensors = int.from_bytes( data[ 5 * 4 + 2:5 * 4 + 4 ], 'little' )

        header.startWavelength = int.from_bytes(
                data[ 20 * 4:20 * 4 + 4 ], 'little' ) / header.granularity
        header.endWavelength = int.from_bytes(
                data[ 21 * 4:21 * 4 + 4 ], 'little' ) / header.granularity

        header.timeStamp = int.from_bytes( data[ 9 * 4:9 * 4 + 4 ], 'little' ) + int.from_bytes(
                data[ 8 * 4:8 * 4 + 4 ], 'little' ) / 1e6

        header.bufferSize = data[ 12 * 4 ]
        header.headerSize = int.from_bytes( data[ 12 * 4 + 2:12 * 4 + 4 ], 'little' )
        header.headerVersion = data[ 12 * 4 + 1 ]

        return header

    # parseHeader

    def sendCommand( self, command: str ):
        """ Command parsing for interrogator interface

            :param command: string of the command for the interrogator

            :returns: byte string of response

        """

        if not command.endswith( "\n" ):
            command += '\n'

        self.sock.send( command.encode( 'utf-8' ) )

        msg_size = int( self.sock.recv( 10 ) )  # get size of incoming payload
        msg = b""
        while len( msg ) < msg_size:
            msg += self.sock.recv( msg_size )

        return msg

    # sendCommand


# class: Interrogator

# ==================================================================================================
# =========================== HELPER FUNCTIONS =====================================================
# ==================================================================================================

def printBytes( msg ):
    print( " 0: ", end='' )
    for i in range( len( msg ) ):
        print( "{:3d}".format( msg[ i ] ), end=' ' )
        if (i + 1) % 4 == 0:
            print( '\n{:2d}: '.format( (i + 1) // 4 ), end='' )
    # if
    # for
    print()


# printBytes

def printHex( msg ):
    for i in range( 0, len( msg ), 4 ):
        print( "{:2d}: {}".format( i // 4, msg[ i:i + 4 ] ) )


# for

# printHex

# ===================================================================================================
# =========================== MAIN METHODS ==========================================================
# ===================================================================================================

def interrogator_cli():
    # address information
    address = "192.168.1.11"
    port = 1852

    # create socket
    with socket.socket( socket.AF_INET, socket.SOCK_STREAM ) as sock:
        # connect
        print( f"Connecting to {address}:{port}" )
        sock.connect( (address, port) )
        print( f"Connected to {address}:{port}" )

        # send the IDN command
        sock.send( "#IDN?\n".encode( "utf-8" ) )
        print( "Sent command" )

        msg_size = int( sock.recv( 10 ) )
        msg = sock.recv( msg_size )
        print( "Received:", msg )
        print()

        while True:
            cmd = input( "Input Command: " )
            sock.send( (cmd + '\n').encode( 'utf-8' ) )
            msg_size_b = sock.recv( 10 )
            msg_size = int( msg_size_b.decode( 'utf-8' ) )
            msg = b""
            while len( msg ) < msg_size:
                msg += sock.recv( msg_size )

            print( f"Received msg of size {msg_size}-{len( msg )} | {msg_size_b} \n {msg}\n" )
            print( "msg_size bytes:" )
            printBytes( msg_size_b )
            print( "msg bytes:" )
            printBytes( msg )
            print( "Both" )
            printBytes( msg_size_b + msg )
            if cmd.upper().startswith( "#GET_DATA" ) or cmd.upper().startswith(
                    "#GET_UNBUFFERED_DATA" ):
                header = Interrogator.parseHeader( msg )
                print( f"Received header: {header}" )
                peaks = interrogator.getData()
                print( f"Peaks: {peaks}" )

            # if
        # while
    # with: socket


# interrogator_cli

def main():
    # address information
    address = "192.168.1.11"
    port = 1852

    # create interrogator
    interrogator = Interrogator( address, port )

    # getting peak message data
    peak_msg = interrogator.getData()
    print( "Received peak msg:", peak_msg )
    print( "Status Header: ", peak_msg.header )
    print( "Peak Container:", peak_msg.peak_container )
    print()

    # get spectrum message data
    spectrum_msg = interrogator.getSpectrum()
    print( "Received spectrum msg:" )
    print( "Status Header:      ", spectrum_msg.header )
    # print( "Spectrum Container: ", spectrum_msg.spectrum_container )
    # plt.plot(
    #         spectrum_msg.spectrum_container.CH1.wavelengths,
    #         spectrum_msg.spectrum_container.CH1.amplitudes, label='CH1' )
    # plt.plot(
    #         spectrum_msg.spectrum_container.CH2.wavelengths,
    #         spectrum_msg.spectrum_container.CH2.amplitudes, label='CH2' )
    # plt.plot(
    #         spectrum_msg.spectrum_container.CH3.wavelengths,
    #         spectrum_msg.spectrum_container.CH3.amplitudes, label='CH3' )
    # plt.plot(
    #         spectrum_msg.spectrum_container.CH4.wavelengths,
    #         spectrum_msg.spectrum_container.CH4.amplitudes, label='CH4' )
    # plt.legend()
    #
    # plt.show()
    print()


# main

if __name__ == "__main__":
    main()

# if __main__
