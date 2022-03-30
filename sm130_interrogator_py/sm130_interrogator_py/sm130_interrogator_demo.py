# standard libraries
from collections import namedtuple
from typing import Optional, Dict

# standard ROS libraries
import rclpy
from rclpy.exceptions import ParameterAlreadyDeclaredException

# 3rd party libraries
import numpy as np

# custom imports
from .sm130_interrogator import FBGInterrogatorNode

# demo interrogator named tuple
DemoInterrogator = namedtuple( 'DemoInterrogator', [ 'is_ready' ] )


class FBGInterrogatorDemo( FBGInterrogatorNode ):
    # PARAMETER NAMES
    param_names = FBGInterrogatorNode.param_names
    param_names[ 'num_chs' ] = 'demo.num_channels'
    param_names[ 'num_aas' ] = 'demo.num_active_areas'

    def __init__( self, name='FBGInterrogatorDemoNode', num_chs=3, num_aas=4 ):
        # fields
        self.base_wavelengths = { }
        self.num_chs = num_chs
        self.num_aas = num_aas

        super().__init__( name )

    # __init__

    def connect( self ) -> bool:
        """ Connect to interrogator (TO BE OVERRIDDEN)

            :return: True if connection is established, False if not.
        """
        # parameters
        try:
            self.num_chs = self.declare_parameter( self.param_names[ 'num_chs' ],
                                                   self.num_chs ).get_parameter_value().integer_value
        except ParameterAlreadyDeclaredException:
            self.num_chs = self.get_parameter( self.param_names[ 'num_chs' ] ).get_parameter_value().integer_value

        try:
            self.num_aas = self.declare_parameter( self.param_names[ 'num_aas' ],
                                                   self.num_aas ).get_parameter_value().integer_value
        except ParameterAlreadyDeclaredException:
            self.num_aas = self.get_parameter( self.param_names[ 'num_aas' ] ).get_parameter_value().integer_value

        # connect the demo interrogator
        if (self.num_chs > 0) and (self.num_aas > 0):
            # configure demo interrogator
            self.is_connected = True
            self.get_logger().info( "Connected to demo sm130 interrogator" )
            self.get_logger().info( "Connected to demo interrogator at: {}".format( self.ip_address ) )

            # initialize reference wavelengths
            for ch in range( 1, self.num_chs + 1 ):
                if ch in range( 1, self.num_chs + 1 ):
                    self.base_wavelengths[ ch ] = (1540 + 10 * np.arange( self.num_aas, dtype=np.float64 ) + ch)
                else:
                    self.base_wavelengths[ ch ] = np.array( [ ] )

            # for
            self.ref_wavelengths = { }  # empty ref_wavelengths just in case there is a parameter change
            self.get_logger().warning( "Please calibrate the sensors." )

        # if

        else:  # not connected
            self.is_connected = False
            self.get_logger().error( "Did not connect to demo sm130 interrogator! # Channels and # AAs must be > 0." )

        # else

        self.interrogator = DemoInterrogator( self.is_connected )

        self.get_logger().debug( f"Current (demo) peaks are: {self.base_wavelengths}" )
        self.get_logger().debug( f"Current (demo) peaks are: {self.get_peaks()}" )

        return self.is_connected

    # connect

    def get_peaks( self ):
        """ Get the demo FBG peak values

            TO BE OVERRIDDEN IN SUBCLASSES

            :returns: dict of demo peaks if connected, otherwise, None

        """
        # check if connected
        if not self.is_connected:
            return None

        # peak_msg = PeakMessage()
        #
        # for ch, peaks in self.base_wavelengths.items():
        #     if ch == 1:
        #         peak_msg.header.numCH1Sensors = len( peaks )
        #         peak_msg.peak_container.CH1 = peaks.tolist()
        #
        #     elif ch == 2:
        #         peak_msg.header.numCH2Sensors = len( peaks )
        #         peak_msg.peak_container.CH2 = peaks.tolist()
        #
        #     elif ch == 3:
        #         peak_msg.header.numCH3Sensors = len( peaks )
        #         peak_msg.peak_container.CH3 = peaks.tolist()
        #
        #     elif ch == 4:
        #         peak_msg.header.numCH4Sensors = len( peaks )
        #         peak_msg.peak_container.CH4 = peaks.tolist()
        #
        #     else:
        #         continue
        #
        # # for

        return self.base_wavelengths

    # get_peaks

    def parse_peaks( self, peak_data: dict ) -> Optional[ Dict[int, np.ndarray] ]:
        """ Parse the peak data into a dict"""
        if peak_data is None:
            return None

        data = { }
        for ch in range( 1, self.num_chs + 1 ):
            data[ ch ] = np.array( peak_data[ ch ], dtype=np.float64 )

        # for


        return data

    # parse_peaks


# class: FBGInterrogatorDemo

def main( args=None ):
    rclpy.init( args=args )

    demo_node = FBGInterrogatorDemo()

    rclpy.spin( demo_node )

    rclpy.shutdown()


# main


if __name__ == '__main__':
    main()

# if __main__
