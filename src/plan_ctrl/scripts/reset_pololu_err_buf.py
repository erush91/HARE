#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

import serial # pySerial for USB comms w/ Pololu
from serial.tools import list_ports
import struct # for data composition over serial
import time

def attempt_connection( COMlist , speed ):
    """ Attempt to connect to each of 'COMlist' until one succeeds """
    ser = None
    for port in COMlist:
        try:
            ser = serial.Serial( port , speed ) # This must match the port selected in the Arduino IDE
            print "Connected to port:" , ser.name 
            return ser
        except Exception as ex:
            print "ERROR: COULD NOT ESTABLISH SERIAL CONNECTION WITH" , port , ", Check that the port is correct ..."
            print ex
    raise Warning( "attempt_connection: Unable to connect to any of " + str( COMlist ) ) # If we made it to here , we attempted all the ports

def send_pololu_error_clear( connection ):
    """ Spam the Maestro with error buffer requests """
    # https://www.pololu.com/docs/0J40/5.e
    # Compact Protocol: This is the simpler and more compact of the two protocols; 
    #                   it is the protocol you should use if your Maestro is the only device connected to your serial line.
    compact = [ 162 ] # ----------- Compact protocol: 0xA2 # int('a2',16) = 162
    polProt = [ 170 ,   0 ,  34 ] # Pololu protocol: 0xAA, device number, 0x22    
    msg     = bytearray( compact )
    repeat  = 5
    for i in xrange( repeat ):
        connection.write( msg )
        time.sleep( 0.001 )
    print "Spammed serial connection with byte message" , msg , "," , repeat , "times!"
        
def clear_pololu_err_buf():
    """ Connect w/ Maestro, Spam error clear, Close connection """
    COMLIST = [ '/dev/pololu' ]
    COMBAUD = 115200
    pol_ser = attempt_connection( COMLIST , COMBAUD )
    if pol_ser:
        print "Connection open!"
    try:
        send_pololu_error_clear( pol_ser )
        if pol_ser.in_waiting:
            readResult = pol_ser.read( pol_ser.in_waiting )
            print "Error Bytes:" , readResult                 
        else:
            print "There were no error bytes to read ..."
        pol_ser.close()
        print "Connection closed!"
    except Exception as ex:
        print "Pololu error clear operation failed for the following reason:"
        print ex
        

if __name__ == "__main__":
    print __prog_signature__()
    termArgs = sys.argv[1:] # Terminal arguments , if they exist
    print "About to bother the pololu ...."
    clear_pololu_err_buf()
    print "... I hope that fixed it. Bye!"