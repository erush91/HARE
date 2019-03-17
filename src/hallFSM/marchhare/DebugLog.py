#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Template Version: 2016-07-08

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

"""
DebugLog.py
James Watson, 2016 July
Module for debugging and logging , slightly better than straight printing
"""

"""
  == TODO ==
* Implement printing to file
"""

try:
    foo = endl
except NameError:
    import os
    endl = os.linesep

# == Session Logging ==

dbgStr = '' # A place to put debug and status information for this session, cleared at start of session when file is run

DEBUGLEVEL = 0
# ~ Convention ~
# -1 in the file, debug no longer needed
# Level should be positive nonzero for all debug groups
# 0 is not to be used in the files, it is the default level and should not correspond to any debugging

def set_dbg_lvl(level):
    """ Set the 'DEBUGLEVEL' for this session, must be an integer """
    global DEBUGLEVEL # Apparently you cannot access global variables from another module?
    DEBUGLEVEL = int(level)
    
def get_dbg_lvl():
    """ Return the 'DEBUGLEVEL' for this session """
    return DEBUGLEVEL
    
def dbgLog(lvl, *dbgMsg):
    """ Write status info to the debug log and print if desired """
    global dbgStr
    if lvl == DEBUGLEVEL:
        temp = ''
        for msg in dbgMsg:
            temp += str(msg) + ' ' # Auto-insert spaces between args just like 'print <OUTPUT>,'
        temp += endl # Otherwise other output might be confusingly appended to the line
        dbgStr += temp 
        print temp

def dbg_data_exists():
    """ Return True if debug data exists, otherwise false """
    return True if dbgStr else False
    
def dbg_contents():
    """ Return the contents of the debug string """
    return dbgStr
    
def dbgClr():
    """ Clear status info for this session """
    global dbgStr
    dbgStr = '' # Set debug string to an empty string
    
def dbg_to_file( path ):
    """ Write the entire 'dbgStr' to a file at 'path' """
    f = file(path , 'w')
    f.write( dbgStr )
    f.close()
    
# == End Logging ==
    
    
# == Benchmark Stopwatch ==

#from collections import namedtuple
#Interval = namedtuple('Point', ['start', 'stop'])

class Interval:
    """ Container for start and stop times """
    def __init__(self):
        self.start = None
        self.stop = None

class Stopwatch(object):
    """ Singleton object for benchmarking, run as many clocks as you like """
    strtTime = 0
    stopTime = 0
    clockHash = {}
    @staticmethod
    def start(name = ''):
        if not name:
            Stopwatch.strtTime = timer()
        else:
            if not name in Stopwatch.clockHash:
                Stopwatch.clockHash[name] = Interval()
            Stopwatch.clockHash[name].start = timer()
    @staticmethod
    def stop(name = ''):
        if not name:
            Stopwatch.stopTime = timer()
        else:
            if not name in Stopwatch.clockHash:
                Stopwatch.clockHash[name] = Interval()
            Stopwatch.clockHash[name].stop = timer()            
    @staticmethod
    def elapsed(name = ''):
        if not name:
            return Stopwatch.stopTime - Stopwatch.strtTime
        else:
            if not name in Stopwatch.clockHash:
                raise KeyError('Stopwatch.elapsed: There is no timer with that name!')
            else:
                return Stopwatch.clockHash[name].stop - Stopwatch.clockHash[name].start
    
# == End Stopwatch ==