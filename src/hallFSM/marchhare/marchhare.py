#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

"""
marchhare.py , Built on Wing IDE 101 for Python 2.7
James Watson, 2016 October
Module ARCHive for a Hobby and Research Environment
Helper functions
"""

# == Init ==================================================================================================================================

# ~~ Libraries ~~
# ~ Standard Libraries ~
import sys , os , datetime , cPickle , heapq , time , operator
from math import trunc , pi  
from random import choice
from warnings import warn
import numpy as np
## ~ Cleanup ~
#plt.close('all') # clear any figures we may have created before 

# ~~ Constants , Shortcuts , Aliases ~~
EPSILON = 1e-7 # Assume floating point errors below this level
infty = 1e309 # URL: http://stackoverflow.com/questions/1628026/python-infinity-any-caveats#comment31860436_1628026
endl = os.linesep # Line separator
pyEq = operator.eq # Default python equality
piHalf = pi/2

# __ End Init ______________________________________________________________________________________________________________________________


# == PATH Manipulation ==

def find_in_path( term , strOut = False ):
    """ Search for a term in loaded 'sys.path' and print all the matching entries """
    term = str( term ).lower()
    rtnStr = ""
    for entry in sys.path:
        if term in entry:
            rtnStr += entry + endl
    if strOut:
        return rtnStr
    else:
        print rtnStr

def find_in_loaded( term , strOut = False ):
    """ Search for a term in loaded modules and print all the matching entries """
    term = str( term ).lower()
    rtnStr = ""
    for key,val in sys.modules.iteritems():
        if term in str( key ).lower() or term in str( val ).lower():
            rtnStr += "{: >60} {: >60} ".format( key , val ) + endl
    if strOut:
        return rtnStr
    else:
        print rtnStr

def add_container_to_path( fName ): 
    """ Add the directory that contains 'fName' to Python path if it is not already there """
    containerDir = os.path.dirname( fName )
    if containerDir not in sys.path:
        sys.path.append( containerDir )

def first_valid_dir( dirList ):
    """ Return the first valid directory in 'dirList', otherwise return False if no valid directories exist in the list """
    rtnDir = False
    for drctry in dirList:
        if os.path.exists( drctry ):
            rtnDir = drctry 
            break
    return rtnDir

def add_first_valid_dir_to_path( dirList ):
    """ Add the first valid directory in 'dirList' to the system path """
    # In lieu of actually installing the library, just keep a list of all the places it could be in each environment
    validDir = first_valid_dir(dirList)
    print __file__ , "is attempting to load a path ...",
    if validDir:
        if validDir in sys.path:
            print "Already in sys.path:", validDir
        else:
            sys.path.append( validDir )
            print 'Loaded:', str( validDir )
    else:
        raise ImportError("None of the specified directories were loaded") # Assume that not having this loaded is a bad thing

# __ End PATH __


# == Helper Functions ==

def sep( title = "" , width = 6 , char = '=' , strOut = False ): 
    """ Print a separating title card for debug """
    LINE = width * char
    if strOut:
        return LINE + ' ' + title + ' ' + LINE
    else:
        print LINE + ' ' + title + ' ' + LINE

# __ End Helper __



# == Time Reporting and Formatting ==

def elapsed_w_msg( msg = "Elapsed:" ): # >>> resenv
    """ Return the time elapsed and print with a msg """
    if elapsed_w_msg.lastTime == None:
        print msg , "Timer started!"
        elapsed_w_msg.lastTime = time.time()
        return 0
    else:
        seconds = elapsed = time.time() - elapsed_w_msg.lastTime
        hours = int( elapsed / elapsed_w_msg.s_in_hr )
        elapsed = elapsed % elapsed_w_msg.s_in_hr
        mins = int( elapsed / elapsed_w_msg.s_in_mn )
        elapsed = elapsed % elapsed_w_msg.s_in_mn 
        print msg , hours , "hrs" , mins , "min" , elapsed , "sec"
        elapsed_w_msg.lastTime = time.time()
        return seconds
elapsed_w_msg.lastTime = None
elapsed_w_msg.s_in_hr = 60 * 60
elapsed_w_msg.s_in_mn = 60

nowTimeStamp = lambda: datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S') # http://stackoverflow.com/a/5215012/893511
""" Return a formatted timestamp string, useful for logging and debugging """

nowTimeStampFine = lambda: datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S-%f') # http://stackoverflow.com/a/5215012/893511
""" Return a formatted timestamp string, useful for logging and debugging """

def format_epoch_timestamp( sysTime ):
    """ Format epoch time into a readable timestamp """
    return datetime.datetime.fromtimestamp( sysTime ).strftime('%Y-%m-%d_%H-%M-%S-%f')

class Timer: 
    """ The simplest timer possible """
    startTime = 0 # init timer to zero so that 'Timer.elapsed' does not fail if 'Timer.begin' is not called first 

    @staticmethod
    def begin():
        """ Begin / reset the timer and return the start time """
        Timer.startTime = time.time()
        return Timer.startTime

    @staticmethod
    def elapsed():
        """ Return the number of seconds that have passed since the last time 'Timer.begin' was called """
        return time.time() - Timer.startTime

def tick_progress( div = 1000 , reset = False ):
    """ Print the 'marker' every 'div' calls """
    if reset:
        tick_progress.totalCalls = 0
    else:
        tick_progress.totalCalls += 1
        if tick_progress.totalCalls % div == 0:
            tick_progress.ticks += 1
            print tick_progress.sequence[ tick_progress.ticks % ( len( tick_progress.sequence ) ) ] ,
tick_progress.totalCalls = 0
tick_progress.sequence = [ "'" , "-" , "," , "_" , "," , "-" , "'" , "`" , "`" ] # This makes quite a pleasant wave
tick_progress.ticks = 0

# __ End Time __


# == Data Structures , Special Lists , and Iterable Operations ==

def elemw( iterable , i ): 
    """ Return the 'i'th index of 'iterable', wrapping to index 0 at all integer multiples of 'len(iterable)' , Wraps forward and backwards """
    seqLen = len( iterable )
    if i >= 0:
        return iterable[ i % ( seqLen ) ]
    else:
        revDex = abs( i ) % ( seqLen )
        if revDex == 0:
            return iterable[ 0 ]
        return iterable[ seqLen - revDex ]

def indexw( iterable , i ): 
    """ Return the 'i'th index of 'iterable', wrapping to index 0 at all integer multiples of 'len(iterable)' """
    seqLen = len( iterable )
    if i >= 0:
        return i % ( seqLen )
    else:
        revDex = abs( i ) % ( seqLen )
        if revDex == 0:
            return 0
        return seqLen - revDex

def same_contents_list( lst1 , lst2 ):
    """ Determine if every element in 'lst1' can be found in 'lst2' , and vice-versa , NOTE: This function assumes all elements are hashable """
    s1 = set( lst1 ) ; s2 = set( lst2 )
    return s1 == s2

def lst( *args ):
    """ Return a list composed of the arbitrary 'args' """
    return list(args)

def tpl( *args ):
    """ Return a tuple composed of the arbitrary 'args' """
    return tuple( args ) 

def prepend( pList , item ):
    """ Prepend an item to the front of the list """
    if type( pList ) == list:
        pList.insert( 0 , item )
    elif type( pList ) == str:
        pList = str( item ) + pList
    return pList
    
def prepad( pList , item , totLen ):
    """ Pad the front of 'pList' with copies of 'item' until it has at least 'totLen' items , Return a reference to the list """
    prmLen = len( pList )
    while len( pList ) < totLen:
        pList = prepend( pList , item )
    return pList

def sort_list_to_tuple( pList ):
    """ Return a tuple that contains the sorted elements of 'pList' """
    return tuple( sorted( pList ) )

def sort_tuple( tup ):
    """ Return a sorted copy of 'tup' """
    return sort_list_to_tuple( list( tup ) ) 

def incr_min_step( bgn , end , stepSize ):
    """ Return a list of numbers from 'bgn' to 'end' (inclusive), separated by at LEAST 'stepSize'  """
    # NOTE: The actual step size will be the size that produces an evenly-spaced list of trunc( (end - bgn) / stepSize ) elements
    return np.linspace( bgn , end , num = trunc( (end - bgn) / stepSize ) , endpoint=True )

def incr_max_step( bgn , end , stepSize ):
    """ Return a list of numbers from 'bgn' to 'end' (inclusive), separated by at MOST 'stepSize'  """
    numSteps = ( end - bgn ) / ( stepSize * 1.0 )
    rtnLst = [ bgn + i * stepSize for i in xrange( trunc(numSteps) + 1 ) ]
    if numSteps % 1 > 0: # If there is less than a full 'stepSize' between the last element and the end
        rtnLst.append( end )
    return rtnLst

def assoc_lists( keys , values ):
    """ Return a dictionary with associated 'keys' and 'values' """
    return dict( zip( keys , values ) )

def assoc_sort_tuples( keyList , valList ):
    """ Associate each element of 'keyList' with each element of 'valList' , sort on 'keyList' , return list of tuples """
    return [ elem for elem in sorted( zip( keyList , valList ) , key = lambda pair: pair[0] ) ]

def tandem_sorted( keyList , *otherLists ): 
    """ Sort multiple lists of equal length in tandem , with the elements of each in 'otherLists' reordered to correspond with a sorted 'keyList' """
    # URL , Sort two lists in tandem: http://stackoverflow.com/a/6618543/893511
    bundle = sorted( zip( keyList , *otherLists ) , key = lambda elem: elem[0] ) # Sort the tuples by the key element
    # print "DEBUG , zipped lists:" , bundle
    rtnLists = [ [] for i in xrange( len( bundle[0] ) ) ]
    for elemTuple in bundle:
        for index , elem in enumerate( elemTuple ):
            rtnLists[ index ].append( elem ) # Send the element to the appropriate list
    return rtnLists

def tandem_sorted_reverse( keyList , *otherLists ): 
    """ Sort multiple lists of equal length in tandem , with the elements of each in 'otherLists' reordered to correspond with a sorted 'keyList' """
    # URL , Sort two lists in tandem: http://stackoverflow.com/a/6618543/893511
    bundle = sorted( zip( keyList , *otherLists ) , key = lambda elem: elem[0] , reverse = True ) # Sort the tuples by the key element
    # print "DEBUG , zipped lists:" , bundle
    rtnLists = [ [] for i in xrange( len( bundle[0] ) ) ]
    for elemTuple in bundle:
        for index , elem in enumerate( elemTuple ):
            rtnLists[ index ].append( elem ) # Send the element to the appropriate list
    return rtnLists

def index_max( pList ):
    """ Return the first index of 'pList' with the maximum numeric value """
    return pList.index( max( pList ) )

def index_min( pList ): 
    """ Return the first index of 'pList' with the maximum numeric value """
    return pList.index( min( pList ) )

def linspace_space( dim , sMin , sMax , num  ): 
    """ Return vector list covering a 'dim'-dimensional space with 'num' points in each dimension, from 'sMin' to 'sMax' in each dimension , O(n^2) """
    rtnLst = [] # List of vectors to return
    currDim = np.linspace( sMin , sMax , num ) # Get evenly-spaced points in this dimension
    if dim == 1: # Base case, this is the last dimension in the vector
        return [ [item] for item in currDim ]
    elif dim > 1: # Recursive case , for every combination of the first m dimensions, create every combination of the remaining m-n dimensions
        for item in currDim:
            for sub in linspace_space( dim-1 , sMin , sMax , num  ):
                rtnLst.append( [item] + sub )
    return rtnLst

def linspace_centers( sMin , sMax , num ):
    """ Return the centers of 'num' bins from 'sMin' to 'sMax' """
    borders = np.linspace( sMin , sMax , num + 1 ) # Get evenly-spaced points
    centers = []
    for i in xrange( num ):
        centers.append( ( borders[i] + borders[i+1] ) / 2.0 )
    return centers

def find_pop( iterable , item ):
    """ Pop 'item' from 'iterable' , ValueError if not in 'iterable' """
    return iterable.pop( iterable.index( item ) )

def find_pop_safe( iterable , item ):
    """ Pop 'item' from 'iterable' , Return 'None' if not in 'iterable' """
    try:
        return find_pop( iterable , item )
    except:
        return None

def insert_sublist( bigList , index , subList ): 
    """ Insert 'subList' into 'bigList' at 'index' and return resulting list """
    return bigList[ :index ] + subList + bigList[ index: ]

def replace_sublist( bigList , begDex , endDex , subList ): 
    """ Replace the elements in 'bigList' from 'begDex' to 'endDex' with 'subList' """
    return bigList[ :begDex ] + subList + bigList[ endDex: ]

def iter_contains_None( listOrTuple ):
    """ Return True if any of 'listOrTuple' is None or contains None , Otherwise return False """
    if isinstance( listOrTuple , ( list , tuple ) ): # Recursive Case: Arg is an iterable , inspect each
        for elem in listOrTuple:
            if iter_contains_None( elem ):
                return True
        return False
    else: # Base Case: Arg is single value
        return True if listOrTuple == None else False

def num_range_to_bins( minNum , maxNum , divs  ):
    """ Return a list of tuples that each represent the bounds of one of 'divs' bins from 'minNum' to 'maxNum' """
    borders = np.linspace( minNum , maxNum , divs + 1 )
    bins = []
    for bDex , border in enumerate( borders[1:] ):
        bins.append( ( borders[bDex-1] , borders[bDex] ) )
    return bins

def bindex( bins , val ):
    """ Given a list of ( lower_i , upper_i ) 'bins' , Return the index that 'val' belongs in , Return None if there is no such bin """
    # NOTE: This function assumes 'bins' takes the form of returned by 'num_range_to_bins'
    for bDex , bin in enumerate( bins ):
        if val > bin[0] and val <= bin[1]:
            return bDex
    return None

def concat_arr( *arrays ):
    """ Concatenate all 'arrays' , any of which can be either a Python list or a Numpy array """
    # URL , Test if any in an iterable belongs to a certain class : https://stackoverflow.com/a/16705879
    if any( isinstance( arr , np.ndarray ) for arr in arrays ): # If any of the 'arrays' are Numpy , work for all cases , 
        if len( arrays ) == 2: # Base case 1 , simple concat    # but always returns np.ndarray
            return np.concatenate( ( arrays[0] , arrays[1] ) )
        elif len( arrays ) > 2: # If there are more than 2 , concat the first two and recur
            return concat_arr( 
                np.concatenate( ( arrays[0] , arrays[1] ) ) , 
                *arrays[2:] 
            )
        else: # Base case 2 , there is only one arg , return it
            return arrays[0]
    if len( arrays ) > 1: # else no 'arrays' are Numpy 
        rtnArr = arrays[0]
        for arr in arrays[1:]: # If there are more than one , just use addition operator in a line
            rtnArr += arr
        return rtnArr
    else: # else there was only one , return it
        return arrays[0] 

def build_sublists_by_cadence( flatList , cadence ): 
    """ Return a list in which each element is a list of consecutive 'flatList' elements of length 'cadence' elements if elements remain """
    rtnList = []
    for flatDex , flatElem in enumerate( flatList ):
        if flatDex % cadence == 0:
            rtnList.append( [] )
        rtnList[-1].append( flatElem )
    return rtnList

def flatten_nested_sequence( multiSeq ):
    """ Flatten a sequence of sequences ( list or tuple ) into a single , flat sequence of the same type as 'multiSeq' """
    masterList = []
    # print multiSeq
    if isinstance( multiSeq , np.ndarray ):
        # print "Converting to nested list"
        multiSeq = multiSeq.tolist()
        # return
    def flatten_recur( multLst , masterList ):
        """ Does the recursive work of 'flatten_nested_lists' """
        for elem in multLst:
            if isinstance( elem , list ):
                flatten_recur( elem , masterList )
            elif isinstance( elem , np.ndarray ):
                # print "Element:" , elem
                # print "To list:" , elem.tolist()
                flatten_recur( elem.tolist() , masterList )
            else:
                masterList.append( elem )
    flatten_recur( multiSeq , masterList )
    if isinstance( multiSeq , tuple ): # If the original nested sequence was a tuple, then make sure to return a tuple
        return tuple( masterList )
    else:
        # print "Got a list containing" , len( multiSeq ) , "elemets"
        # print "Returning a list of  " , len( masterList ) , "elements"
        return masterList

def double_all_elem_except( inList , exceptedIndices = [] ):
    """ Double all elements of a list except those indicated by 'exceptedIndices' """
    rtnList = []
    for i , elem in enumerate( inList ):
        if i in exceptedIndices:
            rtnList.append( elem )
        else:
            rtnList.extend( [ elem , elem ] )
    return rtnList    

# = Containers for Algorithms =

class Stack(list): 
    """ LIFO container based on 'list' """    

    def __init__( self , *args ):
        """ Normal 'list' init """
        list.__init__( self , *args )

    def push( self , elem ):
        """ Push 'elem' onto the Stack """
        self.append( elem )

    # 'Stack.pop' is inherited from 'list'

    def is_empty(self):
        """ Returns true if the Stack has no elements """
        return len(self) == 0

class Queue(list): 
    """ FIFO container based on 'list' """ 

    def __init__( self , *args ):
        """ Normal 'list' init """
        list.__init__( self , *args )

    def push( self , elem ):
        """ Push an item (prepend) to the back of the Queue """
        self.insert( 0 , elem )

    # 'Queue.pop' is inherited from 'list'

    def is_empty(self):
        """ Returns true if the Queue has no elements """
        return len(self) == 0

    def item_list( self ):
        """ Return a copy of the Queue as a list """
        return self[:]

class PriorityQueue( list ): # Requires heapq 
    """ Implements a priority queue data structure. """ 
    # NOTE: PriorityQueue does not allow you to change the priority of an item. 
    #       You may insert the same item multiple times with different priorities. 

    def __init__( self , *args ):
        """ Normal 'list' init """
        list.__init__( self , *args )   
        self.count = 0
        self.s = set([])    

    def push( self , item , priority , hashable = None ):
        """ Push an item on the queue and automatically order by priority , optionally provide 'hashable' version of item for set testing """
        entry = ( priority , self.count , item )
        heapq.heappush( self , entry )
        self.count += 1
        if hashable:
            self.s.add( hashable ) 

    def contains( self , hashable ): 
        ''' Test if 'node' is in the queue '''
        return hashable in self.s

    def pop( self ):
        """ Pop the lowest-priority item from the queue """
        priority , count , item = heapq.heappop( self )
        return item

    def pop_with_priority( self ):
        """ Pop the item and the priority associated with it """
        priority , count , item = heapq.heappop( self )
        return item , priority

    def pop_opposite( self ):
        """ Remove the item with the longest priority , opoosite of the usual pop """
        priority , count , item = self[-1]
        del self[-1]
        return item

    def isEmpty(self):
        """ Return True if the queue has no items, otherwise return False """
        return len( self ) == 0

    # __len__ is provided by 'list'

    def unspool( self , N = infty , limit = infty ):
        """ Pop all items as two sorted lists, one of increasing priorities and the other of the corresponding items """
        vals = []
        itms = []
        count = 0
        while not self.isEmpty() and count < N and self.top_priority() <= limit:
            item , valu = self.pop_with_priority()
            vals.append( valu )
            itms.append( item )
            count += 1
        return itms , vals

    def peek( self ):
        """ Return the top priority item without popping it """
        priority , count , item = self[0]
        return item

    def peek_opposite( self ):
        """ Return the bottom priority item without popping it """
        priority , count , item = self[-1]
        return item

    def top_priority( self ):
        """ Return the value of the top priority """
        return self[0][0]

    def btm_priority( self ):
        """ Return the value of the bottom priority """
        return self[-1][0]

    def get_priority_and_index( self , item , eqFunc = pyEq ):
        """ Return the priority for 'item' and the index it was found at , using the secified 'eqFunc' , otherwise return None if 'item' DNE """
        for index , elem in enumerate( self ): # Implement a linear search
            if eqFunc( elem[-1] , item ): # Check the contents of each tuple for equality
                return elem[0] , index # Return priority if a match
        return None , None # else search completed without match , return None

    def reprioritize_at_index( self , index , priority ):
        """ Replace the priority of the element at 'index' with 'priority' """
        temp = list.pop( self , index ) # Remove the item at the former priority
        self.push( temp[-1] , priority ) # Push with new priority , this item should have the same hashable lookup

class BPQ( PriorityQueue ):
    """ Bounded Priority Queue , does not keep more than N items in the queue """

    def __init__( self , boundN , *args ):
        """ Create a priority queue with a specified bound """
        PriorityQueue.__init__( self , *args )
        self.bound = boundN

    def push( self , item , priority , hashable=None ):
        """ Push an item onto the queue and discard largest priority items that are out of bounds """
        PriorityQueue.push( self , item , priority , hashable ) # The usual push
        while len( self ) > self.bound: # If we exceeded the bounds , then discard down to the limit
            self.pop_opposite()

class LPQ( PriorityQueue ):
    """ Limited Priority Queue , does not keep items with priorities longer than 'limit' """

    def __init__( self , limitR , *args ):
        """ Create a priority queue with a specified bound """
        PriorityQueue.__init__( self , *args )
        self.limit = limitR

    def push( self , item , priority , hashable=None ):
        """ Push an item onto the queue and discard largest priority items that are out of bounds """
        if priority <= self.limit:
            PriorityQueue.push( self , item , priority , hashable ) # The usual push

class PQwR(list): # Requires heapq # TODO: UPDATE ASMENV
    """ Implements a priority queue data structure, replaces items with identical priorituies """ 
    # NOTE: PriorityQueue does not allow you to change the priority of an item. 
    #       You may insert the same item multiple times with different priorities. 

    def __init__( self , *args ):
        """ Normal 'list' init """
        list.__init__( self , *args )   
        self.count = 0
        self.s = set([])    

    def push( self , item , priority , hashable=None ):
        """ Push an item on the queue and automatically order by priority , optionally provide 'hashable' version of item for set testing """
        entry = ( priority , item )
        heapq.heappush( self , entry )
        if hashable:
            self.s.add( hashable ) 

    def contains( self , hashable ): 
        ''' Test if 'node' is in the queue '''
        return hashable in self.s

    def pop( self ):
        """ Pop the lowest-priority item from the queue """
        priority , item = heapq.heappop( self )
        return item

    def pop_with_priority( self ):
        """ Pop the item and the priority associated with it """
        priority , item = heapq.heappop( self )
        return item , priority

    def pop_opposite( self ):
        """ Remove the item with the longest priority , opoosite of the usual pop """
        priority , item = self[-1]
        del self[-1]
        return item

    def isEmpty(self):
        """ Return True if the queue has no items, otherwise return False """
        return len( self ) == 0

    # __len__ is provided by 'list'

    def unspool( self , N = infty , limit = infty ):
        """ Pop all items as two sorted lists, one of increasing priorities and the other of the corresponding items """
        vals = []
        itms = []
        count = 0
        while not self.isEmpty() and count < N and self.top_priority() <= limit:
            item , valu = self.pop_with_priority()
            vals.append( valu )
            itms.append( item )
            count += 1
        return itms , vals

    def peek( self ):
        """ Return the shortest priority item without popping it """
        priority , item = self[0]
        return item

    def peek_opposite( self ):
        """ Return the longest priority item without popping it """
        priority , item = self[-1]
        return item

    def top_priority( self ):
        """ Return the value of the top priority """
        return self[0][0]

    def btm_priority( self ):
        """ Return the value of the bottom priority """
        return self[-1][0]

class BPQwR( PQwR ): 
    """ Bounded Priority Queue , does not keep more than N items in the queue """

    def __init__( self , boundN , *args ):
        """ Create a priority queue with a specified bound """
        PQwR.__init__( self , *args )
        self.bound = boundN

    def push( self , item , priority , hashable=None ):
        """ Push an item onto the queue and discard largest priority items that are out of bounds """
        PQwR.push( self , item , priority , hashable ) # The usual push
        while len( self ) > self.bound: # If we exceeded the bounds , then discard down to the limit
            self.pop_opposite()

class LPQwR( PQwR ): 
    """ Limited Priority Queue , does not accept items with priorities longer than 'limit' """

    def __init__( self , limitR , *args ):
        """ Create a priority queue with a specified bound """
        PQwR.__init__( self , *args )
        self.limit = limitR

    def push( self , item , priority , hashable = None ):
        """ Push an item onto the queue if it is leq the limit """
        if priority <= self.limit:
            PQwR.push( self , item , priority , hashable ) # The usual push

class Counter( dict ): 
    """ The counter object acts as a dict, but sets previously unused keys to 0 , in the style of 6300 """
    # TODO: Add Berkeley / 6300 functionality

    def __init__( self , *args , **kw ):
        """ Standard dict init """
        dict.__init__( self , *args , **kw )
        if "default" in kw:
            self.defaultReturn = kw['default']
        else:
            self.defaultReturn = 0

    def set_default( self , val ):
        """ Set a new default value to return when there is no """
        self.defaultReturn = val

    def __getitem__( self , a ):
        """ Get the val with key , otherwise return 0 if key DNE """
        if a in self: 
            return dict.__getitem__( self , a )
        return 0

    # __setitem__ provided by 'dict'

    def sorted_keyVals( self ):
        """ Return a list of sorted key-value tuples """
        sortedItems = self.items()
        sortedItems.sort( cmp = lambda keyVal1 , keyVal2 :  np.sign( keyVal2[1] - keyVal1[1] ) )
        return sortedItems

    def sample_until_unique( self , sampleFromSeq , sampleLim = int( 1e6 ) ):
        """ Sample randomly from 'sampleFromSeq' with a uniform distribution until a new key is found or the trial limit is reached , return it """
        # NOTE: If 'sampleLim' is set to 'infty' , the result may be an infinite loop if the Counter has a key for each 'sampleFromSeq'
        trial = 1
        while( trial <= sampleLim ):
            testKey = choice( sampleFromSeq )
            if self[ testKey ] == 0:
                return testKey
            trial += 1
        return None

class RollingList( list ): 
    """ A rolling window based on 'list' """ 

    def __init__( self , winLen , *args ):
        """ Normal 'list' init """
        list.__init__( self , [ 0 ] * winLen , *args )

    def append( self , item ):
        """ Append an item to the back of the list """
        list.append( self , item )
        del self[0]

    def prepend( self , item ):
        """ Prepend an item to the front of the list """
        self.insert( 0 , item )
        del self[-1]

    def get_average( self ):
        """ Get the rolling average , NOTE: Calling this function after inserting non-numeric data will result in an error """
        return sum( self ) * 1.0 / len( self )

    def item_list( self ):
        """ Return a copy of the RollingList as a list """
        return self[:]

# _ End Algo Containers _

def is_nonempty_list( obj ): return isinstance( obj , list , tuple ) and len( obj ) > 0 # Return true if 'obj' is a 'list' with length greater than 0  # <<< resenv

# __ End Structures __


# == Generators, Iterators, and Custom Comprehensions ==

def enumerate_reverse( L ):
    """ A generator that works like 'enumerate' but in reverse order """
    # URL, Generator that is the reverse of 'enumerate': http://stackoverflow.com/a/529466/893511
    for index in reversed( xrange( len( L ) ) ):
        yield index, L[ index ]

def increment( reset = None ):
    """ Count from 0 , or user-specified 'reset' : Increments , and returns increased number when called without argument , arg resets """
    if reset == None:
        increment.i += 1
    else:
        increment.i = reset
    return increment.i
increment.i = 0

# __ End Generators __


# == Printing Helpers ==

def lists_as_columns_with_titles( titles , lists ):
    """ Print each of the 'lists' as columns with the appropriate 'titles' """
    longestList = 0
    longestItem = 0
    prntLists = []
    pad = 4 * ' '
    rtnStr = ""
    if len(titles) == len( lists ):
        for lst in lists:
            if len( lst ) > longestItem:
                longestList = len( lst )
            prntLists.append( [] )
            for item in lst:
                strItem = str( item )
                prntLists[-1].append( strItem )
                if len( strItem ) > longestItem:
                    longestItem = len( strItem )
        line = ''
        for title in titles:
            line += title[ : len(pad) + longestItem -1 ].rjust( len( pad ) + longestItem , ' ' )
        print line
        rtnStr += line + endl
        for index in range( longestList ):
            line = ''
            for lst in prntLists:
                if index < len( lst ):
                    line += pad + lst[ index ].ljust( longestItem , ' ' )
                else:
                    line += pad + longestItem * ' '
            print line
            rtnStr += line + endl
        return rtnStr
    else:
        print "Titles" , len( titles ) , "and lists" , len( lists ) , "of unequal length"
        return ''

def print_list( pList ):
    """ Print a list that is composed of the '__str__' of each of the elements in the format "[ elem_0 , ... , elem_n ]" , 
    separated by commas & spaces """
    prnStr = "[ "
    for index , elem in enumerate( pList ):
        if index < len( pList ) - 1:
            prnStr += str( elem ) + " , "
        else:
            prnStr += str( elem ) + " ]"
    print prnStr

def str_args( *args , **kwargs ):
    """ Print a sequence that is composed of the '__str__' of each of the arguments in the format "elem_0 , ... , elem_n" , separated by commas & spaces """
    prnStr = ""
    for index , elem in enumerate( args ):
        if index < len( args ) - 1:
            prnStr += str( elem ) + " , "
        else:
            prnStr += str( elem )
    if "printStr" in kwargs:
        print prnStr
    else:
        return prnStr

def pretty_print_dict( pDict ):
    """ print a dictionary with uniform columns """
    longestRep = 0
    for key in pDict:
        longestRep = max( longestRep , len( str( key ) ) )
    print "{"
    for key in sorted( pDict ):
        print "\t" , str( key ).ljust( longestRep , ' ' ) , ":" , pDict[ key ]
    print "}"

def yesno( pBool ):
    """ Return YES if True, Otherwise return NO """
    return ( "YES" if pBool else "NO" )

# __ End Printing __


# == File Operations ==

def ensure_dir( dirName ):
    """ Create the directory if it does not exist """
    if not os.path.exists( dirName ):
        os.makedirs( dirName )
        
def is_container_too_big( container , mxSize = int(1e4) ):
    """ Check the length of the container and warn if we should be using a database """
    conLen = len( container ) 
    wrnMsg = "Container has " + str( conLen ) + " elements with max " + str( mxSize ) + ", Consider a database!"
    try:
        if conLen <= mxSize:
            return False
        else:
            warn( wrnMsg , RuntimeWarning )
            print wrnMsg
    except Exception:
        return False

def struct_to_pkl( struct , pklPath ): 
    """ Serialize a 'struct' to 'pklPath' """
    f = open( pklPath , 'wb') # open a file for binary writing to receive pickled data
    cPickle.dump( struct , f ) # changed: pickle.dump --> cPickle.dump
    f.close()

def load_pkl_struct( pklPath ): 
    """ Load a pickled object and return it, return None if error """
    fileLoaded = False
    rtnStruct = None
    try:
        f = open( pklPath , 'rb')
        fileLoaded = True
    except Exception as err:
        print "load_pkl_struct: Could not open file,",pklPath,",",err
    if fileLoaded:
        try:
            rtnStruct = cPickle.load( f )
        except Exception as err:
            print "load_pkl_struct: Could not unpickle file,",pklPath,",",err
        f.close()
    return rtnStruct

def unpickle_dict( filename ):
    """ Return the dictionary stored in the file , Otherwise return an empty dictionary if there were no items """
    try:
        infile = open( filename , 'rb' )
        rtnDict = cPickle.load( infile )
        is_container_too_big( rtnDict )
        if len( rtnDict ) > 0:
            return rtnDict
        else:
            return {}
    except IOError:
        return {}

def process_txt_for_LaTeX( TXTpath , ltxPath = None ):  
    """ Add appropriate line breaks to a TXT file and return as TEX file handle """
    # NOTE: This function writes a TEX file and returns the handle
    if os.path.isfile( TXTpath ):
        drctry , txtName = os.path.split( TXTpath )
        ltxName = txtName[:-4] + ".tex"
        txtFile = file( TXTpath , 'r' )
        txtLines = txtFile.readlines()
        txtFile.close()
        if ltxPath == None:
            ltxPath = os.path.join( drctry , ltxName )
        ltxFile = file( ltxPath , 'w' )
        for line in txtLines:
            # print "Line is:", line
            if len(line) > 3:
                ltxFile.write( line.rstrip() + " \\\\ " + endl ) 
            else:
                ltxFile.write( " $\\ $ \\\\ " + endl ) 
        ltxFile.close()
        return ltxFile
    else:
        raise IOError( "process_txt_for_LaTeX: " + str( TXTpath ) + " did not point to a file!" )

def lines_from_file( fPath ): 
    """ Open the file at 'fPath' , and return lines as a list of strings """
    f = file( fPath , 'r' )
    lines = f.readlines()
    f.close()
    return lines

def string_from_file( fPath ):
    """ Open the file at 'fPath' , and return the entire file as a single string """
    f = file( fPath , 'r' )
    fStr = f.read() # https://docs.python.org/2/tutorial/inputoutput.html#methods-of-file-objects
    f.close()
    return fStr

def txt_file_for_w( fPath ): return file( fPath , 'w' )

def strip_endlines_from_lines( lines ):
    """ Remove the endlines from a list of lines read from a file """
    rtnLines = []
    for line in lines:
        currLine = ''
        for char in line:
            if char != '\n' and char != '\r':
                currLine += char
        rtnLines.append( currLine )
    return rtnLines

def strip_comments_from_lines( lines ):
    """ Remove everything after each # """
    # NOTE: This function does not take into account a '#' within a string
    rtnLines = []
    for line in lines:
        rtnLines.append( str( line.split( '#' , 1 )[0] ) )
    return rtnLines

def purge_empty_lines( lines ):
    """ Given a list of lines , Remove all lines that are only whitespace """
    rtnLines = []
    for line in lines:
        if ( not line.isspace() ) and ( len( line ) > 0 ):
            rtnLines.append( line )
    return rtnLines

def parse_lines( fPath , parseFunc ):
    """ Parse lines with 'parseFunc' while ignoring Python-style # comments """
    # NOTE: This function does not take into account a '#' within a string
    rtnExprs = []
    # 1. Fetch all the lines
    lines = lines_from_file( fPath )
    # 2. Scrub comments from lines
    lines = strip_comments_from_lines( lines )
    # 3. Purge empty lines
    lines = purge_empty_lines( lines )
    # 3.5. Remove newlines
    lines = strip_endlines_from_lines( lines )
    # 4. For each of the remaining lines , Run the parse function and save the results
    for line in lines:
        rtnExprs.append( parseFunc( line ) )
    # 5. Return expressions that are the results of processing the lines
    return rtnExprs


# = class LogMH =

class LogMH:
    """ Text buffer object to hold script output, with facilities to write contents """

    def __init__( self ):
        """ String to store logs """
        self.totalStr = ""

    def prnt( self , *args ):
        """ Print args and store them in a string """
        for arg in args:
            self.totalStr += ascii( arg ) + " "
            print ascii( arg ) ,
        print
        self.totalStr += endl

    def sep( self , title = "" , width = 6 , char = '=' , strOut = False ):
        """ Print a separating title card for debug """
        LINE = width * char
        self.prnt( LINE + ' ' + title + ' ' + LINE )
        if strOut:
            return LINE + ' ' + title + ' ' + LINE

    def write( self , *args ):
        """ Store 'args' in the accumulation string without printing """
        numArgs = len( args )
        for i , arg in enumerate( args ):
            self.totalStr += ascii( arg ) + ( " " if i < numArgs-1 else "" )

    def out_and_clear( self , outPath ):
        """ Write the contents of 'totalStr' to a file and clear """
        outFile = file( outPath , 'w' )
        outFile.write( self.totalStr )
        outFile.close()
        self.clear()

    def clear( self ):
        """ Clear the contents of 'accum.totalStr' """
        self.totalStr = ""

# _ End LogMH _


def touch( fname ):
    """ Create the file if it DNE , Otherwise let the file know we touched it """
    # URL , 'touch' a file: https://stackoverflow.com/a/1158096
    if os.path.exists( fname ):
        os.utime( fname , None )
    else:
        open( fname , 'a' ).close()

# __ End File __


# == Batch Operations ==

def validate_dirs_writable( *dirList ):
    """ Return true if every directory argument both exists and is writable, otherwise return false """
    # NOTE: This function exits on the first failed check and does not provide info for any subsequent element of 'dirList'
    # NOTE: Assume that a writable directory is readable
    for directory in dirList:
        if not os.path.isdir( directory ):
            print "Directory" , directory , "does not exist!"
            return False
        if not os.access( directory , os.W_OK ): # URL, Check write permission: http://stackoverflow.com/a/2113511/893511
            print "System does not have write permission for" , directory , "!"
            return False
    return True # All checks finished OK, return true

# __ End Batch __


# == String Processing ==

def ascii( strInput ): 
    """ Return an ASCII representation of the string or object, ignoring elements that do not have an ASCII representation """
    if type( strInput ) in ( unicode , str ):
        return str( strInput.encode( 'ascii' , 'ignore' ) )
    else:
        return str( strInput ).encode( 'ascii' , 'ignore' )

def strip_after_first( pStr , char ): 
    """ Return a version of 'pStr' in which the first instance of 'char' and everything that follows is removed, if 'char' exists in 'pStr', otherwise return 'pStr' """
    firstDex = pStr.find( char )
    if firstDex > -1:
        return pStr[:firstDex]
    return pStr

def tokenize_with_wspace( rawStr , evalFunc = str ): 
    """ Return a list of tokens taken from 'rawStr' that is partitioned with whitespace, transforming each token with 'evalFunc' """
    return [ evalFunc( rawToken ) for rawToken in rawStr.split() ]

def tokenize_with_char( rawStr , separator = ',' ,  evalFunc = str ): 
    """ Return a list of tokens taken from 'rawStr' that is partitioned with a separating character, transforming each token with 'evalFunc' """
    return [ evalFunc( rawToken ) for rawToken in rawStr.split( separator ) ]

def tokenize_with_separator( rawStr , separator , evalFunc = str ):
    """ Return a list of tokens taken from 'rawStr' that is partitioned with 'separator', transforming each token with 'evalFunc' """
    # TODO: Maybe this could be done with brevity using regex?
    tokens = [] # list of tokens to return
    currToken = '' # the current token, built a character at a time
    for char in rawStr: # for each character of the input string
        if not char.isspace(): # if the current char is not whitespace, process
            if not char == separator: # if the character is not a separator, then
                currToken += char # accumulate the char onto the current token
            else: # else the character is a separator, process the previous token
                tokens.append( evalFunc( currToken ) ) # transform token and append to the token list
                currToken = '' # reset the current token
        # else is whitespace, ignore
    if currToken: # If there is data in 'currToken', process it
        tokens.append( evalFunc( currToken ) ) # transform token and append to the token list
    return tokens

def format_dec_list( numList , places = 2 ): 
    """ Return a string representing a list of decimal numbers limited to 'places' """
    rtnStr = "[ "
    for nDex , num in enumerate( numList ):
        if isinstance( numList , np.ndarray ):
            scalar = num.item()
        else:
            scalar = num 
        if nDex < len(numList) - 1:
            rtnStr += ('{0:.' + str( places ) + 'g}').format( scalar ) + ' , '
        else:
            rtnStr += ('{0:.' + str( places ) + 'g}').format( scalar )
    rtnStr += " ]"
    return rtnStr

def string_contains_any( bigStr , subsList ):
    """ Return True if 'bigStr' contains any of the substrings in 'subsList' , Otherwise return False """
    for sub in subsList:
        if sub in bigStr:
            return True
    return False

def get_EXT( fName ):
    """ Return the capitalized file extension at the end of a path without the period """
    return os.path.splitext( fName )[-1][1:].upper()

def strip_EXT( fName ):
    """ Return the filepath before the extension """
    return os.path.splitext( fName )[0]

# __ End Strings __


# == Timing / Benchmarking ==

class HeartRate: # NOTE: This fulfills a purpose similar to the rospy rate
    """ Sleeps for a time such that the period between calls to sleep results in a frequency not greater than the specified 'Hz' """
    
    def __init__( self , Hz ):
        """ Create a rate object with a Do-Not-Exceed frequency in 'Hz' """
        self.period = 1.0 / Hz; # Set the period as the inverse of the frequency , hearbeat will not exceed 'Hz' , but can be lower
        self.last = time.time()
    
    def sleep( self ):
        """ Sleep for a time so that the frequency is not exceeded """
        elapsed = time.time() - self.last
        if elapsed < self.period:
            time.sleep( self.period - elapsed )
        self.last = time.time()

class Stopwatch( object ):
    """ Timer for benchmarking """

    def __init__( self ):
        """ Init with watch started """
        self.strtTime = time.time()
        self.stopTime = infty

    def start( self ):
        self.strtTime = time.time()

    def stop( self ):
        self.stopTime = time.time()

    def duration( self ):
        return self.stopTime - self.strtTime

    def elapsed( self ):
        return time.time() - self.strtTime    

# __ End Timing __


# === Reporting ===

# == class Response ==

class Response:
    """ Container class to hold the result of a search or an error """
    def __init__( self , result = False , errCode = {} , data = [] ): # NOTE: Error codes are dict entries to make lookup easier
        self.result     = result
        self.errorCodes = errCode
        self.data       = data

# __ End Response __

# ___ End Reporting ___


# === Spare Parts ===



# ___ End Parts ___
