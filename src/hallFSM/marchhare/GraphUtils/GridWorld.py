#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

"""
GridWorld.py , Built on Spyder for Python 2.7
James Watson, 2017 March
Grid / Manhattan extensions for graphs

== REQUIRES ==
* ResearchEnv

== NOTES ==
* For now, not performing agent bookkeeping or collision checking in grid worlds until it is needed in some future problem

== TODO ==

"""

# ~~ Libraries ~~
# ~ Standard ~
import collections , sys 
from random import randrange
# ~ Special ~
# ~ Local ~
from .. import Graph

if "AsmEnv" not in sys.modules:
    print "Loading special vars ..." , 
    import os
    EPSILON = 1e-7
    infty = 1e309 # URL: http://stackoverflow.com/questions/1628026/python-infinity-any-caveats#comment31860436_1628026
    endl = os.linesep
    DISPLAYPLACES = 5 # Display 5 decimal places by default
    print "Loaded!"


# === Grid World Helpers ===

# == Manhattan Geometry ==

def vec2D_add( op1 , op2 ):
	""" Add 'op1' and 'op2' and return resultant 2-tuple """
	return ( op1[0] + op2[0] , op1[1] + op2[1] ) # Aliases are hashable tuple addresses

def vec2D_sub( op1 , op2 ):
	""" Subtract 'op2' from 'op1' and return resultant 2-tuple """
	return ( op1[0] - op2[0] , op1[1] - op2[1] ) # Aliases are hashable tuple addresses
 
def taxi_diff(op1, op2):
    """ Return the relative Manhattan displacement from 'op1' to 'op2' """
    return [op2[0] - op1[0], op2[1] - op1[1]]

def taxi_dist(op1, op2):
    """ Return the absolute Manhattan distance between 'op1' and 'op2' """
    disp = taxi_diff(op1, op2)
    return abs(disp[0]) + abs(disp[1])

def vec_clamp_limits( vec , limits ):
    """ Return a version of 'vec' with all elements clamped b/n limits[i]['loBnd' , 'upBnd'], inclusive """
    rtnVec = []
    for cDex , coord in enumerate( vec ):
        if coord < limits[cDex][0]:
            rtnVec.append( limits[cDex][0] )
        elif coord > limits[cDex][1]:
            rtnVec.append( limits[cDex][1] )
        else:
            rtnVec.append( coord )
    return rtnVec

# == End Manhattan ==


# == Automata Math ==

# TODO: Maybe store these as module or object variables to avoid needlessly cumbersome , non-obvious nested dictionary lookups 

"""    ^
+yRows |
       |
       +---->
  (0,0)    +xCols
"""

DIRS = \
[ ( 0 , 1 ) , ( 1 , 1 ) , ( 1 , 0 ) , ( 1 ,-1 ) , ( 0 ,-1 ) , (-1 ,-1) , (-1 , 0 ) , (-1 , 1 ) ]
NT=0        ; NE=1      ; ES=2      ; SE=3      ; SO=4      ; SW=5     ; WE=6      ; NW=7
DIRNAMES = \
{ 'NT':NT, 'NE':NE, 'ES':ES, 'SE':SE, 'SO':SO, 'SW':SW, 'WE':WE, 'NW':NW }
REVNAMES = \
{ NT:'NT', NE:'NE', ES:'ES', SE:'SE', SO:'SO', SW:'SW', WE:'WE', NW:'NW' }
MOOREN = [ NT , NE , ES , SE , SO , SW , WE , NW ] # Moore Neighborhood
VONNEUMANNN = [ NT , ES , SO , WE ] # Von Neumann Neighborhood
MOORESET =      [ REVNAMES[n] for n in MOOREN ] # ---- Names of the Moore actions
VONNEUMANNSET = [ REVNAMES[n] for n in VONNEUMANNN ] # Names of the Von Neumann actions
VNNMNTURNS = { 'FWRD': 0 , 'RGHT': 1 , 'BACK': 2 , 'LEFT': -1 } # Index offset from current index in 'VONNEUMANNSET' to perform the named turn
# MOORESET = [ 'NT' , 'NE' , 'ES' , 'SE' , 'SO' , 'SW' , 'WE' , 'NW' ] # Moore Neighborhood
# VONNEUMANNSET = [ 'NT' , 'ES' , 'SO' , 'WE' ] # Von Neumann Neighborhood

def moore_neighbors_of(addr): # 8 adjacent
    """ Return the Moore neighborhood of 'addr' """
    neighborhood = []
    for nghbr in MOOREN:
        neighborhood.append( vec2D_add(addr, DIRS[nghbr]) )
    return neighborhood
        
def vnnmn_neighbors_of(addr): # 4 cardinal directions
    """ Return the von Neumann neighborhood of 'addr' """
    neighborhood = []
    for nghbr in VONNEUMANNN:
        neighborhood.append( vec2D_add(addr, DIRS[nghbr]) )
    return neighborhood
    
def vnnmn_turn_from( drctn , turn ):
    """ Return the direction index that is 'turn' from 'drctn' """
    return elemw( DIRNAMES[ drctn ] + \
                  VNNMNTURNS[ turn ] , 
                  VONNEUMANNN )
    
def vnnmn_neighbor( addr , drctn ):
    """ Return the address that is 'drctn' from 'addr' """
    return vec2D_add( addr, DIRS[ drctn ] )
        
def vnnmn_NE_nghbrs_of( addr ): 
    """ Return the NE von Neumann neighborhood of 'addr' """
    # This function is used in assigning connections
    neighborhood = []
    for nghbr in [ DIRS[NT] , DIRS[ES] ]:
        # print "Vector Sum:" ,addr , '+' ,nghbr ,'=', vec2D_add( addr , nghbr )
        neighborhood.append( vec2D_add( addr , nghbr ) )   
    return neighborhood

# == End Automata ==


# == Grid Classes ==
        
class GridGraph(Graph):
    """ Graph representing a grid world """
    
    def __init__( self  , pXCols , pYRows , neighborFunc = vnnmn_NE_nghbrs_of , pActSet = VONNEUMANNSET , transFunc = None ):
        super( GridGraph , self ).__init__()
        self.xCols = pXCols # Store the size of the grid world
        self.yRows = pYRows
        self.addrBounds = [ [ 0 , pXCols - 1 ] , [ 0 , pYRows - 1 ] ] # Set the world bounds for address checking
        self.actionSet = pActSet # Specify the set of actions that can be performed for this problem
        # Create Nodes
        for x in xrange( self.xCols ): # for each X 
            for y in xrange( self.yRows ): # for each Y
                temp = Node() # Create a node with the designated address
                temp.give_alias( ( x , y ) )
                self.add_node( temp ) # Attache the node to the graph so we can find it
        # Connect Nodes
        for x in xrange( self.xCols ): # For each X
            for y in xrange( self.yRows ): # For each Y
                temp1 = self.nodes.get_by_als( ( x , y ) ) # Fetch the node at the addr
                for nhbr_addr in neighborFunc( temp1.alias ): # Generate all the neighbors and iterate
                    if self.chk_addr( nhbr_addr ): # if the neighbor is at a valid addr
                        temp2 = self.nodes.get_by_als( tuple( nhbr_addr ) )
                        self.edges.append( temp1.connect_to( temp2 ) )
        # Set up the transition function
        if transFunc == None:
            self.transition = self.transition_determ
        else:
            self.transition = transFunc
                
    def chk_addr( self , addr ): # <<< antworld 
        """ Check if the address passed is valid in this instance of World """ 
        return self.nodes.get_by_als( addr ) != None # NOTE: Searching on the address hash will also check against states removed from rect interior
        
    def push_in_bounds( self , addr ): # NOTE: This should not be necessary as long as the edges are set up correctly
        """ Return a version of 'addr' that is within the bounds of the grid world """ # NOTE: This function assumes that 'addr' is a 2-tuple
        return vec_clamp_limits( addr , self.addrBounds )
        
    def node_from_addr( self , addr ):
        """ Return a Node corresponding to the state 'addr' """
        return self.nodes.get_by_als( addr )
        
    # TODO: Functions that remove nodes/edges that represent blocked spaces
    
    def transition_determ( self , state , action ):
        """ Return the deterministic outcome of a 'state'-'action' pair """
        # { 'NT' , 'NE' , 'ES' , 'SE' , 'SO' , 'SW' , 'WE' , 'NW' }
        addrNext = tuple( vec2D_add( state.alias , DIRS[ DIRNAMES[ action ] ] ) ) # Get the address that is the present plus the action delta
        if self.chk_addr( addrNext ): # If the offset state is present in the problem, check if there is an edge from this state to it
            node = self.node_from_addr( addrNext )
            for outgoing in node.edges:
                print outgoing
                if outgoing.alias == addrNext:
                    return outgoing # Found an edge to get us there, go!
            return state # No edge between here and there, stay in this state
        else:
            return state # State requested by the offset does not exist, no transition
        
    def rand_addr( self , many = False ):
        """ Return a random , valid address within the grid world -OR- Return a list of 'many' valid, random addresses """
        if many:
            return [ ( randrange( self.xCols ) , randrange( self.yRows ) ) for i in xrange( many ) ]
        else:
            return ( randrange( self.xCols ) , randrange( self.yRows ) )  
            
# == End grid classes ==
    
# === End Grid World ===

# == Main Testing ==

if __name__ == "__main__":
    
    foo = GridGraph( 3 , 3 )
    print "There are" , len( foo.nodes ) , "nodes"
    for node in foo.nodes: print node.alias
    print "There are" , len( foo.edges ) , "edges"
    for edge in foo.edges: print edge
    print foo.nodes
    
    state = foo.node_from_addr( ( 1 , 1 ) ) # The middle state
    print foo.actionSet
    for action in foo.actionSet:
        print foo.transition_determ( state , action ).alias , # (1, 2) (2, 1) (1, 0) (0, 1)
    print
    
# == End Testing ==