#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

"""
Graph.py , Built on Spyder for Python 2.7
James Watson, 2017 March
General purpose graph classes and methods
"""

# ~~ Libraries ~~
# ~ Standard ~
import collections , sys 
# ~ Special ~
# ~ Local ~
from marchhare import Counter , Stack


print "Loading special vars ..." , 
import os
EPSILON = 1e-7
infty = 1e309 # URL: http://stackoverflow.com/questions/1628026/python-infinity-any-caveats#comment31860436_1628026
endl = os.linesep
DISPLAYPLACES = 5 # Display 5 decimal places by default
print "Loaded!"

# === Data Structures ===

# == Tagged Object ==

class TaggedObject( object ):
    """ An object with a unique tag and a string alias """
    
    def __init__( self ):
        """ The simplest means to give each object a unique tag """ # NOTE: No need for a global lookup at this time
        self.desc = self.__class__.__name__ # Shorten name access
        self.tag = self.desc + str( id( self ) ) # Create a tag composed of the class name and memory id
        self.alias = "" # Init alias to the empty string
        
    def __str__( self ):
        """ String representation , by class and then by alias ( or tag if alias is not available ) """
        if self.has_alias():
            return self.__class__.__name__ + ":" + str( self.alias )
        else:
            return self.__class__.__name__ + ":" + str( self.tag )
        
    def give_alias( self , nick ):
        """ Assign alias , must be hashable , should use this to assign alias """
        if isinstance( nick , collections.Hashable ):
            self.alias = nick
        else:
            raise ValueError( "TaggedObject.give_alias: Alias must be hashable, received unhashable object" + str( nick ) )
        
    def has_alias( self ):
        """ Return True if an alias other than an empty string has been set """
        return self.alias != ""
    
    def best_ID( self ):
        """ Return the alias if it exists , otherwise return the tag """
        return self.alias if self.has_alias() else self.tag
  
# = class TaggedLookup =
    
class TaggedLookup( list ):
    """ A container for TaggedObjects, can be used for lookup and removal by tag and alias """
    
    def __init__( self , *args ):
        """ Normal 'list' init """
        list.__init__( self , *args )
        self.lookupByTag = {} # Dictionary of objects stored by tag
        self.lookupByAls = {} # Dictionary of objects stored by alias
        self.verbose = False
        
    def append( self , item ):
        """ Add an item to the lookup """
        list.append( self , item )
        self.lookupByTag[ item.tag ] = item
        if item.has_alias():
            self.lookupByAls[ item.alias ] = item
            
    def extend( self , extList ):
        """ Add a list of items to the lookup """
        for item in extList:
            self.append( item )
        
    def get_by_tag(self, pTag):
        """ Return an object reference from the lookup with a matching tag if it exists, otherwise return None """
        rtnObj = None
        try:
            rtnObj = self.lookupByTag[pTag]
        except KeyError:
            if self.verbose:
                print "Object with tag",pTag,"DNE in this problem"
        return rtnObj
        
    def tag_exists( self , tag ):
        """ Return True if 'tag' exists in 'lookupByTag' as a key , Otherwise return False """
        return tag in self.lookupByTag.keys()
        
    def get_by_als(self, pAls):
        """ Return an object reference from the lookup with a matching alias if it exists, otherwise return None """
        try:
            return self.lookupByAls[pAls]
        except KeyError:
            if self.verbose:
                print "Object with alias",pAls,"DNE in this problem"
            return None
        
    def als_exists( self , als ):
        """ Return True if 'als' exists in 'lookupByAls' as a key , Otherwise return False """
        return als in self.lookupByAls.keys()    
                        
    def __str__(self):
        """ Print all contents by tag """
        rtnStr = ''
        for key, val in self.lookupByTag.iteritems():
            rtnStr += str(key) + '\t\t' + str(val) + endl 
        return rtnStr
        
    # '__len__' inherited from 'list'
        
    def get_list( self ):
        """ Get all the elements in the lookup in a list """
        return self[:] # This object is already a list , just copy and return
    
    def get_alias_list( self ):
        """ Get the aliases of all the elements in the lookup in a 1-to-1 list , NOTE: This function does not filter empty aliases """
        return [ elem.alias for elem in self ]
    
    def get_alias_index( self , pAlias ):
        """ Get the index of the element with the given alias , otherwise return 'None' """
        try:
            return self.get_alias_list().index( pAlias )
        except ValueError:
            return None
    
    def recalc_alias_lookup( self , erase = False ):
        """ Make sure that any elements that have recently acquired an alias appear in the alias lookup """
        for elem in self:
            if elem.has_alias():
                self.lookupByAls[ elem.alias ] = elem
    
    def assign_aliases_by_index( self ):
        """ Set the alias of each of the contained objects to be its corresponding index in the list , NOTE: Rewrites all element aliases! """
        for index , elem in enumerate( self ):
            elem.give_alias( index )
        self.recalc_alias_lookup() # Now that aliases have been assigned , add them to the lookup
        
    def pop( self , index = None ):
        """ Remove the entry at the end of the list , or at 'index' if it is provided and valid """
        if index == None:
            index = len( self ) - 1
        elif index > len( self ) - 1:
            raise IndexError( "Index " + str( index )  + " is not valid for a structure of length " + str( len( self ) ) )
        if self[ index ].has_alias(): # 1. Remove from the alias lookup if it exists there
            del self.lookupByAls[ self[ index ].alias ]
        del self.lookupByTag[ self[ index ].tag ] # 2. Remove from the tag lookup
        return list.pop( self , index ) # 3. Remove from the structure itself
        
    def remove( self , value ):
        """ Remove the entry that matches 'value' """
        try:
            rmDex = self.index( value )
            self.pop( rmDex ) 
        except ValueError:
            raise ValueError( "No object " + str( value ) + " exists in this lookup" )
            
    def remove_by_alias( self , rmAlias ):
        """ If 'rmAlias' exists in the lookup , remove that object from the lookup , otherwise take no action """
        rmDex = self.get_alias_index( rmAlias )
        if rmDex != None:
            self.pop( rmDex )

# = End TaggedLookup =  

# == End Tagged ==

# == Weighted List ==

class WeightedList( list ): 
    """ A list in which every element is labelled with a dictionary , Use this when it is not convenient to add new attibutes to list elements """
    
    def __init__( self , *args ):
        """ Normal 'list' init """
        list.__init__( self , *args )
        self.weights = [ None for elem in xrange( len( self ) ) ]
        self.costs   = [ 1    for elem in xrange( len( self ) ) ]
        
    def append( self , item , weight = {} , cost = 1.0 ):
        """ Append an item and its label """
        list.append( self , item )
        self.weights.append( weight )
        self.costs.append( cost )
        
    def extend( self , items , weights = None , costs = None ):
        if weights == None or len( items ) == len( weights ):
            list.extend( self , items )
            self.weights.extend( weights if weights != None else [ {}  for elem in xrange( len( items ) ) ] )
            self.weights.extend( costs   if costs   != None else [ 1.0 for elem in xrange( len( items ) ) ] )
        elif weights != None:
            raise IndexError( "LabeleWeightedListdList.extend: Items and labels did not have the same length! " + str( len( items ) ) + " , " + str( len( weights ) ) )
    
    def pop( self , index = None ):
        """ Remove the item at 'index' and the associated 'weights' & 'costs' , Note: 'weights' & 'costs' are not returned """
        if index == None:
            index = len( self ) - 1
        self.weights.pop( index )
        self.costs.pop( index )
        return list.pop( self , index )
    
    def remove( self , value ):
        """ Remove the 'value' and the associated 'weights' & 'costs' """
        try:
            rmDex = self.index( value )
            self.pop( rmDex )
        except ValueError:
            raise ValueError( str( value ) + " was not found in the structure" )
    
    def get_weight( self , index , weightName ):
        """ Get the 'label' value for the item at 'index' , if that label DNE at that index return None """
        try:
            return self.weights[ index ][ weightName ]
        except IndexError:
            return None
        
    def set_weight( self , index , weightName , value ):
        """ Set the 'weight' value for the item at 'index' to 'value' """
        self.weight[ index ][ weightName ] = value
        
    def get_cost( self , index ):
        """ Get the cost for the item at 'index' """
        return self.costs[ index ]
    
    def set_cost( self , index , val ):
        """ Set the cost for the item at 'index' to 'val' """
        self.costs[ index ] = val
        
    def index( self , val ):
        """ Return he index of the first occurrence of 'val' or 'None' """
        try:
            return list.index( self , val )
        except:
            return None
        
    # Only add other 'list' functions as needed
    
# == End Weighted ==
      
# === End Structures ===


# === Graph Classes ===

# == class SimpleNode ==

class SimpleNode:
    """ Search tree node without a specific application in mind """    
    
    def __init__( self , data = None , parent = None ):
        self.data          = data # - Configuration
        self.children      = [] # --- Successors , possibly leading to the goal configuration
        self.parent        = parent # Parent , for constructing path back to the start configuration

    def add_child( self , child ):
        """ Add a successor node """
        # NOTE: No correctness checks are made before adding
        self.children.append( child )
        
    def remove_child_by_ref( self , childRef ):
        """ Remove the object referred to by 'childRef' from 'self.children' , if 'childRef' DNE then fail gracefully """
        try:
            self.children.remove( childRef )
        except ValueError:
            print "WARN , Node.remove_child_by_ref: Reference" , childRef , "DNE in list of successors!"
        
    def __str__( self ):
        """ Return a string representation of the TreeNode """
        return "Node@" + str( id( self ) )

# __ End SimpleNode __
        

# == class Node ==

class Node( TaggedObject ):
    """ A graph node with edges """    
    
    def __init__( self , pGraph = None , alias = None , nCost = 1.0 ):
        super( Node , self ).__init__()
        self.edges = WeightedList() # NOTE: These represent outgoing edges. Incoming edges are represented as references to this Node in other Nodes
        # The graph that this node belongs to
        self.graph = pGraph 
        if alias != None: # If there was an alias specified , then store it
            self.give_alias( alias ) 
        self.bag = Counter( default = None ) # This is a generic place to dump miscellaneous
        self.cost = nCost # Numeric cost associated with this node
        self.parent = None
        
    def add_child( self , pNode , weight = {} ):
        """ Connect an edge between this Node and 'pNode' """
        # NOTE: This function assumes that an undirected edge has the same weight in both directions
        self.edges.append( pNode , weight ) # NOTE: This function assumes this Node is the tail
        pNode.parent = self
        
    def connect_to( self , pNode , pDir = False , weight = {} ):
        """ Connect an edge between this Node and 'pNode' """
        # NOTE: This function assumes that an undirected edge has the same weight in both directions
        self.edges.append( pNode , weight ) # NOTE: This function assumes this Node is the tail
        if not pDir: # If the edge is not directed, add a reference from 'pNode' back to this Node
            pNode.edges.append( self , weight )
        
    def connect_to_checked( self , pNode , pDir = False , weight = {}  ):
        """ Connect an edge between this Node and 'pNode' , avoiding duplicates """
        if pNode not in self.edges and self not in pNode.edges:
            self.connect_to( pNode , pDir , weight )
        else: # Else this edge exists, connection failed
            return None
        
    def get_successors( self ):
        """ Return a list of successors of the current node """
        return self.edges[:]
        
    def num_successors( self ):
        """ Return the number of successors """
        return len( self.edges )
        
# __ End Node __

# == class Graph ==

class Graph( TaggedObject ):
    """ A simple graph structure with nodes and edges """
    
    def __init__( self , rootNode = None ):
        super( Graph , self ).__init__()
        self.nodes = TaggedLookup()
        self.edges = [] # For now not providing any sort of fancy lookup structure for edges
        self.root = rootNode
        
    def __str__( self ):
        """ Return a string representation of the Graph """
        return "Graph@" + str( id( self ) ) + " with " + str( len( self.nodes ) ) + " nodes"
    
    def get_root( self ):
        """ Return the root node """
        return self.root
    
    def set_root( self , rootNode ):
        """ Set the root node """
        self.root = rootNode
        
    def add_node_by_ref( self , nodeRef ):
        """ Add a Node , make note of tag and alias for easy lookup """ 
        # NOTE: A Node needn't be a member of the Graph in order to be reachable , this function is for establishing the relationship when it is important
        nodeRef.graph = self # Establish node membership
        self.nodes.append( nodeRef )
        
    def add_node_by_ref_safe( self , nodeRef ):
        """ Add the node only if a Node with such a tag does not already exist in the Graph """
        if ( not self.nodes.tag_exists( nodeRef.tag ) ) and ( not self.nodes.als_exists( nodeRef.alias ) ):
            self.nodes.append( nodeRef )
        
    def rem_node_by_ref( self , nodeRef ):
        """ Remove a Node that matches the reference """
        self.nodes.remove( nodeRef )
        # NOTE: At this time , not checking if edges still exist in 'edges' that refer to this node
        
    def create_node_with_als( self , alias ):
        """ Create a Node with a given 'alias' and add it to the graph """
        temp = Node( self , alias )
        temp.graph = self # Establish node membership
        self.nodes.append( temp )
        return temp
        
    def get_node_by_als( self , nodeAls ):
        """ Search for a node by its alias and return the reference if the node exists in the problem """
        return self.nodes.get_by_als( nodeAls )
        
    def connect_by_ref( self , tail , head , pDir = False , weight = {} ):
        """ Connect two nodes , assuming we already have references to them """
        self.edges.append( tail.connect_to( head , pDir , weight ) )
        
    def connect_by_als( self , tailAls , headAls , pDir = False , weight = {} ):
        """ Connect two nodes using their aliases """
        tail = self.get_node_by_als( tailAls )
        head = self.get_node_by_als( headAls )
        if ( tail and head ):
            self.edges.append( tail.connect_to( head , pDir , weight ) )
        else:
            print "WARN , Graph.connect_by_als: One of" , tailAls , "or" , headAls , "DNE in this Graph"
        
    def get_node_list( self ):
        """ Return a list of nodes in this graph """
        return self.nodes.get_list()
    
    def assign_indices_to_nodes( self ):
        """ Load each of the nodes with information about their current index """
        for noDex , currNode in enumerate( self.get_node_list() ):
            currNode.bag['index'] = noDex
            
    def erase_indices_from_nodes( self ):
        """ Safely erase index information from all of the node in this graph """
        for noDex , currNode in enumerate( self.get_node_list() ):
            # URL , Two ways to remove dict key: https://stackoverflow.com/questions/11277432/how-to-remove-a-key-from-a-python-dictionary
            currNode.bag.pop( 'index' , None ) # Remove the 'index' key , if it exists
            
    def dfs( self , initNode , is_goal ):
        ''' Depth First Search 
    
        init_state - the intial state on the map
        f -          transition function of the form s_prime = f(s,a)
        is_goal -    function taking as input a state s and returning True if its a goal state
        actions -    set of actions which can be taken by the agent
    
        returns -     ((path, action_path), visited) or None if no path can be found
            goal_node -   The first node that meets the goal requirement
            path -        a list of tuples. The first element is the initial state followed by all states traversed until the final goal state
            action_path - the actions taken to transition from the initial state to goal state '''
        
        frontier = Stack() # Search stack, pop from the top
        n0 = initNode
        n0.path = [] # Attach a sequence to each node to record the path to it, this may consume a lot of space for large problems
        frontier.push( n0 ) # Push the starting node on the frontier
        visited = set([]) 
        while len( frontier ) > 0: # While states remain on the frontier
            n_i = frontier.pop() # Pop last element
            if n_i.tag not in visited: # If the state has not been previously popped, then
                visited.add( n_i.tag ) # Log the visit
                currentPath = n_i.path # Path to the current state
                if is_goal( n_i ): # If the goal has been reached, return path and visited information
                    return ( n_i , n_i.path + [ n_i.tag ] , visited )
                else: # else is not goal
                    for edge in n_i.edges:
                        s_prime = edge
                        s_prime.path = currentPath[:] + [ n_i.tag ] # Assemble a plan that is the path so far plus the current action
                        frontier.push( s_prime ) # Push onto top of the frontier                        
        return ( None , None , visited ) # The frontier has been exhausted without finding the goal, return the sad story of the journey    
        
# == End Graph ==
        
# === End Graphs ===