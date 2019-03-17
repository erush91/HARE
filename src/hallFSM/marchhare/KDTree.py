#!/usr/bin/env python


import numpy as np
from math import pi , sqrt

from random import choice , randrange # ADDED
from types import NoneType # ADDED


# === RRT classes and functions ===

class TreeNode:
    """ Search tree node for use with RRT """    
    
    def __init__(self, state, parent=None):
        self.state = state
        self.children = []
        self.parent = parent

    def add_child(self, child):
        self.children.append(child)
        
    def __str__( self ):
        return "TreeNode@" + str( self.state )

class kdNode: # ADDED
    """ Search tree node with the combined aspects of a kd-tree binary node and an RRT search node """    
    
    def __init__( self , pState , pSplitDim = 0 , pParent = None , pLeft = None , pRight = None ):
        """ Create a node representing a configuration state """
        # Configuration State
        self.state = pState
        # Binary Node Members
        self.left = pLeft
        self.rght = pRight
        self.splitDim = pSplitDim
        # RRT Node Members
        self.parent = pParent
        self.prntDist = 0
        self.rootDist = 0
        self.children = []
        
    def add_child( self , child ):
        """ Add a child (RRT) """ 
        child.prntDist = vec_dif_mag( child.state , self.state ) # Calc the distance between this state and the child state
        child.rootDist = self.rootDist + child.prntDist # calc the distance between the child state and the root via parents
        child.parent = self
        self.children.append( child )
        
    def rmv_child( self , child ):
        """ Remove a child from the list of successors , note that this does not update distance information from the removed node! """
        child.parent = None
        # print "Removing child, does the child exist?" , child in self.children
        self.children.remove( child ) # Standard Python allows removal by object reference
        # print "Removed child,  does the child exist?" , child in self.children
        
    def rewire( self , nuParent , rrtObj = None ):
        """ Conncect this node to a new parent """
#        if rrtObj != None: # This is for display only!
#            rrtObj.edges.remove( ( self.parent.state , self.state ) )
#            rrtObj.edges.append( ( nuParent.state , self.state ) )
            
        self.parent.rmv_child( self ) # This does not change the distances
        nuParent.add_child( self ) # This will calc new distances to parent and root
        
    
    def __str__( self ):
        """ Return a string that identifies the name of the class and the state that it represents """
        return self.__class__.__name__ + "@" + str( self.state )

class kd_Tree_Approx:
    """ kd-Tree for searching nearest neighbors in R^n space , NN search results are approximate but unsure why """
    
    def __init__( self , pDim ):
        """ Init the tree in which each node holds a state with 'pDim' """
        self.dim = pDim
        # self.limits = pLimits
        self.root = None
        
    def insert_recur( self , rootNode , addNode , splitD ):
        """ Create a BinaryNode from a state and add it to the binary tree """
        if not rootNode: # Base Case: There is no node at the root, install this node as the root
            # print "Adding node!" , 
            addNode.splitDim = splitD
            rootNode = addNode
        elif addNode.state[ rootNode.splitDim ] < rootNode.state[ rootNode.splitDim ]: # else if the new nodes's splitting coord is less than root's splitting coord
            rootNode.left = self.insert_recur( rootNode.left , addNode , indexw( splitD + 1 , addNode.state ) ) # increment splitting coord and recur on the left tree
        else: # else the new node's splitting coord is greater than or equal to the root's splitting coord
            rootNode.rght = self.insert_recur( rootNode.rght , addNode , indexw( splitD + 1 , addNode.state ) ) # increment splitting coord and recur on the right tree
        return rootNode # Root exists at this level, and the new node was added at some deeper level, leave the root node intact
        
    def insert( self , addNode ):
        """ Add a node to the kd-tree """
        self.root = self.insert_recur( self.root , addNode , 0 )
        
    def insert_state( self , addState ):
        """ Create a node that corresponds to the configuration state and add it to the kd-tree """
        self.root = self.insert_recur( self.root , kdNode( addState ) , 0 )

    @staticmethod
    def NN_recur( testPoint , node , bestNode = None , bestDist = infty ):
        """ Find the nearest neighbor to 'state' in the subtree below 'node' """
        # Base Case: Once the algorithm reaches a leaf node, it saves that node point as the "current best"
        if not node.left and not node.rght:
            # testDist = vec_dif_sqr( node.state , testPoint )
            splitDex = node.splitDim # Get the splitting dimension
            return node , vec_dif_sqr( node.state , testPoint ) # testDist if testDist < bestDist else bestDist
            
        # Recurive Case: the algorithm moves down the tree recursively, in the same way that it would if the search point were being inserted 
        #                 (i.e. it goes left or right depending on whether the point is lesser than or greater than the current node in the split dimension) 
        
        else: # else this is a non-leaf node
            # before we go any further, check how far this node is from the test point
            nodeDist = vec_dif_sqr( node.state , testPoint )
            if nodeDist < bestDist:
                bestDist = nodeDist
                bestNode = node
                
            # Now we can check successors   
            splitDex = node.splitDim # Get the splitting dimension
            twoSides = False
            leftBest = False
            
            # 1. Get the "current best" from the first leaf node encountered on the binary search
            if node.left and node.rght: # Both branches are present, compare on the split dimension
                twoSides = True
                if abs( node.left.state[ splitDex ] - testPoint[ splitDex ] ) <= abs( node.rght.state[ splitDex ] - testPoint[ splitDex ] ):
                    recurNode , recurDist = kd_Tree_Approx.NN_recur( testPoint , node.left , bestNode , bestDist ) # Current best from the left
                    leftBest = True
                else:
                    recurNode , recurDist = kd_Tree_Approx.NN_recur( testPoint , node.rght , bestNode , bestDist ) # Current best from the right
                
            elif node.left: # Only the the left node exists, recur
                recurNode , recurDist = kd_Tree_Approx.NN_recur( testPoint , node.left , bestNode , bestDist )  # Current best from the left
                # No alternate branch to explore
            else: # Only the the right node exists, recur
                recurNode , recurDist = kd_Tree_Approx.NN_recur( testPoint , node.rght , bestNode , bestDist ) # Current best from the right
                # No alternate branch to explore
            # 2. The algorithm unwinds the recursion of the tree, performing the following steps at each node:
            #    a. If the current node is closer than the current best, then it becomes the current best.
            if recurDist < bestDist:
                bestDist = recurDist
                bestNode = recurNode
                
            #    b. The algorithm checks whether there could be any points on the other side of the splitting plane that are closer to 
            #       the search point than the current best. If the best distance cross the cutting line, then search the other subtree
            # radius = bestDist # The radius is the best distance so far
            if abs( node.state[ splitDex ] - testPoint[ splitDex ] ) <= bestDist and twoSides: # If the radius crosses the split and there is another side
                if leftBest:
                    recurNode , recurDist = kd_Tree_Approx.NN_recur( testPoint , node.rght , bestNode , bestDist ) # Current best from the right
                else:
                    recurNode , recurDist = kd_Tree_Approx.NN_recur( testPoint , node.left , bestNode , bestDist ) # Current best from the left
            
            # Check for the best again, the other side may or may not have been better
            if recurDist < bestDist:
                bestDist = recurDist
                bestNode = recurNode
            
        return bestNode , bestDist # vec_dif_mag( bestNode.state , testPoint ) # Return the closest node and the actual (not squared) distance
        
    def NN( self , testState ):
        """ Find the nearest neighbor to 'state' """
        bestNode = kd_Tree_Approx.NN_recur( testState , self.root )[0]
        return bestNode , vec_dif_mag( bestNode.state , testState )
        
    @staticmethod
    def k_NN_recur( testPoint , node , k , bestQueue ):
        """ Find the nearest neighbor to 'state' in the subtree below 'node' """
        # Base Case: Once the algorithm reaches a leaf node, it saves that node point as the "current best"
        if not node.left and not node.rght:
            bestQueue.push( node , vec_dif_sqr( node.state , testPoint ) )
            # return node , bestQueue # testDist if testDist < bestDist else bestDist
            
        # Recurive Case: the algorithm moves down the tree recursively, in the same way that it would if the search point were being inserted 
        #                 (i.e. it goes left or right depending on whether the point is lesser than or greater than the current node in the split dimension) 
        
        else: # else this is a non-leaf node
            # before we go any further, check how far this node is from the test point
            bestQueue.push( node , vec_dif_sqr( node.state , testPoint ) )
            
            # Now we can check successors   
            splitDex = node.splitDim # Get the splitting dimension
            twoSides = False
            leftBest = False
            
            # 1. Get the "current best" from the first leaf node encountered on the binary search
            if node.left and node.rght: # Both branches are present, compare on the split dimension
                twoSides = True
                if abs( node.left.state[ splitDex ] - testPoint[ splitDex ] ) <= abs( node.rght.state[ splitDex ] - testPoint[ splitDex ] ):
                    kd_Tree_Approx.k_NN_recur( testPoint , node.left , k , bestQueue ) # Current best from the left
                    leftBest = True
                else:
                    kd_Tree_Approx.k_NN_recur( testPoint , node.rght , k , bestQueue ) # Current best from the right
                
            elif node.left: # Only the the left node exists, recur
                kd_Tree_Approx.k_NN_recur( testPoint , node.left , k , bestQueue )  # Current best from the left
                # No alternate branch to explore
            else: # Only the the right node exists, recur
                kd_Tree_Approx.k_NN_recur( testPoint , node.rght , k , bestQueue ) # Current best from the right
                # No alternate branch to explore
            # 2. The algorithm unwinds the recursion of the tree, performing the following steps at each node:
            #    a. If the current node is closer than the current best, then it becomes the current best.
            #    b. The algorithm checks whether there could be any points on the other side of the splitting plane that are closer to 
            #       the search point than the current best. If the best distance cross the cutting line, then search the other subtree
            if len( bestQueue ) < k and ( abs( node.state[ splitDex ] - testPoint[ splitDex ] ) <= bestQueue.btm_priority() and twoSides ) or \
               ( abs( node.state[ splitDex ] - testPoint[ splitDex ] ) <= bestQueue.top_priority() and twoSides ): # If the radius crosses the split and there is another side
                if leftBest:
                    kd_Tree_Approx.k_NN_recur( testPoint , node.rght , k , bestQueue ) # Current best from the right
                else:
                    kd_Tree_Approx.k_NN_recur( testPoint , node.left , k , bestQueue ) # Current best from the left
            
        # Return nothing, just unspool the queue after the algo has finished
    
    def k_NN( self , state , k ):
        """ Return the 'k' nearest neighbors to 'state' """
        rtnQ = BPQwR( k )
        
        kd_Tree_Approx.k_NN_recur( state , self.root , k , rtnQ )
        rtnPnts , NN_dist = rtnQ.unspool()
        # print rtnPnts
        
        return rtnPnts , [ sqrt( d ) for d in NN_dist ]  
        
    @staticmethod
    def r_NN_recur( testPoint , node , r , bestQueue ):
        """ Find the nearest neighbor to 'state' in the subtree below 'node' """
        # Base Case: Once the algorithm reaches a leaf node, it saves that node point as the "current best"
        if not node.left and not node.rght:
            bestQueue.push( node , vec_dif_sqr( node.state , testPoint ) )
            # return node , bestQueue # testDist if testDist < bestDist else bestDist
            
        # Recurive Case: the algorithm moves down the tree recursively, in the same way that it would if the search point were being inserted 
        #                 (i.e. it goes left or right depending on whether the point is lesser than or greater than the current node in the split dimension) 
        
        else: # else this is a non-leaf node
            # before we go any further, check how far this node is from the test point
            bestQueue.push( node , vec_dif_sqr( node.state , testPoint ) )
            
            # Now we can check successors   
            splitDex = node.splitDim # Get the splitting dimension
            twoSides = False
            leftBest = False
            
            # 1. Get the "current best" from the first leaf node encountered on the binary search
            if node.left and node.rght: # Both branches are present, compare on the split dimension
                twoSides = True
                if abs( node.left.state[ splitDex ] - testPoint[ splitDex ] ) <= abs( node.rght.state[ splitDex ] - testPoint[ splitDex ] ):
                    kd_Tree_Approx.r_NN_recur( testPoint , node.left , r , bestQueue ) # Current best from the left
                    leftBest = True
                else:
                    kd_Tree_Approx.r_NN_recur( testPoint , node.rght , r , bestQueue ) # Current best from the right
                
            elif node.left: # Only the the left node exists, recur
                kd_Tree_Approx.r_NN_recur( testPoint , node.left , r , bestQueue )  # Current best from the left
                # No alternate branch to explore
            else: # Only the the right node exists, recur
                kd_Tree_Approx.r_NN_recur( testPoint , node.rght , r , bestQueue ) # Current best from the right
                # No alternate branch to explore
            # 2. The algorithm unwinds the recursion of the tree, performing the following steps at each node:
            #    a. If the current node is closer than the current best, then it becomes the current best.
            #    b. The algorithm checks whether there could be any points on the other side of the splitting plane that are closer to 
            #       the search point than the current best. If the best distance cross the cutting line, then search the other subtree
            if ( abs( node.state[ splitDex ] - testPoint[ splitDex ] ) <= r and twoSides ): # If the radius crosses the split and there is another side
                if leftBest:
                    kd_Tree_Approx.r_NN_recur( testPoint , node.rght , r , bestQueue ) # Current best from the right
                else:
                    kd_Tree_Approx.r_NN_recur( testPoint , node.left , r , bestQueue ) # Current best from the left
            
        # Return nothing, just unspool the queue after the algo has finished
    
    def r_NN( self , state , r ):
        """ Return all the nearest neighbors to 'state' that are within distance 'r' """
        # print "Entered r_NN!"
        rtnQ = LPQwR( r**2 ) # Using squared distance for the sake of efficiency
        
        kd_Tree_Approx.r_NN_recur( state , self.root , r**2 , rtnQ ) # Using squared distance for the sake of efficiency
        # rtnPnts = [ node.state for node in rtnQ.unspool()[0] ]
        rtnPnts , NN_dist = rtnQ.unspool()
        # print rtnPnts
        
        return rtnPnts , [ sqrt( d ) for d in NN_dist ]    
        
    def delete_node( self , delNode ):
        pass # unconderned with deletion at this time, it's probable that all the child nodes will have to be re-added because they will now be
        #      split on new dimensions

