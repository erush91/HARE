#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Template Version: 2017-06-18

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

"""@module MeshVFN.py
@brief Mesh processing in Python

James Watson, 2017 October
Built on Spyder for Python 2.7

Dependencies: numpy , numpy-stl (stl) , marchhare
"""

# === Init =================================================================================================================================

# ~~ Imports ~~
# ~ Standard ~
import os , operator
from math import pi , radians
from random import choice
from warnings import warn
# ~ Special ~
from stl import mesh # https://pypi.python.org/pypi/numpy-stl/ # pip install numpy-stl
from scipy.spatial import ConvexHull
import numpy as np
# ~ Local ~
from marchhare import tandem_sorted , incr_max_step , iter_contains_None 
from MathKit import round_small
from Vector import vec_avg , matx_zeros , vec_angle_between , vec_unit # , matx_2D_pretty_print
from VectorMath.Vector2D import point_in_poly_w , d_point_to_segment_2D_signed
from VectorMath.Vector3D import point_basis_change , transform_to_frame , pnt_proj_to_plane
from Graph import TaggedLookup

# ~~ Constants , Shortcuts , Aliases ~~
EPSILON = 1e-7 # ------ Assume floating point errors below this level
infty   = 1e309 # ----- URL: http://stackoverflow.com/questions/1628026/python-infinity-any-caveats#comment31860436_1628026
endl    = os.linesep #- Line separator
pyEq    = operator.eq # Default python equality
piHalf  = pi/2

# ___ End Init _____________________________________________________________________________________________________________________________


# === Basic Mesh Geo =======================================================================================================================

def tri_normal( p0 , p1 , p2 ):
    """ Return the unit normal vector for a triangle with points specified in CCW order """
    vec1 = np.subtract( p1 , p0 )
    vec2 = np.subtract( p2 , p0 )
    return vec_unit( np.cross( vec1 , vec2 ) )

# ___ End Mesh Geo _________________________________________________________________________________________________________________________


# === Regrasp Code =========================================================================================================================

## STL_to_mesh_obj
# @brief Return a 'Mesh' object produced from the STL file at 'STLpath'
# @param STLpath - Path to an STL file to be processed
def STL_to_mesh_obj( STLpath ):
    return mesh.Mesh.from_file( STLpath )

# == numpy-stl helpers ==
# These functions deal with the 'Mesh' objects produced by numpy-stl
    
## mesh_verts_to_list
# @brief Return all mesh points in a single list , NOTE: and STL file is guaranteed to have many redundant points
# @param pMesh - 'mesh.Mesh' object , as returned from 'STL_to_mesh_obj'
def mesh_verts_to_list( pMesh ):
    pointsOnly = []
    # Each facet has three vertices , each of which can be found in one of three lists at the same index
    for vList in [ pMesh.v0 , pMesh.v1 , pMesh.v2 ]: # for each collection of points
        for point in vList: # for each point in the collection , append the point
            pointsOnly.append( point ) # The point is unique, add it to the unique list
    return pointsOnly

## len_mesh
# @brief Return the number of facets in the mesh
# @param pMesh - 'mesh.Mesh' object , as returned from 'STL_to_mesh_obj'
def len_mesh( pMesh ):
    return len( pMesh.normals )

## tri_tuple_from_mesh
# @brief Return a tuple of ( normal , Vertex0 , Vertex1 , Vertex2 ) at 'index'
# @param pMesh  - 'mesh.Mesh' object , as returned from 'STL_to_mesh_obj'
# @param index - index of the facet to access , if it is beyond the number of vertices , throws 'IndexError'
def tri_tuple_from_mesh( pMesh , index ):
    if index <= len_mesh( pMesh ) - 1:
        return ( pMesh.normals[ index ] , pMesh.v0[ index ] , pMesh.v1[ index ] , pMesh.v2[ index ] )
    else:
        raise IndexError( "tri_tuple: Index " + str( index ) + " was out of bounds! Size " + str( len_mesh( pMesh ) ) if pMesh else "UNKNOWN" )

## tri_centers_from_mesh
# @brief Return a list of the geometric centers of each of the facets in the mesh
# @param pMesh  - 'mesh.Mesh' object , as returned from 'STL_to_mesh_obj'
def tri_centers_from_mesh( pMesh ):
    """ Return the center of a triangle, the average position of its three vertices """
    rtnPnts = []
    for i in xrange( len( pMesh.normals ) ): 
        rtnPnts.append( vec_avg( pMesh.v0[i] , pMesh.v1[i] , pMesh.v2[i]) )
    return rtnPnts

# __ End numpy-stl __

## stl_to_verts_list
# @brief Return all STL vertices in a single list
# @param STLpath - Path to an STL file to be processed
def STL_to_verts_list( STLpath ):
    return mesh_verts_to_list( STL_to_mesh_obj( STLpath ) )

## uniq_pts_from_STL
# @brief Return a set of unique points extracted from an STL file and store them as a list
# @param STLpath - Path to an STL file to be processed
def uniq_pts_from_STL( STLpath ):
    return [ list( coord ) for coord in list( set( [ tuple( elem ) for elem in mesh_verts_to_list( STL_to_mesh_obj( STLpath ) ) ] ) ) ]

## AABB_3D
# @brief Given a 'R3PointsList', return a nested tuple of the 3D Axis-Aligned Bounding Box ( (xMin,xMax) , (yMin,yMax) , (zMin,zMax) )
# @param R3PointsList - list of R3 points [ x , y , z ] 
def AABB_3D( R3PointsList ):
    xMin =  infty
    xMax = -infty
    yMin =  infty
    yMax = -infty
    zMin =  infty
    zMax = -infty
    
    for point in R3PointsList:
        # x extent
        if point[0] < xMin:
            xMin = point[0]
        if point[0] > xMax:
            xMax = point[0]
        # y extent    
        if point[1] < yMin:
            yMin = point[1]
        if point[1] > yMax:
            yMax = point[1]
        # z extent
        if point[2] < zMin:
            zMin = point[2]
        if point[2] > zMax:
            zMax = point[2]
            
    return ( ( xMin , xMax ) , ( yMin , yMax ) , ( zMin , zMax ) )

## AABB_2D
# @brief Given a 'R2PointsList', return a nested tuple of the 2D Axis-Aligned Bounding Box ( (xMin,xMax) , (yMin,yMax) )
# @param R3PointsList - list of R3 points [ x , y , z ] 
def AABB_2D( R2PointsList ):
    xMin =  infty
    xMax = -infty
    yMin =  infty
    yMax = -infty
    
    for point in R2PointsList:
        # x extent
        if point[0] < xMin:
            xMin = point[0]
        if point[0] > xMax:
            xMax = point[0]
        # y extent    
        if point[1] < yMin:
            yMin = point[1]
        if point[1] > yMax:
            yMax = point[1]
            
    return ( (xMin,xMax) , (yMin,yMax)  )

## span_3D
# @brief Given a 3D AABB ( (xMin,xMax) , (yMin,yMax) , (zMin,zMax) ) , Return the extent along each axis
# @param bbox3D - R3 Axis-Aligned Bounding Box ( (xMin,xMax) , (yMin,yMax) , (zMin,zMax) )
def span_3D( bbox3D ):
    return tuple( [ abs(bbox3D[0][1] - bbox3D[0][0]) , abs(bbox3D[1][1] - bbox3D[1][0]) , abs(bbox3D[2][1] - bbox3D[2][0]) ] )

## span_2D
# @brief Given a 2D AABB ( (xMin,xMax) , (yMin,yMax) ) , Return the extent along each axis
# @param bbox2D - R2 Axis-Aligned Bounding Box ( (xMin,xMax) , (yMin,yMax) )
def span_2D( bbox2D ):
    return tuple( [ abs( bbox2D[0][1] - bbox2D[0][0] ) , abs( bbox2D[1][1] - bbox2D[1][0] ) ] )

## span_3D_from_pts
# @brief Given a list of R3 points , Return the extent along each axis
# @param R3PointsList - list of R3 points [ x , y , z ] 
def span_3D_from_pts( R3PointsList ):
    return span_3D( AABB_3D( R3PointsList ) )

## centroid_discrete_masses
# @brief Return the COM for a collection of point masses 'massCenters' [ mass_i , [ x_i , y_i , ... ] ] , with 'totalMass' if known
# @param massCenters - List of point-mass pairs taking the form [ ... , [ mass_i , [ x_i , y_i , ... ] ] , ... ]
# @param totalMass   - Mass equal to the sum of the mass elements in 'massCenters'
def centroid_discrete_masses( massCenters , totalMass = 0 ):
    if totalMass == 0: # if the user did not calculate the total mass for us , then do so
        for [ mass_i , center_i ] in massCenters:
            totalMass += mass_i
    centroid = [ 0 for i in xrange( len( massCenters[0][1] ) ) ] # build a zero vector the same dimensionality as the data coords
    for [ mass_i , center_i ] in massCenters: # for every mass-point pair in the data
        for i , coord in enumerate( center_i ): # for every coordinate in the point
            centroid[i] += mass_i / totalMass * coord # Add the coordinate scaled by it's mass distribution
    return centroid 

## bounding_points_from_hull
# @brief Given a scipy.spatial.ConvexHull , return a list of the (supposedly) bounding points
# helper function for 'volume_centroid_of_points_convex'
# @param hull - hull object returned by scipy.spatial.ConvexHull
def bounding_points_from_hull( hull ):
    # NOTE: Somehow the ConvexHull was not so convex?
    pntList = []
    # ConvexHull.vertices contains the indices of ConvexHull.points that are the bounding poly
    for index in hull.vertices: 
        pntList.append( hull.points[ index ] )
    return pntList

## volume_centroid_of_points_convex
# @brief Considering a point cloud 'R3PointsList' to be the convex boundary of a 3D object , return the centroid , assuming uniform desnity
# NOTE: WARNING OLD , This is largely obsolete due to 'COM_volume_from_mesh' , See below
# @param R3PointsList - list of points in R3 describing the extent of a convex , solid object
# @param [numSlices]  - Number of slices to consider in the z direction [optional]
def volume_centroid_of_points_convex( R3PointsList , numSlices = 100 ):
    # numSlices: number of z slices to partition point cloud into
    
    # Establish the extent of the z-axis that the model spans, 'zMin' to 'zMax'
    zMin =  infty
    zMax = -infty
    for center in R3PointsList:
        if center[2] < zMin:
            zMin = center[2]
        if center[2] > zMax:
            zMax = center[2]

    # Init z slices of model
    zSpan = abs( zMax - zMin ) # z extent that the model spans
    sliceThknss = zSpan * 1.0 / numSlices # the z thickness of each slice
    zSlices = [ [] for i in xrange( numSlices ) ]
    sliceCount = [ 0 for i in xrange( numSlices ) ]

    # Flatten all the points found in each z-slize with 'sliceThknss' onto a plane
    loopCount = 0
    for center in R3PointsList:
        # Find the index of the slice that this point belongs to
        index = int( abs( center[2] - zMin ) / sliceThknss ) # 'int' rounds down to the next integer
        if index >= numSlices:
            index = numSlices - 1
        #index = abs(center[2] - zMin) / sliceThknss
        #print index,center[:-1],'\t',
        zSlices[ index ].append( center[:-1] ) # store only the x,y coordinates of the point in the slice
        #zSlices[index].append( index ) # store only the x,y coordinates of the point in the slice
        sliceCount[ index ] += 1
        loopCount += 1
 
    # form the convex hull of each slice
    sliceBounds = []
    #index = 0
    for index, zSlc in enumerate( zSlices ):
        sliceBounds.append( [] )
        try:
            sliceHull = ConvexHull( zSlc ) # zSlices[index] )#, qhull_options='Qm' ) # Not 100% certain 'Qm' is needed?
            sliceBounds[-1] = bounding_points_from_hull( sliceHull )
        except Exception as err:
            print "Encountered" , type( err ).__name__ , "on index", index , "with args:", err.args,"with full text:",str( err )
        #index += 1

    # Compute the cross sectional area, compute the center of the slice
    slcCentroids = []
    slcTotalArea = 0
    for bndDex, bound in enumerate( sliceBounds ):
        if len( bound ): # If our convex hull was succesfull
            A = 0
            triCentroids = list() # []
            totArea = 0
            for pntDex in range( 2 , len( bound ) ):
                # http://www.mathopenref.com/coordtrianglearea.html
                # Assume that we are dealing with convex polygons (hulls), otherwise use shoelace algorithm
                Ax = bound[ pntDex   ][ 0 ]
                Bx = bound[ pntDex-1 ][ 0 ]
                Cx = bound[ 0        ][ 0 ]
                Ay = bound[ pntDex   ][ 1 ]
                By = bound[ pntDex-1 ][ 1 ]
                Cy = bound[ 0        ][ 1 ]
                A = 0.5 * abs( Ax*(By-Cy) + Bx*(Cy-Ay) + Cx*(Ay-By) )
                totArea += A
                triCentroids.append( ( A , vec_avg( bound[ pntDex ] , bound[ pntDex-1 ] , bound[0] ) ) )
            slcTotalArea += totArea
            flatCentroid = centroid_discrete_masses( triCentroids ,totArea )
            slcCentroids.append( ( totArea , [ flatCentroid[0] , flatCentroid[1] , zMin + sliceThknss/2 + bndDex * sliceThknss ] ) )
        else: # Else produce a slice of zero area, and assume there wille enough good slices to produce a volume centroid
            slcCentroids.append( ( 0 , [ 0 , 0 , 0 ] ) )
        #print "Calculated an area of",totArea,"for this slice"
        
    return centroid_discrete_masses( slcCentroids , slcTotalArea )

## CCW_tri_to_bases
# @brief Compute local basis vectors for a triangle defined by CCW vertices , including normal
# @param v0 - R3 , 1st vertex
# @param v1 - R3 , 2nd vertex
# @param v2 - R3 , 3rd vertex
def CCW_tri_to_bases( v0 , v1 , v2 ):
    origin = v0 
    xBasis = vec_unit( np.subtract( v1 , v0 ) )
    vecB   = vec_unit( np.subtract(  v2 , v0     ) )
    zBasis = vec_unit( np.cross( xBasis , vecB ) ) 
    yBasis = vec_unit( np.cross( zBasis , xBasis ) )
    return origin , xBasis , yBasis , zBasis
    
## mesh_facets_to_matx_V_F
# @brief Compress the facet information from the 'mesh.Mesh' object into vertex and facet matrices similar to what you might expect for OpenGL
# V - N x 3 (float) matrix in which each row is a unique point in the mesh , output similar to 'uniq_pts_from_STL'
# F - M x 3 (int)   matrix in which each row is a list of indices of 'V' that comprise the facet , right-hand [v0,v1,v2] order is preserved
# @param pMesh  - 'mesh.Mesh' object , as returned from 'STL_to_mesh_obj'
def mesh_facets_to_matx_V_F_N( pMesh ):
    return tri_list_to_matx_V_F_N( pMesh.v0 , pMesh.v1 , pMesh.v2 , pMesh.normals )
    
## tri_list_to_matx_V_F_N
# @brief Compress the facet information from the 'mesh.Mesh' object into vertex and facet matrices similar to what you might expect for OpenGL
# V - N x 3 (float) matrix in which each row is a unique point in the mesh , output similar to 'uniq_pts_from_STL'
# F - M x 3 (int)   matrix in which each row is a list of indices of 'V' that comprise the facet , right-hand [v0,v1,v2] order is preserved
# @param pMesh  - 'mesh.Mesh' object , as returned from 'STL_to_mesh_obj'
def tri_list_to_matx_V_F_N( v0List , v1List , v2List , normals = None ):
    V = [] ; F = [] ; N = [] # Vertices , faces , normals
    vertexSet = set([]) # Set of all unique vertices
    haveNs = True if isinstance( normals , ( list , np.ndarray ) ) else False
    for i in xrange( len( v0List ) ): 
        f_i = []
        tri = [ v0List[i] , v1List[i] , v2List[i] ]
        for vertex in tri:
            # 1. Cast the vertex as a tuple
            temp = tuple( vertex )
            # 2. Test if we have seen this vertex before
            if temp in vertexSet:
                # 2.1. If we have encounted this vertex before , find it in the list of vertices , and note the index to populate F
                f_i.append( V.index( temp ) )
            else:
                # 2.2. If we have not encounted this vertex before , append it to the list / set of vertices , and populate F
                vertexSet.add( temp )
                V.append( temp )
                f_i.append( len( V ) - 1 )
        F.append( f_i )
        if not haveNs:
            origin , xBasis , yBasis , zBasis = CCW_tri_to_bases( *tri )
            N.append( zBasis )
        else:
            N.append( normals[i] )
    return [ list( elem ) for elem in V ] , F , N # Cast the vertices to lists and return them and the faces ( already nested lists )

## facet_adjacency_matx
# @brief Construct an adjacency matrix for all facets , NOTE: Adjacency is defined by sharing a side , sharing one vertex is not adjacent
# @param F - Matrix of facets in which each row is a list of indices that comprise a facet , as produced by 'mesh_facets_to_matx_V_F'
def facet_adjacency_matx( F ):
    N          = len( F ) # Number of facets in the mesh 
    dim        = 3                   # Number of dimensions
    shareCount = 0                   # Number of shared vertices
    nghbrCount = 0                   # Number of 
    rtnMatx    = matx_zeros( N , N )
    
    for i in xrange( N - 1 ):
        nghbrCount = 0; # In this formulation there can only be 3 neighbors
        for j in xrange( i + 1 , N ):
            shareCount = 0;
            for k in xrange( dim ):
                for m in xrange( dim ):
                    if F[i][k] == F[j][m]:
                        shareCount += 1;
                if( shareCount > 1 ): 
                    break; # If the facets share two vertices , they are adjacent , stop searching
            if( shareCount > 1 ): #{ // If the facets are adjacent , increment neighbor count and mark adjacency in the matrix
                nghbrCount += 1;
                rtnMatx[i][j] = 1; # i is adjacent to j , which means that
                rtnMatx[j][i] = 1; # j is adjacent to i   also
            if( nghbrCount > 2 ): 
                break; # If there are 3 neighbors , we are at the max , stop searching
    return rtnMatx;
    
## facet_adjacency_list
# @brief Given a facet vertex lookup matrix 'F' , find all of the side-sharing neighbors of each facet
# @param F - Matrix of facets in which each row is a list of indices that comprise a facet , as produced by 'mesh_facets_to_matx_V_F'
def facet_adjacency_list( F ):
    N = len( F )
    neighborList = [ [] for i in xrange( N ) ]
    matx = facet_adjacency_matx( F ) # Get the adjacency matrix , This is where the hard work is done
    totalNeighbors = 0
    for i in xrange( N - 1 ):
        for j in xrange( i + 1 , N ):
            if matx[i][j] == 1:
                totalNeighbors += 1
                neighborList[i].append( j );
                neighborList[j].append( i ); 
    return neighborList

## facet_adjacency_list_ordered
# @brief Given a facet vertex lookup matrix 'F' , find all of the side-sharing neighbors of each facet , ORDERED VERSION
# Return a list of a lists , len( neighborList ) == len( F ) , such that each is a list of neighbors to the facet of the corresponding index
# Each of the lists are ordered so that the neighbors at indices correspond to a specific border edge
# 0 : [ v0 , v1 ]  ,  1 : [ v1 , v2 ]  ,  2 : [ v2 , v0 ]
# If the edge at an index does not form a border , then the value at that position is 'None'
# NOTE: This function assumes that [ v0 , v1 , v2 ] , are ordered by the Right Hand Rule with an outward-facing normal , in proper STL they will
# NOTE: This function does not assume that all facets will have 3 edge-neighbors , but in a properly enclosed STL , they will
# @param F - Matrix of facets in which each row is a list of indices that comprise a facet , as produced by 'mesh_facets_to_matx_V_F'
def facet_adjacency_list_ordered( F ):
    indices_RH = [ ( 0 , 1 ) , ( 1 , 2 ) , ( 2 , 0 ) ] # Right-hand indices
    indices_LH = [ ( 1 , 0 ) , ( 2 , 1 ) , ( 0 , 2 ) ] # Left-hand  indices , Neighbor will see vertices of the border segment in reverse order
    N = len( F ) # Number of facets
    neighborList = [ [ None , None , None ] for i in xrange( N ) ] # Ordered list of neighbors ( neighbor pointers )
    
    for i in xrange( N - 1 ):
        # Copy right-hand edges for each neighbor position that has not yet been associated with a neighbor
        neighbors = [ indices_RH[ checkDex ] if neighborList[i][ checkDex ] == None else None for checkDex in xrange(3) ] # Avoid repeat search
        for j in xrange( i + 1 , N ): # For each unique pairing of facets ( i , j )
            for nDex , neighbor in enumerate( neighbors ):
                if neighbor != None: # If we have not yet located the neighbor for this edge , Check needed to avoid repeats , see above
                    for nnDex , neighborNeighbor in enumerate( indices_LH ):
                        # nDex : This facet's edge  ,  nnDex : Candidate neighbor edge
                        if neighborList[ j ][ nnDex ] == None:
                            match = True
                            for pairDex in xrange( 2 ): # For each of the points that form the candidate border
                                # Unless this edge has two vertices that are in reverse order of the other facet edge , this edge is not the 
                                #  border between the two
                                if F[ i ][ neighbor[ pairDex ] ] != F[ j ][ neighborNeighbor[ pairDex ] ]:
                                    match = False
                            if match:
                                neighborList[ i ][ nDex  ] = j # Mark the located neighbor
                                neighbors[ nDex ] = None # Mark this neighbor as found
                                neighborList[ j ][ nnDex ] = i # This facet is the neighbor's neighbor at its identified border
                            # else no match , no action , continue
                        # else the neighbor already has a match for this neighbor-nerighbor
                # else 'neighbor' is Null , we found this neighbor already and there is no action
    return neighborList
                        
## cluster_mesh
# @brief Cluster the facets into roughly flat clusters
# In order to establish consistency in cluster bookkeeping , the list of clusters returned is sorted in increasing order by the least facet index contained
# @param pMesh      - 'mesh.Mesh' object , as returned from 'STL_to_mesh_obj'
# @param CRITRN_ANG - The maximum angle between the facet and the cluster in order for the facet to be a member of the cluster
def cluster_mesh( pMesh , CRITRN_ANG ): # returns clusters , clusterNorms
    V , F , N = mesh_facets_to_matx_V_F_N( pMesh ) # ----- 1. Fetch the reduced vertices and facets
    # N     = pMesh.normals[:] # --------------------- 2. Fetch the normals for each of the facets
    return cluster_V_F_N( V , F , N , CRITRN_ANG ) # 3. Call the helper function

## cluster_V_F_N
# @brief Cluster the facets into roughly flat clusters
# NOTE: This function assumes that the number of vertices has already been reduced
# In order to establish consistency in cluster bookkeeping , the list of clusters returned is sorted in increasing order by the least facet index contained
# This function was split from 'cluster_mesh' for the case that { V , F , N } were already calculated
# @param V          - N x 3 (float) matrix in which each row is a unique point in the mesh , output similar to 'uniq_pts_from_STL'
# @param F          - M x 3 (int)   matrix in which each row is a list of indices of 'V' that comprise the facet , right-hand [v0,v1,v2] 
# @param N          - List of normal vectors corresponding to F
# @param CRITRN_ANG - The maximum angle between the facet and the cluster in order for the facet to be a member of the cluster [rad]
def cluster_V_F_N( V , F , N , CRITRN_ANG ):
    
    numRows   = len( F ) # Total number of facets in the mesh
    currFacet = 0 # ------ Current facet under scrutiny
    j         = 0 # ------ Index of the cluster currently being built
    
    facetQueue = [] # ----------------------- Queue of facet neighbors that need to be checked for cluster membership
    unusedFacets = set( range( numRows ) ); # Set of facet indices that have not been clustered
    faceLists = []; # ----------------------- List of vectors of facet indices that comprise each cluster
    faceNormals = []; # --------------------- normals of the faces
    adj_lst = facet_adjacency_list( F ); # -- 3. List of facets that are adjacent to each facet
    
    # 4. While there are unused facets , choose a facet to nucleate a cluster , then expand neighbors , gathering a flat region as it expands
    while( len( unusedFacets ) > 0 ): 
        
        # Pop facet , this nucleates a new cluster
        currFacet = choice( list( unusedFacets ) ) # Choose a seed facet at random from the set up unused facets
        unusedFacets.remove( currFacet ); # -------- Remove that facet from the list of unused facets
        
        # create new face with facet
        faceLists.append( [ currFacet ] ); # ----- Add the new group to the face vector
        faceNormals.append( N[ currFacet ][:] ); # Add the face normal
        
        for i in xrange( len( adj_lst[ currFacet ] ) ): # for each neighbor
            if adj_lst[ currFacet ][ i ] in unusedFacets: # if the neighbor is unused , enqueue
                facetQueue.append( adj_lst[ currFacet ][ i ] );
                
        # while the queue is non-empty , pop facets and check each for cluster match , if there is a match , enque its neighbors for checking
        while( len( facetQueue ) > 0 ): 
            currFacet = facetQueue.pop(0); # pop facet
            
            # test angle
            if vec_angle_between( N[ currFacet ] , faceNormals[ j ] ) <= CRITRN_ANG: # angle pass
                # add the facet to the face
                faceLists[j].append( currFacet );
                unusedFacets.remove( currFacet ); # mark the facet as used
                # Calc a new average normal for this cluster
                # Multiply the current vector times the previous number of members , add the new normal , divide by the new number of vectors
                faceNormals[j] = np.divide( np.add( np.multiply( faceNormals[j] , len( faceLists[j] ) - 1  ) , 
                                                    N[ currFacet ] ) , 
                                            len( faceLists[j] ) );
                faceNormals[j] = vec_unit( faceNormals[j] );
                # for each neighbor
                for i in xrange( len( adj_lst[ currFacet ] ) ):
                    # if the neighbor is unused , enqueue
                    if adj_lst[ currFacet ][ i ] in unusedFacets:
                        facetQueue.append( adj_lst[ currFacet ][ i ] )
            else: # angle fail : NO OP
                pass
        j += 1; # Advance to the next cluster
    minList = [ min( cluster ) for cluster in faceLists ]
    sortedLists = tandem_sorted( minList , faceLists , faceNormals )
    #      cluster facets , normals
    return sortedLists[1] , sortedLists[2]
    
## get_outside_segments_from_mesh_subset
# @brief Given a list of ordered neighbors in F , and a list of facets in F , return a list of line segments that form the boundary of the sublist
# @param V ----------------- N x 3 (float) matrix in which each row is a unique point in the mesh , output similar to 'uniq_pts_from_STL'
# @param F ----------------- M x 3 (int)   matrix in which each row is a list of indices of 'V' that comprise the facet , right-hand [v0,v1,v2] 
# @param orderedNeighborsF - A list of ordered neighbors as produced by 'facet_adjacency_list_ordered'
# @param facetSublist ------ List of facets in F representing one facet or cluster 
def get_outside_segments_from_mesh_subset( V , F , orderedNeighborsF , facetSublist ):
    indices_RH = [ ( 0 , 1 ) , ( 1 , 2 ) , ( 2 , 0 ) ] # Right-hand indices
    rtnList = [] # ------------------ List of segments to return
    regionSet = set( facetSublist ) # Set of all the facets in the region under consideration
    # 1. For each facet
    for i , neighborList in enumerate( orderedNeighborsF ):
        # 2. Check that the facet is in the sublist under consideration
        if i in sorted( list( regionSet ) ):
            # 3. If it is , fetch a list of that facet's neighbors ( already fetched as 'neighborList' )
            # 4. For each neighbor , check ( if the neighbor is unoccupied || if the neighbor DNE in the sublist )
            for j , neighbor in enumerate( neighborList ):
                if ( neighbor == None ) or ( neighbor not in regionSet ):
                    # 5. If the above is met , then this neighbor position is an outside border
                    # 6. Fetch a right-hand ordered ( CCW ) segment that represents this border , and append it to the list to be returned
                    pairDices = indices_RH[ j ]
                    segment = []
                    for borderDex in pairDices: # For each of the vertices that make up the border
                        # Fetch the element of V that corresponds to the end of the segment that corresponds to the border segment
                        segment.append( tuple( V[ F[ i ][ borderDex ] ] ) )
                    rtnList.append( tuple( segment ) )
    return rtnList
    
## polygon_from_RH_border_segments
# @brief Reorder 'segmentList' into a CCW cycle , Warn if this does not form a contiguous region
# @param segmentList - List of border segments as produced by 'get_outside_segments_from_mesh_subset'
def polygon_from_RH_border_segments( segmentList ):
    segCpy = segmentList[:] # Copy the segment list to safely modify
    # rtnSeq = [ segCpy.pop( choice( range( len( segCpy ) ) ) ) ] # pop a random starting segment for the polygon
    rtnSeq = [ segCpy.pop( 0 ) ] # pop the first segment for the polygon , Always pop this one for consistency
    
    def pop_segment_with_bgn( segList , bgnPnt ):
        for i , segment in enumerate( segList ):
            if segment[0] == bgnPnt:
                return segList.pop( i ) # I know its bad to modify a list that is being iterated over , but search stops here
        return None
    
    while( len( segCpy ) ):
        bgnPnt = rtnSeq[-1][1]
        result = pop_segment_with_bgn( segCpy , bgnPnt )
        if result == None:
            break
        rtnSeq.append( result )
    
    if len( segCpy ) > 0:
        warn( "polygon_from_RH_border_segments: Could not construct a simple polygon using all segments" )
        
    if rtnSeq[0][0] != rtnSeq[-1][1]:
        warn( "polygon_from_RH_border_segments: Could not construct a closed polygon" )
        
    return rtnSeq

## flatten_cluster_border
# @brief Flatten the 3D cluster border to a collection of 2D segments 
# @param orderedSegments - Collection of 3D segments ( ( x1 , y1 , z1 ) , ( x2 , y2 , z2 ) )
# @param clusterNormal   - Normal vector associated with this cluster
def flatten_cluster_border( orderedSegments , clusterNormal ):
    """ Flatten a cluster to an XY plan perpendicular to the """
    zBasis = vec_unit( clusterNormal )
    # 1. Cluster origin is the first point of the first segment in the sequence
    origin = orderedSegments[0][0]
    # 2. The X basis is the first segment in the sequence , Note this is not necessarily orthogonal to normal for a lumpy cluster!
    xTempV = vec_unit( np.subtract( orderedSegments[0][1] , orderedSegments[0][0] ) )
    # 3. Calc the Y basis
    yBasis = vec_unit( np.cross( zBasis , xTempV ) )
    # 4. Enforce orthonormal on 'xBasis'
    xBasis = vec_unit( np.cross( yBasis , zBasis ) )
    # 5. Express all of the points in the new basis and Project the points onto the flat plane
    rtnSegments = []
    for segment in orderedSegments:
        sgmnt2D = [ ]
        for endPnt in segment:
            point = point_basis_change( endPnt , origin , xBasis , yBasis , zBasis )
            sgmnt2D.append( tuple( [ point[0] , point[1] ] ) )
        rtnSegments.append( tuple( sgmnt2D ) )
    # 6. Return sufficent information to reconstruct the cluster in the object frame
    return { 'origin'  : tuple( origin )       ,
             'xBasis'  : tuple( xBasis )       ,
             'yBasis'  : tuple( yBasis )       ,
             'zBasis'  : tuple( zBasis )       ,
             'segments': tuple( rtnSegments  ) }
    
## clusters_to_egdes
# @brief Extract cluster borders from all of the clusters formed for a mesh
# @param V ----------------- N x 3 (float) matrix in which each row is a unique point in the mesh , output similar to 'uniq_pts_from_STL'
# @param F ----------------- M x 3 (int)   matrix in which each row is a list of indices of 'V' that comprise the facet , right-hand [v0,v1,v2] 
# @param orderedNeighborsF - neighbor list as returned by 'facet_adjacency_list_ordered'
# @param clusters ---------- Clusters of facets as returned by 'cluster_mesh'
# @param clusterNorms ------ Normal vectors associated with each of the 'clusters' , in the same order
def clusters_to_egdes( V , F , orderedNeighborsF , clusters , clusterNorms ):
    """ Given a collection of clusters and their normals , Return a corresponding collection of flat polygons representing cluster edges """
    rtnPolys = []
    for clusDex , cluster in enumerate( clusters ):
        rtnPolys.append(
            flatten_cluster_border( 
                polygon_from_RH_border_segments( 
                    get_outside_segments_from_mesh_subset( V , F , orderedNeighborsF , cluster )
                ) , 
                clusterNorms[ clusDex ] 
            )
        )
    return rtnPolys

## clusters_borders_to_polygons
# @brief Extract the vertices from cluster edge segments , and return border polygons with geometry information
# @param borderList - List per-cluster egde segment collections , as produced by 'clusters_to_egdes'
def clusters_borders_to_polygons( borderList ):
    # 1. For each cluster produced by clusters_to_egdes
    for flatBorder in borderList:
        # 2. Turn the list of segments into a CCW list of points = polygon , save
        allPts = [ segment[0] for segment in flatBorder['segments'] ]
        # allPts = allPts + [ allPts[0] ]
        flatBorder['polygon2D'] = tuple( allPts )
        poly = flatBorder['polygon2D']
        # 2.5. Transform the polygon to 3D
        origin   = flatBorder['origin']
        xBasis_B = flatBorder['xBasis'] ; yBasis_B = flatBorder['yBasis'] ; zBasis_B = flatBorder['zBasis']
        poly3D = []
        for point in poly:
            poly3D.append( tuple( transform_to_frame( [ point[0] , point[1] , 0 ] , origin , xBasis_B , yBasis_B , zBasis_B ) ) )
        flatBorder['polygon3D'] = tuple( poly3D )
    return borderList # This isn't strictly necessary

## tile_clusters_with_trial_points
# @brief Return a list of clusters that have been tiled with a rectangular grid of 2D points
# @param borderList - Cluster borders as returned by 'clusters_to_egdes'
# @param spacing ---- [optional] Grid spacing for trial points , If no value provided grid spacing will be 10 units per longest AABB extent
def tile_clusters_with_trial_points( borderList , spacing = None ):
    
    # 1. For each cluster produced by clusters_to_egdes
    for flatBorder in borderList:
        # 2. Turn the list of segments into a CCW list of points = polygon , save
        allPts = [ segment[0] for segment in flatBorder['segments'] ]
        # allPts = allPts + [ allPts[0] ]
        flatBorder['polygon2D'] = tuple( allPts )
        poly = flatBorder['polygon2D']
        # 2.5. Transform the polygon to 3D
        origin   = flatBorder['origin']
        xBasis_B = flatBorder['xBasis'] ; yBasis_B = flatBorder['yBasis'] ; zBasis_B = flatBorder['zBasis']
        poly3D = []
        for point in poly:
            poly3D.append( tuple( transform_to_frame( [ point[0] , point[1] , 0 ] , origin , xBasis_B , yBasis_B , zBasis_B ) ) )
        flatBorder['polygon3D'] = tuple( poly3D )
        # 3. Get the 2D bounding box for the polygon , save
        flatBorder['AABB'] = AABB_2D( flatBorder['polygon2D'] )
        AABB = flatBorder['AABB']
        xMin = AABB[0][0] ; xMax = AABB[0][1]
        yMin = AABB[1][0] ; yMax = AABB[1][1]
        # 3.5. If there was no spacing defined , determine spacing depending on the scale of the cluster
        if not spacing:
            spacing = max( span_2D( AABB ) ) * 1.0 / 10.0
        # 4. Get X divisions , for every X division
        trialPts = []
        for coordX in incr_max_step( xMin , xMax , spacing )[  1 : -1 ]:
            # 5. Get Y divisions , for every Y division
            for coordY in incr_max_step( yMin , yMax , spacing )[  1 : -1 ]:
                point = tuple( [ coordX , coordY ] )
                # 6. test for point in poly
                if point_in_poly_w( point , poly ):
                    # 7. If passed , add to collection of trial points
                    trialPts.append( point )
                # else point not in poly , discard
        # 8. Save the collection of trial points
        flatBorder['trialPoints2D'] = tuple( trialPts )
        # 9. Transform all of the points into 3D
        pts2D = flatBorder['trialPoints2D']
        pts3D = []
        for point in pts2D:
#            pnt = tuple( transform_by_bases( [ point[0] , point[1] , 0 ] , xBasis_B , yBasis_B , zBasis_B ) )
#            print pnt
            pts3D.append( tuple( transform_to_frame( [ point[0] , point[1] , 0 ] , origin , xBasis_B , yBasis_B , zBasis_B ) ) )
        flatBorder['trialPoints3D'] = tuple( pts3D )
            
    return borderList # This isn't strictly necessary

## COM_volume_from_mesh
# @brief Return an estimate of the volume and the center of mass (COM) the mesh , This version works for nonconvex shapes
# An incomplete implementation of this function is avaiable in "spare_parts.py"    
# @param pMesh - 'mesh.Mesh' object , as returned from 'STL_to_mesh_obj'
def COM_volume_from_mesh( pMesh ):
    volume , cog , inertia = pMesh.get_mass_properties()
    return cog , volume

## simplify_points
# @brief Simplify points 'V' , return voxel centers of all voxels ( with spacing 'div' ) that contain one or more V[i]
# @param V --- N x 3 (float) matrix in which each row is a unique point in the mesh , output similar to 'uniq_pts_from_STL'
# @param div - Voxel size to simply the mesh points [ if 'None' , spacing will be such there are 20 divisions in shortest dimension ]
def simplify_points( V , div = None ):
    cloud = PointCloud() # Instantiate a point cloud data object
    cloud.from_list( V ) # populate the point cloud object with points from the STL
    if div == None:
        cloudSpan = span_3D( AABB_3D( V ) ) # get the x,y,z extents of the point cloud
        leaf = min( cloudSpan ) * 0.05 # For now use this as the (somewhat subjective) simplification metric
    else:
        leaf = div
    simpleFilter = cloud.make_voxel_grid_filter() # instantiate voxel grid filter based on this point cloud
    simpleFilter.set_leaf_size( leaf , leaf , leaf ) # Run the voxel grid filter using the simplification metric
    simpleCloud = simpleFilter.filter() # Apply the filter and save the simplified cloud
    return simpleCloud.to_array() # Load the simplified cloud into a list of points

## repair_convex_V_F_N
# @brief For 'V' , 'F' , 'N' describing a convex mesh , flip 'F' & 'N' so that they are facing the exterior of the mesh , in place
# Note: This function assumes that the VFN mesh is convex
# @param V - N x 3 (float) matrix in which each row is a unique point in the mesh , output similar to 'uniq_pts_from_STL'
# @param F - M x 3 (int)   matrix in which each row is a list of indices of 'V' that comprise the facet , right-hand [v0,v1,v2] 
# @param N - M x 3 (float) matrix in which each row N[i] is the normal vector of facet F[i]
def repair_convex_V_F_N( V , F , N ):
    # 0. Compute pseudo-centroid
    pCentroid = vec_avg( *V )
    # 1. For each facet
    for i , f_i in enumerate( F ):
        tri = [ V[ index ] for index in f_i ]
        # 2. Get facet center
        triCen = vec_avg( *tri )
        # 3. Construct a vector from centroid to center
        outRay = np.subtract( triCen , pCentroid )
        # 4. Dot the vector with the normal
        dotResult = round_small( np.dot( N[i] , outRay ) )
        # 5. If negative , Flip facet
        if dotResult < 0:
            F[i] = [ f_i[0] , f_i[2] , f_i[1] ]
            N[i] = np.multiply( N[i] , -1.0 )

## hull_faces_from_points
# @brief Return a list of convex hull faces
# @param R3PointsList - List of R3 points for which a convex hull will be formed
# @param CRITRN_ANG --- Maximum angle there can be between two adjacent convex hull face normals for them to be merged to the same support
def hull_faces_from_points( R3PointsList , CRITRN_ANG = radians( 5.0 ) ):
    
    # 1. Compute the convex hull
    simpleHull = ConvexHull( R3PointsList ) # construct a convex hull of 'R3PointsList'
    
    # 2. Pack the simplices into V , F , N
    v0List = [] ; v1List = [] ; v2List = []
    for face in simpleHull.simplices:
#        print "face:" , face
        v0List.append( simpleHull.points[ face[0] ] )
        v1List.append( simpleHull.points[ face[1] ] )
        v2List.append( simpleHull.points[ face[2] ] )   
    V , F , N = tri_list_to_matx_V_F_N( v0List , v1List , v2List )
    
    # 3. Calc neighbors
    orderedNeighborsF = facet_adjacency_list_ordered( F )
    if iter_contains_None( orderedNeighborsF ):
        repair_convex_V_F_N( V , F , N )
        orderedNeighborsF = facet_adjacency_list_ordered( F ) # Neighbors must be recalc'd after repair for clustering to work
        
    # 4. Cluster the simplices
    clusters , clusterNorms = cluster_V_F_N( V , F , N , CRITRN_ANG )
#    print "DEBUG , There were" , len( clusters ) , "clusters"
    
    # 5. Flatten the simplices in the manner above
    return clusters_borders_to_polygons( clusters_to_egdes( V , F , orderedNeighborsF , clusters , clusterNorms ) )

## identify_support_stability
# @brief Given the 'supportList' and 'COM' , Add stability information to each of the supports
# @param supportList - List of candidate support faces given by 'hull_faces_from_points'
# @param COM --------- R3 location of the Center of Mass in the mesh (STL) reference frame 
def identify_support_stability( supportList , COM ):
    # 1. For each of the supports
    for support in supportList:
        # 2. Project the center of mass onto the support
        pnt3D = point_basis_change( COM , support['origin'] , support['xBasis'] , support['yBasis'] , support['zBasis'] )
        pnt2D = pnt3D[:2] # We have already projected the COM to the plane during the basis change , Just remove the Z component!
        pnt3D_partFrame = pnt_proj_to_plane( COM , support['origin'] , support['zBasis'] )
        # 3. Determine if the projected COM lies within the support bounds
        inside = point_in_poly_w( pnt2D , support['polygon2D'] )
        # 4. For each edge of the support
        segDist = []
        for segment in support['segments']:
            # 5. Calc the distance of the COM to the corresponding edge
            segDist.append( -d_point_to_segment_2D_signed( pnt2D , segment ) ) # Assume that the segments are defined CCW
        # 6. Store stability information in each support
        support['stable']       = inside
        support['projectedCOM3D'] = pnt3D_partFrame # In the model frame
        support['projectedCOM2D'] = pnt2D # In the support frame
        support['leastCOMdist'] = min( segDist )
        support['distCOMedge']  = segDist[:]
    return supportList # Not strictly necessary

## process_STL
# @brief Takes an STL path as input , convert to mesh geometry , and produce / gather the information needed to perform grasp planning
# @param pMesh -------------------- 'mesh.Mesh' object , as returned from 'STL_to_mesh_obj'
# @param clusterAngleCriterionRad - Maximum angle there can be between two adjacent STL facet normals for them to be merged to the same cluster
# @param spacingForGraspPoints ---- Grid distance between candidate grasp points on a cluster surface [ if 'None' , will be spacing for 20 on smallest dimension of each cluster ]
# @param simlificationVoxelSize --- Voxel size to simply the mesh points [ if 'None' , spacing will be such there are 20 divisions in shortest dimension ]
# @param supportFaceMergeCritRad -- Maximum angle there can be between two adjacent convex hull face normals for them to be merged to the same support
def process_STL( STLpath                                 ,
                 clusterAngleCriterionRad = radians( 7 ) ,
                 spacingForGraspPoints    = None         ,
                 simlificationVoxelSize   = None         ,
                 supportFaceMergeCritRad  = radians( 7 ) ):
    return process_mesh( STL_to_mesh_obj( STLpath ) ,
                         clusterAngleCriterionRad   ,
                         spacingForGraspPoints      ,
                         simlificationVoxelSize     ,
                         supportFaceMergeCritRad    )

## GraspMesh
# Container for all the geometry information needed for grasp planning
class GraspMesh:
    """ Container for all the geometry information needed for grasp planning """
    
    ## __init__
    # @brief Create a Container for all the geometry information needed for grasp planning 
    # @param V --------- float  , 3 x N , Vertices of the mesh
    # @param F --------- int    , 3 x M , Each row indicates the indices of the vertices that comprise a triangle face , in CCW order
    # @param N --------- float  , 3 x M , Normal vector for each facet F[i]
    # @param adjacency - int    , 3 x M , Each row indicates the indices of the neighboring facets , ordered , See 'facet_adjacency_list_ordered'
    # @param clusters -- [dict] , Contains trial grasp points , facet membership , and geometric information for each cluster
    # @param supports -- [dict] , Contains support stability information , geometric information for each support face
    # @param volume ---- float  , volume of the mesh in [UNIT^3] , Where UNIT is the measurement unit given by the STL scale
    # @param COM ------- float  , 1 x 3 , R3 location of the Center of Mass in the mesh (STL) reference frame 
    def __init__( self , V , F , N , adjacency , clusters , supports , volume , COM ):
        self.V          = V              ##< float  , 3 x N , Vertices of the mesh
        self.F          = F              ##< int    , 3 x M , Each row indicates the indices of the vertices that comprise a triangle face , in CCW order
        self.N          = N              ##< float  , 3 x M , Normal vector for each facet F[i]
        self.adjacencyF = adjacency      ##< int    , 3 x M , Each row indicates the indices of the neighboring facets , ordered , See 'facet_adjacency_list_ordered'
        self.clusters   = clusters       ##< [dict] , Contains trial grasp points , facet membership , and geometric information for each cluster
        self.supports   = supports       ##< [dict] , Contains support stability information , geometric information for each support face
        self.volume     = volume         ##< float  , volume of the mesh in [UNIT^3] , Where UNIT is the measurement unit given by the STL scale
        self.COM        = COM            ##< float  , 1 x 3 , R3 location of the Center of Mass in the mesh (STL) reference frame 
        self.graspPairs = None           ##< tuple  , P x 2 x 3 , R3 pairs of grasp points 
        self.superGraph = TaggedLookup() ##< TaggedLookup , A list of subgraphs, one for each stable pose

"""
~~~ GraspMesh Object Structure ~~~

NOTE: V-F-N structures do not match their OpenGL counterparts, they are 2D matrices

V         ##< float , 3 x N , Vertices of the mesh
[ ...                 ,
  [ x_i , y_i , z_i ] , 
  ...                 ]

F         ##< int   , 3 x M , Each row indicates the indices of the vertices that comprise a triangle face , in CCW order
[ ...                 ,
  [ v0_i , v1_i , v2_i ] , 
  ...                 ]

N         ##< float , 3 x M , Normal vector for each facet F[i]
[ ...                 ,
  [ x_i , y_i , z_i ] , 
  ...                 ]

adjacency ##< int   , 3 x M , Each row indicates the indices of the neighboring facets , ordered , See 'facet_adjacency_list_ordered'
[ ...                 ,
  [ neighbor(v0,v1)_i  ,  neighbor(v1,v2)_i  ,  neighbor(v2,v0)_i ] , 
  ...                 ]

clusters  ##< list of dict , Contains trial grasp points , facet membership , and geometric information for each cluster
{
    AABB          : ( ( xMin , xMax ) , ( yMin , yMax ) ) , 2D Axis Aligned Bounding Box in the ___ cluster frame
    membersF      : Indices of the mesh 'F' that belong to this cluster , list of int
    polygon2D     : Tuple of R2 points that comprise the CCW polygonal border of the cluster ____ , cluster frame
    polygon3D     : Tuple of R3 points that comprise the CCW polygonal border of the cluster ____ , part frame
    segments      : Tuple of R2 point pairs that comprise the CCW polygonal border of the cluster , cluster frame
    trialPoints2D : Tuple of R2 condidate grasp contact points __________________________________ , cluster frame
    trialPoints3D : Tuple of R3 condidate grasp contact points __________________________________ , part frame
    origin        : ( x_o , y_o , z_o ) , R3 origin of the cluster in the _________________________ part frame
    xBasis        : ( x , y , z ) , R3 X basis vector of the cluster in the _______________________ part frame
    yBasis        : ( x , y , z ) , R3 Y basis vector of the cluster in the _______________________ part frame
    zBasis        : ( x , y , z ) , R3 Z basis vector of the cluster in the _______________________ part frame
}

supports  ##< list of dict  , Contains support stability information , geometric information for each support face
{
    distCOMedge    : list of float , minimum distance of the projected COM to each of the support boundaries
    leastCOMdist   : float , support quality , minimum value in 'distCOMedge'
    stable         : bool , True if the projection of the center of mass is within the support boundary , False otherwise
    projectedCOM2D : R2 Center of Mass projected onto the support plane __________________________ , support frame
    projectedCOM3D : R2 Center of Mass projected onto the support plane __________________________ , part frame
    polygon2D      : Tuple of R2 points that comprise the CCW polygonal border of the support ____ , support frame
    polygon3D      : Tuple of R3 points that comprise the CCW polygonal border of the support ____ , part frame
    segments       : Tuple of R2 point pairs that comprise the CCW polygonal border of the support , support frame
    origin         : ( x_o , y_o , z_o ) , R3 origin of the support in the _________________________ part frame
    xBasis         : ( x , y , z ) , R3 X basis vector of the support in the _______________________ part frame
    yBasis         : ( x , y , z ) , R3 Y basis vector of the support in the _______________________ part frame
    zBasis         : ( x , y , z ) , R3 Z basis vector of the support in the _______________________ part frame
} 
# NOTE: 2017-11-20 , Not storing the convex hull itself at this time

volume    ##< float , volume of the mesh in [UNIT^3] , Where UNIT is the measurement unit given by the STL scale

COM       ##< float , 1 x 3 , R3 location of the Center of Mass in the part / mesh (STL) reference frame 
"""

## process_mesh
# @brief Takes an mesh object as input, and produce / gather the information needed to perform grasp planning
# NOTE: Changinf the spacing and angle criteria will change the number of candidate grasps found
# @param pMesh -------------------- 'mesh.Mesh' object , as returned from 'STL_to_mesh_obj'
# @param clusterAngleCriterionRad - Maximum angle there can be between two adjacent STL facet normals for them to be merged to the same cluster
# @param spacingForGraspPoints ---- Grid distance between candidate grasp points on a cluster surface [ if 'None' , will be spacing for 20 on smallest dimension of each cluster ]
# @param simlificationVoxelSize --- Voxel size to simply the mesh points [ if 'None' , spacing will be such there are 20 divisions in shortest dimension ]
# @param supportFaceMergeCritRad -- Maximum angle there can be between two adjacent convex hull face normals for them to be merged to the same support
def process_mesh( pMesh                                   ,
                  clusterAngleCriterionRad = radians( 7 ) ,
                  spacingForGraspPoints    = None         ,
                  simlificationVoxelSize   = None         ,
                  supportFaceMergeCritRad  = radians( 7 ) ):
    # 1. Cluster mesh
    clusters , clstrNormals = cluster_mesh( pMesh , clusterAngleCriterionRad ) # returns clusters , clusterNorms
    V , F , N =  mesh_facets_to_matx_V_F_N( pMesh )
    adjacency = facet_adjacency_list_ordered( F )
    clusterBorders = clusters_to_egdes( V , F , adjacency , clusters , clstrNormals )
    # 2. Identify candidate grasp points
    tile_clusters_with_trial_points( clusterBorders , spacingForGraspPoints ) # Each cluster is now packed with trial point information
    for bDex , border in enumerate( clusterBorders ): # Pack the cluster information back into the borders
        border['membersF'] = clusters[ bDex ]
    # 3. Compute volume and COM
    COM , volume = COM_volume_from_mesh( pMesh ) 
    # 4. Compute the convex hull , support surfaces , and support stability
    hullFaces = identify_support_stability( 
        hull_faces_from_points( simplify_points( V , simlificationVoxelSize ) , 
                                supportFaceMergeCritRad ) , 
        COM 
    )
    # 5. Return structure
    return GraspMesh( V , F , N , adjacency , clusterBorders , hullFaces , volume , COM )

# ___ End Regrasp __________________________________________________________________________________________________________________________
