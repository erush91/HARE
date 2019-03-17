#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Template Version: 2016-07-08

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

"""
PointCloud.py
James Watson, 2016 August
Working with collections of points in R3
"""

# ~~ Imports ~~
# ~ Standard Libraries ~
import os
import cPickle # Purportedly fast storage
# ~ Special Libraries ~
import numpy as np
import scipy.spatial as sci_spatial
from stl import mesh # https://pypi.python.org/pypi/numpy-stl/ # sudo pip install numpy-stl
# ~ Local Libraries ~
from DebugLog import *
from Vector import vec_avg, vec_mag, point_in_poly_w

# ~~ Constants , Shortcuts , Aliases ~~
EPSILON = 1e-7
infty = 1e309 # URL: http://stackoverflow.com/questions/1628026/python-infinity-any-caveats#comment31860436_1628026

       
def bound_box_3D(R3PointsList):
    """ Given a 'R3PointsList', return a nested tuple of the 3D Bounding Box ( (xMin,xMax) , (yMin,yMax) , (zMin,zMax) ) """
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
            
    return ( (xMin,xMax) , (yMin,yMax) , (zMin,zMax) )
    
def span_3D(bbox3D):
    """ Given a 'bbox3D' ( (xMin,xMax) , (yMin,yMax) , (zMin,zMax) ), Return the extent along each axis """
    return ( abs(bbox3D[0][1] - bbox3D[0][0]) , abs(bbox3D[1][1] - bbox3D[1][0]) , abs(bbox3D[2][1] - bbox3D[2][0]) )  
    
def span_R3_pts( R3PointsList ):
    """ Return the extent of a collection of points in each of the principal directions """
    return span_3D( bound_box_3D( R3PointsList ) )

def bounding_points_from_hull(hull):
    """ Given a scipy.spatial.ConvexHull, return a list of the (supposedly) bounding points """
    # NOTE: Somehow the ConvexHull was not so convex?
    pntList = []
    # ConvexHull.vertices contains the indices of ConvexHull.points that are the bounding poly
    for index in hull.vertices: 
        pntList.append( hull.points[index] )
    return pntList
    
def centroid_discrete_masses(massCenters, totalMass):
    """ Return the COM for a collection of point masses 'massCenters' with a known 'totalMass' """
    centroid = [0 for i in range(len(massCenters[0][1]))] # build a zero vector the same dimensionality as the data coords
    for massPoint in massCenters: # for every mass-point pair in the data
        for i, coord in enumerate(massPoint[1]): # for every coordinate in the point
            centroid[i] += massPoint[0]/totalMass * coord # Add the coordinate scaled by it's mass distribution
    return centroid 

def volume_centroid_of_points(R3PointsList, numSlices = 100):
    """ Return the centroid of a 3D object for which 'R3PointsList' forms a point cloud boundary, assuming uniform desnity """
    # NOTE: This function only operates on a rough convex hull of a 3D point cloud, so cavities and concavities are not considered
    # Establish the extent of the z-axis that the model spans, 'zMin' to 'zMax'
    zMin =  infty
    zMax = -infty
    for center in R3PointsList:
        if center[2] < zMin:
            zMin = center[2]
        if center[2] > zMax:
            zMax = center[2]

    # Init z slices of model
    zSpan = abs(zMax - zMin) # z extent that the model spans
    sliceThknss = zSpan * 1.0 / numSlices # the z thickness of each slice
    zSlices = [[] for i in range(numSlices)]
    sliceCount = [0 for i in range(numSlices)]

    # Flatten all the points found in each z-slize with 'sliceThknss' onto a plane
    loopCount = 0
    for center in R3PointsList:
        # Find the index of the slice that this point belongs to
        index = int( abs(center[2] - zMin) / sliceThknss ) # 'int' rounds down to the next integer
        if index >= numSlices:
            index = numSlices - 1
        #index = abs(center[2] - zMin) / sliceThknss
        #print index,center[:-1],'\t',
        zSlices[index].append( center[:-1] ) # store only the x,y coordinates of the point in the slice
        #zSlices[index].append( index ) # store only the x,y coordinates of the point in the slice
        sliceCount[index] += 1
        loopCount += 1
 
    # form the convex hull of each slice
    sliceBounds = []
    #index = 0
    for index, zSlc in enumerate(zSlices):
        sliceBounds.append([])
        try:
            sliceHull = sci_spatial.ConvexHull( zSlc ) # zSlices[index] )#, qhull_options='Qm' ) # Not 100% certain 'Qm' is needed?
            sliceBounds[-1] = bounding_points_from_hull(sliceHull)
        except Exception as err:
            dbgLog(1, "Encountered" , type(err).__name__ , "on index", index , "with args:", err.args,"with full text:",str(err) )
        #index += 1

    # Compute the cross sectional area, compute the center of the slice
    slcCentroids = []
    slcTotalArea = 0
    for bndDex, bound in enumerate(sliceBounds):
        if len(bound): # If our convex hull was succesfull
            A = 0
            triCentroids = list() # []
            totArea = 0
            for pntDex in range( 2 , len(bound) ):
                # http://www.mathopenref.com/coordtrianglearea.html
                # Assume that we are dealing with convex polygons (hulls), otherwise use shoelace algorithm
                Ax = bound[pntDex][0]
                Bx = bound[pntDex-1][0]
                Cx = bound[0][0]
                Ay = bound[pntDex][1]
                By = bound[pntDex-1][1]
                Cy = bound[0][1]
                A = 0.5 * abs( Ax*(By-Cy) + Bx*(Cy-Ay) + Cx*(Ay-By) )
                totArea += A
                triCentroids.append( ( A , vec_avg( bound[pntDex] , bound[pntDex-1] , bound[0] ) ) )
            slcTotalArea += totArea
            flatCentroid = centroid_discrete_masses(triCentroids,totArea)
            slcCentroids.append( ( totArea , [ flatCentroid[0] , flatCentroid[1] , zMin + sliceThknss/2 + bndDex * sliceThknss ] ) )
        else: # Else produce a slice of zero area, and assume there will be enough good slices to produce a volume centroid
            slcCentroids.append( ( 0 , [ 0 , 0 , 0 ] ) )
        #print "Calculated an area of",totArea,"for this slice"
        
    return centroid_discrete_masses(slcCentroids,slcTotalArea)
    
def moments_of_inertia_principal(R3PointsList, numSlices = 100 , kg_per_m3 = 1000):
    """ Return the centroid of a 3D object for which 'R3PointsList' forms a point cloud boundary, assuming uniform desnity """
    # return mass , I_xx , I_yy , I_zz
    # NOTE: This function only operates on a rough convex hull of a 3D point cloud, so cavities and concavities are not considered
    # NOTE: This function assumes that all distances are in meters
    
    # R3PointsList: List of R3 points in no particular order
    # numSlices:    Number of z slices to partition point cloud into
    # kg_per_m3:    Density in kg/m^3
    
    # Establish the extent of the z-axis that the model spans, 'zMin' to 'zMax'
    zMin =  infty
    zMax = -infty
    for point in R3PointsList:
        if point[2] < zMin:
            zMin = point[2]
        if point[2] > zMax:
            zMax = point[2]

    # Init z slices of model
    zSpan = abs(zMax - zMin) # z extent that the model spans
    sliceThknss = zSpan * 1.0 / numSlices # edge length of a voxel
    voxelMass = (sliceThknss ** 3) * kg_per_m3 # mass = volume * density
    
    zSlices = [[] for i in xrange(numSlices)] # Points assigned to this slice (We don't get them in any particular order)
    # sliceCount = [0 for i in xrange(numSlices)] # number of points in each slice
    zValues = [ i * sliceThknss + sliceThknss/2.0 for i in xrange(numSlices) ] # z coord of each slice

    # Flatten all the points found in each z-slize with 'sliceThknss' onto a plane
    for point in R3PointsList:
        # Find the index of the slice that this point belongs to
        index = int( abs(point[2] - zMin) / sliceThknss ) # 'int' rounds down to the next integer
        if index >= numSlices:
            index = numSlices - 1
        zSlices[index].append( point[:-1] ) # store only the x,y coordinates of the point in the slice

    # form the convex hull of each slice
    sliceBounds = []
    #index = 0
    for index, zSlc in enumerate(zSlices):
        sliceBounds.append([])
        try:
            sliceHull = sci_spatial.ConvexHull( zSlc ) 
            sliceBounds[-1] = bounding_points_from_hull(sliceHull)
        except Exception as err:
            dbgLog(1, "Encountered" , type(err).__name__ , "on index", index , "with args:", err.args,"with full text:",str(err) )
        #index += 1

    # Compute the cross sectional area
    slcTotalArea = 0 # Area of all slices, use to compute mass
    I_xx = I_yy = I_zz = 0 # Init rotational inertias to 0
    for bndDex, bound in enumerate(sliceBounds): # For each slice
        if len(bound): # If our convex hull was succesfull
            totArea = 0
            for pntDex in range( 2 , len(bound) ): # For every point in the bound from the third to the last
                # http://www.mathopenref.com/coordtrianglearea.html
                # Assume that we are dealing with convex polygons (hulls), otherwise use shoelace algorithm
                Ax = bound[pntDex][0]
                Bx = bound[pntDex-1][0]
                Cx = bound[0][0]
                Ay = bound[pntDex][1]
                By = bound[pntDex-1][1]
                Cy = bound[0][1]
                totArea += 0.5 * abs( Ax*(By-Cy) + Bx*(Cy-Ay) + Cx*(Ay-By) )
            slcTotalArea += totArea
            
            # Partition the slice into voxels and compute the contribution of each voxel (point mass) to each rotational inertia
            for xDir in [-1, 1]: # For each side of the x-axis
                xDex = 0
                xInterior = True
                while xInterior:
                    for yDir in [-1, 1]: # For each side of the y-axis
                        yDex = 0
                        yInterior = True
                        while yInterior:
                            voxelCenter = [ xDir * xDex * sliceThknss + xDir * sliceThknss / 2,
                                            yDir * yDex * sliceThknss + yDir * sliceThknss / 2,
                                            zValues[bndDex] ]
                            yDex += 1
                            yInterior = point_in_poly_w( voxelCenter[:2] , bound)
                            if yInterior:
                                I_xx += voxelMass * vec_mag( [ voxelCenter[1] , voxelCenter[2]] ) ** 2  # Distance for x-axis in the Y-Z plane
                                I_yy += voxelMass * vec_mag( [ voxelCenter[0] , voxelCenter[2]] ) ** 2  # Distance for y-axis in the X-Z plane
                                I_zz += voxelMass * vec_mag( [ voxelCenter[0] , voxelCenter[1]] ) ** 2  # Distance for x-axis in the X-Y plane
                    xDex += 1
                    xInterior = point_in_poly_w( [xDir * xDex * sliceThknss + xDir * sliceThknss / 2 , 0] , bound )
                    
    mass = slcTotalArea * sliceThknss * kg_per_m3
    
    return mass , I_xx , I_yy , I_zz
    
# == STL / Mesh Helpers ==

def mesh_from_stl( STLpath ):
    """ Create a mesh object from an STL file, raise IOError if the file does not exist """
    if os.path.isfile( STLpath ):
        return mesh.Mesh.from_file( STLpath )
    else:
        raise IOError( "mesh_from_stl: No such file at " + str(STLpath) )
    
def len_mesh(mesh):
    return len(mesh.normals)

def uniq_pts_from_mesh( mesh , offset=[0.0 , 0.0 , 0.0] ):
    """ Return a set of unique points extracted from numpy-stl mesh """
    # we cannot guarantee that v0 vectors of neighboring facets are not shared. Therefore, we have to iterate through all three
    #arrays of points, collecting the non-repeats into an array
    # pointsOnly = []
    pointsOnly = {}
    # URL, Comparison of uniqueifiers if you need this done faster: http://www.peterbe.com/plog/uniqifiers-benchmark
    for vList in [mesh.v0 , mesh.v1 , mesh.v2]: # for each collection of points
        for point in vList: # for each point in the collection 
            pointsOnly[ tuple(point) ] = 1 # Hashing the coords is MANY , MANY times faster than a linear search!
    return [ np.add( list(item) , offset ) for item in pointsOnly.keys() ] # Cast each point back to array, add offset, and return as a list    

def uniq_pts_from_stl(STLpath, pklPath = False):
    """ Return a set of unique points extracted from an STL file and store them as a list, write to file if 'pklToFile' """
    
    meshObj = mesh_from_stl( STLpath )
    pointsOnly = uniq_pts_from_mesh( meshObj )
    
    if pklPath: # Set to true to save the hand points
        f = open( pklPath , 'wb') # open a file for binary writing to receive pickled data
        cPickle.dump( pointsOnly , f ) 
        f.close()
        print "uniq_pts_from_stl: STL was cPickled to as a list of R3 points to file:" , pklPath
        
    return pointsOnly

# == End STL ==