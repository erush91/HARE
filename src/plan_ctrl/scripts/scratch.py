

def even_index_bounds( total , divisions ):
    rtnBounds = []
    stepSize = total * 1.0 / divisions
    runTot = stepSize
    while( runTot < total ):
        rtnBounds.append(  int( round( runTot ) )  );
        runTot += stepSize;
    if len( rtnBounds ) < divisions:
        rtnBounds.append( total )
    return rtnBounds

bounds = even_index_bounds( 720 , 100 )
print bounds
print len( bounds )