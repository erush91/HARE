

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

print "==================================================================="

class ListRoll( list ):
    """ Rolling List """
    
    def __init__( self , pLength ):
        """ Create a list with a marker for the current index """
        list.__init__( self , [ 0 for i in xrange( pLength ) ] )
        self.length  = pLength
        self.currDex = 0
        
    def add( self , element ):
        """ Insert the element at the current index and increment index """
        self[ self.currDex ] = element
        self.currDex = ( self.currDex + 1 ) % self.length
        
testArr = ListRoll( 5 )

for i in xrange(1,21):
    testArr.add(i)
    print testArr , ',' , len( testArr ) , sum( testArr )
        

