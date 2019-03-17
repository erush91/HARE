/***********  
Cpp_Helpers.cpp
James Watson , 2017 March
Functions of general use for C++ programming

Template Version: 2017-04-13
***********/

#include "Cpp_Helpers.h"

// == Struct Helpers ==

IndexSearchResult default_false_result(){  return IndexSearchResult{ false , 0 };  } // Return a failed search result , index 0

IDSearchResult default_ID_search_rs(){  return IDSearchResult{ false , 0 };  } // Return a failed search result , index 0

// __ End Helpers __

// == Debug Tools ==

void assert_report( bool assertion , string report ) { // Reporting wrapper for 'assert'
    if ( !assertion ) { cerr << endl << report << endl; } // if an assert will trigger, print BEFORE
    assert( assertion ); // crash if necessary
}

void sep_dbg(){ cout << "==================== Debug ====================" << endl; } // Print a separator for debug information

void sep( string title , size_t width , char dingbat ){
    // Print a separating title card for debug 
    cout << string( width , dingbat ) << " " << title << " " << string( width , dingbat ) << endl;
}

void newline(){ cout << endl; } // print a new line

string yesno( bool condition ){  return ( condition ? "YES" : "NO" );  }

void waitkey(){  
    cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    cout << "BREAKPOINT: Press [Enter] to continue...\n";  
    getchar();  
}

void waitkey( bool condition ){
    if( condition ){
        cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
        cout << "BREAKPOINT: Press [Enter] to continue...\n";  
        getchar();
    }
}

void waitkey( bool condition , string message ){
    if( condition ){
        cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
        cout << "BREAKPOINT: " << message << " , Press [Enter] to continue...\n";  
        getchar();
    }
}

// __ End Debug __


// == Logic / Control Tools ==

void toggle( bool& bit ){  bit = !bit;  }

// __ End Logic / Ctrl __


// == Math Tools ==

void rand_init(){  srand( time( NULL ) );  }

float rand_float(){ return (float)  rand() / (float)RAND_MAX; }

double rand_dbbl(){ return (double) rand() / (double)RAND_MAX; }

bool dice_roll( double prob ){  return rand_dbbl() < prob;  }

int    randrange( int end ){    return (int)( rand() % end );    }
size_t randrange( size_t end ){ return (size_t)( rand() % end ); }

// NOTE: 'randrange' functions do not actually check if range bounds are in the proper order

int randrange( int bgn , int end ){ return bgn + (int)( rand() % ( end - bgn ) ); }

usll tri_num( usll n ){ return n * ( n + 1 ) / 2; } // --- Return the nth triangular number
size_t tri_num( size_t n ){ return n * ( n + 1 ) / 2; } // Return the nth triangular number

double round_zero( double num ){ if( abs( num ) < EPSLNDB ){ return 0.0d; }else{ return num; } }

// __ End Math __


// == File Tools ==

bool file_exists( const string& fName ){ // Return true if the file exists , otherwise return false
    struct stat buf; 
    if( stat( fName.c_str() , &buf ) != -1 ){ return true; } else { return false; }
}

std::vector<string> readlines( string path ){ // Return all the lines of text file as a string vector
    std::vector<string> rtnVec;
    if( file_exists( path ) ){
        ifstream fin( path ); // Open the list file for reading
        string line; // String to store each line
        while ( std::getline( fin , line ) ){ // While we are able to fetch a line
            rtnVec.push_back( line ); // Add the file to the list to read
        }
        fin.close();
    } else { cout << "readlines: Could not open the file " << path << endl; }
    return rtnVec;
}

void printlines( const std::vector<string>& lines ){
    // Print all the lines read from a file
    size_t len = lines.size();
    for( size_t i = 0 ; i < len ; i++ ){  cerr << lines[i] << endl;  } // unbuffered dump
}

string timestamp(){
    // Return a timestamp suitable for filenames
    // URL: http://www.cplusplus.com/forum/beginner/32329/#msg174653
    time_t t = time(NULL);
    tm* timePtr = localtime(&t);
    
    return to_string( 1900 + timePtr->tm_year ) + "-" + prepad( to_string( timePtr->tm_mon ) , 2 , '0' ) + "-"
         + prepad( to_string( timePtr->tm_mday ) , 2 , '0' ) + "_" + prepad( to_string( timePtr->tm_hour ) , 2 , '0' )
         + "-" + prepad( to_string( timePtr->tm_min ) , 2 , '0' ) + "-" + prepad( to_string( timePtr->tm_sec ) , 2 , '0' );
}

// = Linux File Tools =

#ifdef IS_LINUX

std::vector<string> list_dir( string dirStr ){ 
    // Return a string vector that contains all the entries in the directory
    // URL , Iterate over files: https://stackoverflow.com/a/612112
    std::vector<string> rtnEntries;
    DIR* dirp = opendir( dirStr.c_str() );
    dirent* dp = nullptr;
    while ( ( dp = readdir(dirp) ) != NULL ){  rtnEntries.push_back( string( dp->d_name ) );  }
    (void)closedir( dirp );
    return rtnEntries;
}

#endif

// _ End Linux _

// __ End File __


// == String Tools ==

void remove_all( string& rawStr , char keyChar ){ // Destructively remove all instances of 'keyChar' from 'rawStr'
    string::size_type i = 0; // http://stackoverflow.com/a/1488798/7186022
    while ( i < rawStr.length() ){ // While the end of the string has not been reached
        i = rawStr.find( keyChar , i ); // Assign the index to the next instance of 'keyChar'
        if ( i == string::npos ){ break; } // If we reached the end of the string , exit
        rawStr.erase( i ); // If an instance of the char has been found , erase it ( 'erase' resizes , 'remove' does not )
    } // If we are outside the loop , we have reached the end of the string
}

// Destructively remove all newlines from 'rawStr'
void strip_newlines( string& rawStr ){ remove_all( rawStr , '\r' ); remove_all( rawStr , '\n' ); }

string strip_after_dot( string fName ){ // Return a copy of 'fName' with the first period and all following characters removed
    size_t index = 0; string rtnStr = "";
    while( fName[index] != '.' && index < fName.size() ){ // While not period and string remains
        rtnStr += fName[index]; // add the char at index and increment index
        index++; 
    } 
    return rtnStr; // Return the transformed copy of the string
}

string prepad(  string original , size_t totLen , char padChar ){
    size_t origLen = original.size();
    if( origLen < totLen ){  return string( totLen - origLen , padChar ) + original;  }
    else{  return original;  }
}

string postpad( string original , size_t totLen , char padChar ){
    size_t origLen = original.size();
    if( origLen < totLen ){  return original + string( totLen - origLen , padChar );  }
    else{  return original;  }
}

// Return true if 'bigStr' contains 'subStr' , Otherwise return false
bool str_has_sub( string bigStr , string subStr ){  return bigStr.find( subStr ) != string::npos;  }

// Return true if the char is a newline , Otherwise return false
bool isnewline( char queryChar ){  return ( queryChar == '\r' ) || ( queryChar == '\n' );  }

std::vector<double> tokenize_to_dbbl_w_separator( string rawStr , char separator ){ 
    // Return a vector of doubles between 'separator'
    size_t len = rawStr.length();
    //~ tokens = [] # 
    std::vector<double> tokens; // list of tokens to return
    //~ currToken = '' # 
    string currToken = ""; // the current token, built a character at a time
    char currChr;
    //~ for char in rawStr: # for each character of the input string
    for( size_t i = 0 ; i < len ; i++ ){
        currChr = rawStr[i];
        //~ if not char.isspace(): # 
        if( !isspace( currChr ) && !isnewline( currChr ) ){ // if the current char is not whitespace, process
            //~ if not char == separator: # if the character is not a separator, then
            if( !( currChr == separator ) ){
                //~ currToken += char # accumulate the char onto the current token
                currToken += currChr;
            }else{ // else the character is a separator, process the previous token
            //~ else: # 
                //~ tokens.append( evalFunc( currToken ) ) # 
                tokens.push_back( atof( currToken.c_str() ) ); // transform token and append to the token list
                //~ currToken = '' # 
                currToken = ""; // reset the current token
            }
        } //~ # else is whitespace, ignore
    }
    //~ if currToken: # If there is data in 'currToken', process it
    if( currToken.length() > 0 )
        //~ tokens.append( evalFunc( currToken ) ) # transform token and append to the token list
        tokens.push_back( atof( currToken.c_str() ) ); // transform token and append to the token list
    //~ return tokens
    return tokens;
}

stdvec<float> tokenize_to_float_w_separator( string rawStr , char separator ){ 
    // Return a vector of doubles between 'separator'
    size_t len = rawStr.length();
    //~ tokens = [] # 
    std::vector<float> tokens; // list of tokens to return
    //~ currToken = '' # 
    string currToken = ""; // the current token, built a character at a time
    char currChr;
    //~ for char in rawStr: # for each character of the input string
    for( size_t i = 0 ; i < len ; i++ ){
        currChr = rawStr[i];
        //~ if not char.isspace(): # 
        if( !isspace( currChr ) && !isnewline( currChr ) ){ // if the current char is not whitespace, process
            //~ if not char == separator: # if the character is not a separator, then
            if( !( currChr == separator ) ){
                //~ currToken += char # accumulate the char onto the current token
                currToken += currChr;
            }else{ // else the character is a separator, process the previous token
            //~ else: # 
                //~ tokens.append( evalFunc( currToken ) ) # 
                tokens.push_back( strtof( currToken.c_str() , NULL ) ); // transform token and append to the token list
                //~ currToken = '' # 
                currToken = ""; // reset the current token
            }
        } //~ # else is whitespace, ignore
    }
    //~ if currToken: # If there is data in 'currToken', process it
    if( currToken.length() > 0 )
        //~ tokens.append( evalFunc( currToken ) ) # transform token and append to the token list
        tokens.push_back( atof( currToken.c_str() ) ); // transform token and append to the token list
    //~ return tokens
    return tokens;
}

// __ End String __


// == Timing ==

// = class StopWatch =

void StopWatch::mark(){
    // Mark the present time for comparison later
    clock_gettime( CLOCK_MONOTONIC , &tMrk );
}

StopWatch::StopWatch(){  mark();  } // New StopWatch with the time of it's instantiation marked

StopWatch::StopWatch( double intervalSec ){
    // New StopWatch with a monitored interval specified
    mark();
    interval = intervalSec;
}

double StopWatch::seconds_elapsed(){
    // Get the number of seconds that have passed since the last mark
    // 1. Get current time
    clock_gettime( CLOCK_MONOTONIC , &tNow );
    // 2. Get elapsed time
    return (double) ( tNow.tv_sec - tMrk.tv_sec ) + (double) ( tNow.tv_nsec - tMrk.tv_nsec ) / BILLION;
}

bool StopWatch::has_interval_elapsed(){
    // If the interval has passed since the last mark, return true and reset mark, Otherwise return false
    bool answer = false;
    // 1. Get elapsed time
    double diff = seconds_elapsed();
    // 2. If the interval has elapsed, set mark
    if( diff >= interval ){
        answer = true;
        mark();
    }
    return answer;
}



// _ End StopWatch _

// __ End Timing __


// == Container Tools ==

std::vector<bool> bool_false_vector( size_t length ){
    std::vector<bool> rtnVec;
    for( size_t i = 0 ; i < length ; i++ ){ rtnVec.push_back( false ); }
    return rtnVec;
}

std::vector<std::vector<bool>> bool_false_vec_vec( size_t length ){
    std::vector<std::vector<bool>> rtnVecVec;
    for( size_t i = 0 ; i < length ; i++ ){ 
        std::vector<bool> temp;
        for( size_t j = 0 ; j < length ; j++ ){  temp.push_back( false );  }
        rtnVecVec.push_back( temp ); 
    }
    return rtnVecVec;
}

size_t random_false_elem_index( std::vector<bool> vec ){
    // Return a random index of an element that has value 'false'
    std::vector<size_t> availableIndices;
    size_t vecLen = vec.size();
    // Build a vector of available 'false' indices so that we are guaranteed to make the right choice
    for( size_t i = 0 ; i < vecLen ; i++ ){ if( vec[i] == false ){ availableIndices.push_back( i ); } }
    return rand_choice( availableIndices );
}

bool all_elem_true( const std::vector<bool>& bulVec ){ 
    // Return true if all elements true , otherwise return false
    size_t len = bulVec.size();
    for( size_t i = 0 ; i < len ; i++ ){  if( !bulVec[i] ){  return false;  }  }
    return true;
}

bool any_elem_true( const std::vector<bool>& bulVec ){
    // Return true if any elements true , otherwise return false
    size_t len = bulVec.size();
    for( size_t i = 0 ; i < len ; i++ ){  if( bulVec[i] ){  return true;  }  }
    return false;
}

size_t count_elem_true( const std::vector<bool>& bulVec ){
    // Return the number of elements that are true
    size_t len       = bulVec.size() , 
           totalTrue = 0             ;
    for( size_t i = 0 ; i < len ; i++ ){  if( bulVec[i] ){  totalTrue++;  }  }
    return totalTrue;
}

llin indexw( llin len , llin i ){
    //  Return the 'i'th index of 'iterable', wrapping to index 0 at all integer multiples of 'len' , Wraps forward and backwards , Python Style
    llin revDex = 0;
    if( i >= 0 ){
        return i % ( len );
    } else {
        revDex = abs( i ) % ( len );
        if( revDex == 0 ){ return 0; }
        return len - revDex;
    }
}

size_t wrap_index_for_len( size_t index , size_t len ){  return index % len;  }

std::ostream& operator<<( std::ostream& os , const std::set<int>& elemSet ) {
    size_t count = 0              ,
           len   = elemSet.size() ;
    os << "(set){ ";
    std::set<int>::iterator it;
    for( it = elemSet.begin() ; it != elemSet.end() ; ++it ){
        os << *it;
        if( count + 1 < len ){ os << ", "; }
        count++;
    }
    os << " }";
    return os; // You must return a reference to the stream!
}

std::vector<size_t> vec_index_zeros( size_t len ){
    std::vector<size_t> rntVec;
    for( size_t i = 0 ; i < len ; i++ ){  rntVec.push_back( 0 );  }
    return rntVec;
}

std::vector<double> vec_dbbl_zeros( size_t len ){
    std::vector<double> rntVec;
    for( size_t i = 0 ; i < len ; i++ ){  rntVec.push_back( 0.0d );  }
    return rntVec;
}

std::vector<std::vector<double>> vec_vec_dbbl_zeros( size_t len ){
    // Return a square vector of zeros
    std::vector<std::vector<double>> rtnVecVec;
    for( size_t i = 0 ; i < len ; i++ ){
        std::vector<double> temp;
        for( size_t j = 0 ; j < len ; j++ ){  temp.push_back( 0 );  }
        rtnVecVec.push_back( temp );
    }
    return rtnVecVec;
}

std::vector<double> err_vec( size_t len ){
    std::vector<double> rtnErr;
    for( size_t i = 0 ; i < len ; i++ ){  rtnErr.push_back( nan("") );  }
    return rtnErr;
}

bool is_err( const std::vector<double>& vec ){
    // Return true if all of the elements are NaN, Otherwise return false
    size_t len = vec.size();
    for( size_t i = 0 ; i < len ; i++ ){  if( !isnan( vec[i] ) ){ return false; }  }
    return true;
}

std::vector<std::vector<size_t>> enumerate_in_base( size_t digits , size_t base ){
    // Return all possible combinations of digits in 'base' that have 'digits'
    std::vector<std::vector<size_t>> rtnVecVec;
    // Basement Case: User asked for no digits
    if( digits == 0 ){  return rtnVecVec;  }
    // Base Case: There is only one digit , Count from 0 to 'base'-1
    if( digits == 1 ){
        for( size_t i = 0 ; i < base ; i++ ){
            std::vector<size_t> temp = { i };
            rtnVecVec.push_back( temp );
        }
        return rtnVecVec;
    }
    // Recursive Case: There is more than 1 digit , Generate leading digits and append trailing digits
    // 1. Fetch trailing digits
    std::vector<std::vector<size_t>> trailingDigits = enumerate_in_base( digits-1 , base );
    size_t /* ------------------- */ trLen /* -- */ = trailingDigits.size();
    std::vector<size_t> /* ------ */ leadingDigit;
    // 2. Generate leading digits and append trailing digits
    for( size_t i = 0 ; i < base ; i++ ){
        leadingDigit = { i };
        for( size_t j = 0 ; j < trLen ; j++ ){  rtnVecVec.push_back(  vec_join( leadingDigit , trailingDigits[j] )  );  }
    }
    return rtnVecVec;
}

// __ End Container __

// === Functors ===

// == class Incrementer ==

Incrementer::Incrementer( llin start ){ count = start; } // Create an incrementer with a starting number

llin Incrementer::operator()(){ llin temp = count;  count++;  return temp;  } // Increment the counter and return it

// __ End Incrementer __

// ___ End Functors ___


// === Memory Utils ===

string pointer_info_str( void* generalPointer ){
    return string( "Pointer Null?: " ) 
         + string( generalPointer == nullptr ? "YES" : "NO" ) 
         + string( " , Data: " )
         + string( (char*)generalPointer );
}

// ___ End Memory ___
