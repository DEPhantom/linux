// Compilation : g++ HR-Server05-ReceiveBinary.cpp -o HR-Server05-ReceiveBinary
// 
// Usage : either >>./HR-Server05-ReceiveBinary<< or >>./HR-Server05-ReceiveBinary srcFileName destFileName<<
//         e.g.,  >>./HR-Server05-ReceiveBinary Dummy receivedImage.jpg<<
//                >>./HR-Server05-ReceiveBinary Dummy CopyOfHelloICE<<       // CopyOfHelloICE is supposedly an executable

# include <unistd.h>
# include <stdio.h>
# include <sys/socket.h>
# include <stdlib.h>
# include <netinet/in.h>
# include <string.h>

# define PORT 23456    // we don't use commonly used ports such as 8080

# define PRINT_IT true  
# define NOT !

# define MAX_TEXT_SIZE 256  // max size of text msg
   
char * gDebugMsg = ( char * ) malloc( sizeof( char ) * MAX_TEXT_SIZE ) ;

void DP( bool printIt, char * msg ) { // Debug Print
// either 
// >>DP( PRINT_IT, msg )<< 
// or 
// >>DP( NOT PRINT_IT, msg )<<

  if ( printIt )
    printf( "%s\n", msg ) ;

} // DP()

int main( int argc, char const *argv[] ) {

  int server_fd ;
  int new_socket ; // client connects with this server via 'new_socket'
  struct sockaddr_in clientAddress;
  int addrlen = sizeof( clientAddress );
  int opt = 1;
     
  int bytesRead = -99999 ;    // size of data saved in 'buffer'
  int bytesToSend = -99999 ;  // size of text messages to send (not used for the moment)

  // to be used for text-communications with clients
  char * outBoundMsg = ( char * ) malloc( sizeof( char ) * MAX_TEXT_SIZE ) ;
  char * inBoundMsg = ( char * ) malloc( sizeof( char ) * MAX_TEXT_SIZE ) ;

  // to be used for checking size-msg
  char * text_Data = ( char * ) malloc( sizeof( char ) * MAX_TEXT_SIZE ) ;
  char * text_has = ( char * ) malloc( sizeof( char ) * MAX_TEXT_SIZE ) ;
  char * text_bytes = ( char * ) malloc( sizeof( char ) * MAX_TEXT_SIZE ) ;
  int numOfBytesInData = 0 ;

  // 0. memory allocation (for saving content of the file-to-receive)

  const int BUFFER_SIZE = 1024*1024 ; // Will 10M suffice??? // this is kind of big ...
  
  char * buffer = ( char * ) malloc( sizeof(char) * BUFFER_SIZE );

  strcpy( gDebugMsg, "'buffer' allocated.\n" ) ;
  DP( NOT PRINT_IT, gDebugMsg ) ;

  // 1. Prepare socket connection with client ; result is 'new_socket'

  if ( ( server_fd = socket( AF_INET, SOCK_STREAM, 0 ) ) == 0 ) { // AF_INET : IPv4 protocol, SOCK_STREAM : TCP
    perror( "Server status : socket() returned a zero fd" );
    exit( EXIT_FAILURE );
  } // if soket() returns a zero fd
     
  // Not sure what 'setsockopt()' does

  if ( setsockopt( server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt) ) ) {
    perror( "Server status : setsockopt() failed" );
    exit( EXIT_FAILURE );
  } // if setsockopt() returns non-zero

  // Attach the socket (server_fd) to the port PORT ; AF_INET : IPv4 protocol

  clientAddress.sin_family = AF_INET;
  clientAddress.sin_addr.s_addr = INADDR_ANY;
  clientAddress.sin_port = htons( PORT );
     
  if ( bind( server_fd, ( struct sockaddr * ) & clientAddress, sizeof( clientAddress ) ) < 0 ) {
    perror("Server status : bind() failed") ;
    exit( EXIT_FAILURE ) ;
  } // if bind() returns less-than-zero

  // start listening

  if ( listen( server_fd, 3 ) < 0 ) { // maximum num of connection request in queue : 3
    perror( "Server status : listen() failed" ) ;
    exit( EXIT_FAILURE ) ;
  } // if listen() returns less-than-zero

  if ( ( new_socket = accept( server_fd, ( struct sockaddr * ) & clientAddress, ( socklen_t * ) & addrlen ) ) < 0 ) {
    perror( "Server status : accept() failed" ) ;
    exit( EXIT_FAILURE ) ;
  } // if accept() returns less-than-zero

  // 2. Either print usage info or prepare the destination file and the source file

  FILE * src = NULL ;
  FILE * dest = NULL ;

  if ( argc != 3 ) {
    printf( "Usage : HR-Server03-ReceiveBinary srcFileName destFileName (However, srcFileName not used)\n" ) ;
    exit( 1 ) ;
  } // if argc is not 3
  
  else { // argc is 3 ; i.e., >>HR-Server03-ReceiveBinary srcFileName destFileName<<
    // src = fopen( argv[1], "rb" ) ;   // the to be sent file
    dest = fopen( argv[2], "wb" ) ;  // the to be received file
  } // else argc IS 3

  // 3. receive and check size-info first (as a double-check measure)

  bytesRead = read( new_socket, inBoundMsg, MAX_TEXT_SIZE );
  // printf("Server report : Message received from client is >>%s<<\n", inBoundMsg );

  sscanf( inBoundMsg, "%s%s%d%s", text_Data, text_has, & numOfBytesInData, text_bytes ) ;

  if (    strcmp( text_Data, "Data" ) != 0 
       || strcmp( text_has, "has" ) != 0 
       || strcmp( text_bytes, "bytes" ) != 0 )
    printf( "Server error : Size message received is %s %s %d %s\n", text_Data, text_has, numOfBytesInData, text_bytes ) ;

  // int recv(int sockfd, void *buf, int len, int flags);

  while ( ( bytesRead = recv( new_socket, buffer, BUFFER_SIZE, 0)) > 0 ) {

    // fwrite( buffer, 1, bytesRead, dest ) ; // this is suggested on StackOverflow

    for ( int i = 0 ; i < bytesRead ; i++ )
      fputc( buffer[i], dest ) ;

  } // while bytesRead non-zero

  bytesRead = -99999 ; // prepare for a possible new round

  // 收尾

  // Do the following ONLY WHEN we are COMPLETELY done with file-receiving !!!!!!!!

  free( buffer ) ; 
  buffer = NULL ;  // hsia company principle

  strcpy( gDebugMsg, "buffer freed\n" ) ;
  DP( NOT PRINT_IT, gDebugMsg ) ;

  // close the files

  // fclose( src ) ; // 'segmentation fault' (on CentOS) if 'src' opened but not used
  fclose( dest ) ;

  return 0 ;

} // main()
