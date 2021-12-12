// Compilation : g++ HR-Client05-SendBinary.cpp -o HR-Client05-SendBinary
// 
// Usage : either >>./HR-Client05-SendBinary<< or >>./HR-Client05-SendBinary srcFileName destFileName<<
//         e.g.,  >>./HR-Client05-SendBinary file1.jpg Dummy<<
//                >>./HR-Client05-SendBinary HelloICE Dummy<<   // HelloICE is supposedly an executable

# include <stdio.h>
# include <stdlib.h>
# include <sys/socket.h>
# include <arpa/inet.h>
# include <unistd.h>
# include <string.h>

# define SERVER_IP  "192.168.60.153"    // instead of "127.0.0.1" (localhost) // <------------ SET IT CORRECTLY
# define PORT 23456    // we don't use commonly used ports such as 8080

# define MAX_TEXT_SIZE 256  // max size of 'outBoundMsg' and 'inBoundMsg'
   
int main(int argc, char const *argv[])
{
  int sock = 0 ;
  struct sockaddr_in serverAddress;
  int bytesRead = -99999 ;    // size of data saved in 'buffer'
  int bytesToSend = -99999 ;  // size of text messages to send

  char * outBoundMsg = ( char * ) malloc( sizeof( char ) * MAX_TEXT_SIZE ) ;
  char * inBoundMsg = ( char * ) malloc( sizeof( char ) * MAX_TEXT_SIZE ) ;

  // 1. Prepare socket connection ; result is 'sock'

  if ( ( sock = socket( AF_INET, SOCK_STREAM, 0 ) ) < 0 ) { // AF_INET : IPv4 protocol, SOCK_STREAM : TCP
    printf("\nClient status : Socket creation error \n");
    return -1;
  } // if socket() returns less-than-zero
  
  serverAddress.sin_family = AF_INET;
  serverAddress.sin_port = htons( PORT ); // note : PORT here
     
  // Convert IPv4 and IPv6 addresses from text to binary form

  if ( inet_pton( AF_INET, SERVER_IP, &serverAddress.sin_addr ) <= 0 ) { // note : SERVER_IP here
    printf("\nClient status : Invalid address/ Address not supported \n");
    return -1;
  } // if inet_pton() returns a non-positive integer
  
  if ( connect( sock, (struct sockaddr *)&serverAddress, sizeof(serverAddress) ) < 0 ) {
    printf("\nClient status : Connection Failed \n");
    return -1;
  } // if connect() returns a less-than-zero integer

  // assert : 'sock' can be used to connect to server now

  // 2. Either print usage info or prepare the destination file and the source file

  FILE * src = NULL ;
  FILE * dest = NULL ;

  if ( argc != 3 ) {
    printf( "Usage : SendToServer nameOfFileToSend nameOfFileReceived (However, nameOfFileReceived not used)\n" ) ;
    exit( 1 ) ;
  } // if argc is not 3
  
  else { // argc is 3 ; i.e., >>SendToServer srcFile destFile<<
    src = fopen( argv[1], "rb" ) ;   // the to be sent file
    dest = fopen( argv[2], "wb" ) ;  // the to be received file
  } // else argc IS 3

  // 3. memory allocation (for saving content of the file-to-send)

  long filesize = 0 ; // size of the to-be-sent-file // should it be >>unsigned long<<???
  char * buffer = NULL ; // for storing content of the to-be-sent-file

  fseek( src, 0, SEEK_END );
  filesize = ftell( src );   
  buffer = ( char * ) malloc( sizeof( char ) * filesize );  // there may be a VERY VERY BIG array here !!!

  // 4. save content of the to-be-sent-file in buffer

  rewind( src );
  bytesRead = fread( buffer, sizeof( char ), filesize, src ) ;

  // 5. send msg-size-info first (as a double-check measure)

  // void *memset(void *ptr, int x, size_t n);
  // ptr : Starting address of memory to be filled
  // x   : Value to be filled
  // n   : Number of bytes to be filled starting from ptr

  memset( outBoundMsg, 0, MAX_TEXT_SIZE ) ;
  sprintf( outBoundMsg, "Data has %d bytes", bytesRead ) ;  // C-string with a NULL at the end

  bytesToSend = strlen( outBoundMsg ) ;
  send( sock , outBoundMsg , bytesToSend , 0 ) ;
  // printf( "Client report : Sent %d of char to server.\n", bytesToSend ) ;

  bytesToSend = -99999 ; // prepare for a possible new round

  // 6. now send the real stuff - buffer content

  send( sock, buffer, bytesRead, 0 ) ;
  // as opposed to
  // send( sock, buffer, filesize, 0 );    // the original author claims that "this is being sent perfectly, no errors"

  bytesRead = -99999 ; // prepare for a possible new round

  // 收尾

  // Do the following ONLY WHEN we are COMPLETELY done with file-sending !!!!!!!!

  free( buffer ) ; 
  buffer = NULL ;  // hsia company principle

  // close the files
  fclose( src ) ;
  // fclose( dest ) ;

  return 0;

} // main()
