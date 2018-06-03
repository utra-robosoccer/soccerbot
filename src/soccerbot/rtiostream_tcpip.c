/*
 * Copyright 1994-2017 The MathWorks, Inc.
 *
 * File: rtiostream_tcpip.c     
 *
 * Abstract: This source file implements both client-side and server-side TCP/IP
 *  and UDP/IP communication. Typically, this driver is used to support host-target
 *  communication where the client-side device driver runs on the host and the
 *  server-side driver runs on the target. For this implementation, both
 *  client-side and server-side driver code has been combined into a single
 *  file.
 *
 *  If you are using this code as a starting point to implement a TCP/IP or 
 *  UDP/IP driver for a custom target it is only necessary to include code 
 *  for the server side of the connection.
 */

#ifndef _WIN32
/* Required BSD Unix extensions are not available by default on certain Unix
 * distributions */
#define _BSD_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <limits.h>
#include "rtiostream.h"
#include "tmwtypes.h"

#ifdef _WIN32
  /* WINDOWS */

#if defined(_MSC_VER)
 /* temporarily disable warning triggered
  * by windows.h */
 #pragma warning(push)
 #pragma warning(disable: 4255)
#endif

#include <windows.h>

#if defined(_MSC_VER)
 /* restore warning */
 #pragma warning(pop)
#endif

# ifdef __LCC__
# ifndef __LCC64__
#   include <winsock2.h>
# endif
#   include <errno.h>
# endif

#define RTIOSTREAM_ECONNRESET WSAECONNRESET

#elif defined(VXWORKS)
 /*VxWorks headers*/
# include <selectLib.h>
# include <sockLib.h>
# include <inetLib.h>
# include <ioLib.h>
# include <taskLib.h>
# include <netinet/tcp.h> 

#define RTIOSTREAM_ECONNRESET ECONNRESET

#else
  /* UNIX */
# include <signal.h>
# include <sys/time.h>      /* Linux */
# include <sys/types.h>     /* Linux */
# include <sys/socket.h>
# include <sys/poll.h>
# include <netinet/in.h>    /* Linux */
# include <netinet/tcp.h>   /* Linux */
# include <arpa/inet.h>     /* Linux */
# include <netdb.h>
# include <errno.h>
# include <fcntl.h>  
# include <unistd.h>

#define RTIOSTREAM_ECONNRESET ECONNRESET
#endif

#if defined(_WIN32) || defined(VXWORKS)
#define USE_SELECT  
#endif

#ifdef USE_MEXPRINTF
#include "mex.h"
#define printf mexPrintf
#define SERVER_PORT_PRINTF(FORMAT, ARG1) mexPrintf(FORMAT, ARG1)
#else
/* If stdout is redirected to file, it is essential that the port number is 
 * available immediately in the output file. With LCC, printf does not flush 
 * correctly to the redirected output file - use fprintf & fflush instead. */
#define SERVER_PORT_PRINTF(FORMAT, ARG1) fprintf(stdout, FORMAT, ARG1); \
                                         fflush(stdout)
#endif

/***************** DEFINES ****************************************************/                                         
#define HOSTNAME_MAXLEN (64U)

#define SERVER_PORT_NUM  (17725U)   /* sqrt(pi)*10000 */

/* 
 * EXT_BLOCKING  
 *
 * Depending on the implementation of the main program (e.g., grt_main.c,
 * rt_main.c), the EXT_BLOCKING flag must be set to either 0 or 1.
 *
 * rt_main.c (tornado/vxworks): rt_main.c is a real-time, multi-tasking target.
 * The upload and packet servers are called via background (low priority) tasks.
 * In this case, it is o.k. for the transport function to block as the blocked
 * tasks will simply be pre-empted in order to enable the model to run.  It is
 * desirable to block instead of to poll to free up the cpu for any other
 * potential work. 
 */
#ifdef VXWORKS
# define EXT_BLOCKING (1)  
#else
# define EXT_BLOCKING (0)  
#endif

/* timeout of 0 means to return immediately */
#define BLOCKING_RECV_TIMEOUT_NOWAIT (0)
/* timeout of -1 means to wait indefinitely */
#define BLOCKING_RECV_TIMEOUT_NEVER (-1)
/* rogue value for blocking receive timeout */
#define DEFAULT_BLOCKING_RECV_TIMEOUT (-2)
/* timeout of -3 means to wait for 10 ms to avoid high CPU load */
#define BLOCKING_RECV_TIMEOUT_10MS (-3)
/* wake up from blocking every second */
#define DEFAULT_BLOCKING_RECV_TIMEOUT_SECS_CLIENT (1) 
/* only wake up from blocking when data arrives */
#define DEFAULT_BLOCKING_RECV_TIMEOUT_SECS_SERVER (BLOCKING_RECV_TIMEOUT_NEVER)
/* server wait time for client to close its socket */
#define BLOCKING_RECV_TIMEOUT_SOCK_SHUTDOWN (60)

/* define a set of verbosity levels:
 *
 * 0: no verbose output
 * 1: verbose output with data
 * 2: extra verbose output including when data size is zero*/
typedef enum {VERBOSITY_LEVEL_0=0, VERBOSITY_LEVEL_1, VERBOSITY_LEVEL_2} VerbosityLevel;
/* default verbosity value */
#define DEFAULT_VERBOSITY VERBOSITY_LEVEL_0

/* default protocol value */
#define DEFAULT_PROTOCOL TCP_PROTOCOL
/* allowed -protocol strings */
#define TCP_PROTOCOL_STRING "TCP"
#define UDP_PROTOCOL_STRING "UDP"
#define UDP_PACKET_LOSS_DETECTON_PROTOCOL_STRING "UDP_PACKET_LOSS_DETECTION"

/* default UDP max packet size 
 *
 * The maximum size of UDP packets that are transmitted / received must be
 * consistent on the host and target otherwise receive errors will occur at 
 * the side with the smaller buffer size specified.
 *
 * Use the "-maxudppacketsize SIZE" argument to specify a different packet size.
 * This option is particularly useful when using a custom server implementation
 * that uses a different max packet size to the default. 
 *
 * The maximum UDP payload is 65507 bytes, which can be achieved for localhost
 * based communications on Linux and Windows, but Mac has a lower size of
 * 9216.
 */
#define UDP_MAX_PACKET_SIZE 9216
#define DEFAULT_MAX_UDP_PACKET_SIZE UDP_MAX_PACKET_SIZE
/* increase the UDP socket receive size to decrease the 
 * possibility of buffer overflow */
#define DEFAULT_UDP_SOCKET_RECEIVE_SIZE_REQUEST (512 * 1024) 
#define DEFAULT_UDP_SOCKET_SEND_SIZE_REQUEST (512 * 1024) 

#define DEFAULT_IS_USING_SEQ_NUM 1

#ifdef WIN32
  /* WINDOWS */
# define close closesocket
# define SOCK_ERR SOCKET_ERROR
#else
  /* UNIX, VXWORKS */
# define INVALID_SOCKET (-1)
# define SOCK_ERR (-1)

  typedef int SOCKET;
#endif

/*
 * send prototype differs on different platforms. The following typedefs
 * suppress compiler warnings.
 */
#if defined(WIN32)
typedef const char * send_buffer_t;
#elif defined(VXWORKS)
typedef char * send_buffer_t;
#else
typedef const void * send_buffer_t;
#endif

/* MIN utility */
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

/***************** TYPEDEFS **************************************************/

#if (defined(_WIN32)) || (defined(VXWORKS))
   /* socklen_t may not be available */
   typedef int rtiostream_socklen_t;
#else
   typedef socklen_t rtiostream_socklen_t;
#endif

/* Server specific data structure */
typedef struct ServerData_tag {
    int       port;           /* port number associated with the server socket */
    SOCKET    listenSock;     /* listening socket to accept incoming connections */
    char      *serverInfoFile; /* the filename that is used to write the server 
                                 port number when dynamic port allocation is used */                      
} ServerData;

/* UDP send / receive buffer data structure */
typedef struct UDPPacketBuffer_tag {
    char * buffer; /* pointer to the buffer */
    char * dataPtr; /* pointer to the current position in the buffer */
    int dataAvail; /* amount of data in the buffer */
} UDPPacketBuffer;

/* Type for the optional UDP sequence number */
typedef uint32_T udpSeqNum_T;
/* byte size of the UDP sequence number */
#define UDP_SEQ_NUM_SIZE ((int) sizeof(udpSeqNum_T))

/* UDP specific data structure */
typedef struct UDPData_tag {
   int isUsingSeqNum; /* is this connection using sequence numbers */
   int maxPacketSize; /* max packet size (buffer size) */
   UDPPacketBuffer * recvBuffer; /* buffer for an incoming datagram */
   UDPPacketBuffer * sendBuffer; /* buffer for an outgoing datagram */
   udpSeqNum_T sendSeqNum; /* sequence number to add to outgoing datagrams */
   udpSeqNum_T expectedRecvSeqNum; /* expected sequence number in incoming 
                                   datagrams */
   int resetExpectedRecvSeqNum; /* flags whether to reset expectedRecvSeqNum
                                   to the sequence number of the next incoming
                                   datagram */
} UDPData;

/* enum of supported communications protocols */
typedef enum {TCP_PROTOCOL, UDP_PROTOCOL} CommsProtocol;

/* Data encapsulating a single client / server connection  */
typedef struct ConnectionData_tag {
   int isInUse; /* is this ConnectionData instance currently in use? */
   int isServer; /* is this ConnectionData instance a Server (or client)? */
   int blockingRecvTimeout; /*    Timeout value in seconds. rtIOStreamRecv 
                                  blocks until at least some of the requested 
                                  data is available or the timeout expires.   
                                  If a timeout occurs the receiveSize will be 0. 

                                  A value of BLOCKING_RECV_TIMEOUT_NOWAIT (0)
                                  means to block for 0 seconds (polling mode). 
                                  rtIOStreamRecv processes 
                                  any pending data or, if no data is available, 
                                  returns immediately with a receiveSize of 0.

                                  A value of BLOCKING_RECV_TIMEOUT_NEVER (-1)
                                  means to block indefinitely (full blocking 
                                  mode). rtIOStreamRecv blocks
                                  until at least some of the requested data is 
                                  available.   receiveSize should always be 
                                  greater than 0. 

                                  A value of BLOCKING_RECV_TIMEOUT_10MS (-3)
                                  means to block for up to 10 ms to avoid high CPU 
                                  load.
                             */
   int verbosity; /* enum indicating the level of verbosity to be displayed on the output */
   CommsProtocol protocol; /* TCP or UDP protocol */
   SOCKET sock; /* socket to send/receive packets */
   ServerData * serverData; /* Server specific data - NULL for clients */
   UDPData * udpData; /* UDP specific data - NULL for TCP */
   int udpSendBufSize;
   int udpRecvBufSize;
} ConnectionData;

/**************** LOCAL DATA *************************************************/

/* All local data resides in the per client / 
 * server instance ConnectionData structures to make sure each connection is
 * completely independent.
 *
 * Each ConnectionData does not use much memory; any optionally required 
 * send / recv buffers are dynamically allocated and freed when the ConnectionData 
 * actually becomes in use.
 *
 * The static array will be deallocated when the shared library is unloaded. 
 *
 * Using an array rather than a linked list allows us to have fast direct lookup
 * of ConnectionData from connectionID during calls to rtIOStreamSend/Recv */
#define MAX_NUM_CONNECTIONS (50)
static ConnectionData connectionDataArray[MAX_NUM_CONNECTIONS];

/************** LOCAL FUNCTION PROTOTYPES ************************************/

static int initConnectionData(int connectionID, 
                          int isServer, 
                          CommsProtocol protocol, 
                          SOCKET sock, 
                          int blockingRecvTimeout,
                          int maxPacketSize, 
                          int serverPort, 
                          char * serverInfoFile, 
                          int verbosity, 
                          int isUsingSeqNum,
                          int udpSendBufSize,
                          int udpRecvBufSize); 

static int getConnectionID(void);

static ConnectionData * getConnectionData(int connectionID);

static void freeConnectionData(ConnectionData * connection);

static UDPPacketBuffer * createUDPPacketBuffer(int maxPacketSize);

static void freeUDPPacketBuffer(UDPPacketBuffer ** udpPacketBuffer);

static void resetUDPPacketBuffer(UDPPacketBuffer * udpPacketBuffer);

static int processUDPRecvSeqNum(ConnectionData * connection);

static int initialUDPServerRecvfrom(ConnectionData * connection,
                                    struct sockaddr * clientSA,
                                    rtiostream_socklen_t * clientSALen);

static int waitForClientClose(ConnectionData * connection);

static int socketDataSet(
    ConnectionData * connection, 
    const void *src,
    const size_t size,
    size_t *sizeSent);

static int socketDataGet(
    ConnectionData * connection, 
    char          *dst,
    const size_t   size,
    size_t        *sizeRecvd);

static int socketDataPending(
    const SOCKET sock,
    ConnectionData * connection,
    int    *outPending,
    int timeoutSecs);

static int serverStreamRecv( 
    ConnectionData * connection, 
    void * dst,
    size_t size,
    size_t * sizeRecvd);

static SOCKET serverOpenSocket(int port, char * serverInfoFile, CommsProtocol protocol,int udpSendBufSize, int udpRecvBufSize);

#if (!defined(VXWORKS))
static SOCKET clientOpenSocket(char * hostName, unsigned int portNum, CommsProtocol protocol, int udpSendBufSize, int udpRecvBufSize);
#endif

static void serverAcceptSocket(ConnectionData * connection);

static int processArgs(
    const int       argc,
    void         *  argv[],
    char        **  hostName, 
    unsigned int *  portNum,
    unsigned int *  isClient,
    int          *  isBlocking,
    int          *  recvTimeout, 
    char        **  serverInfoFile, 
    CommsProtocol * protocol, 
    int           * maxPacketSize,
    int           * verbosity, 
    int           * isUsingSeqNum,
    int           * udpSendBufSize,
    int           * udpRecvBufSize);

#if (!defined(VXWORKS))
static unsigned long nameLookup(char * hostName);
#endif

/*************** LOCAL FUNCTIONS **********************************************/

/* Function: initConnectionData =================================================
 * Abstract:
 *  Initializes a client / server ConnectionData for the specified protocol.
 *
 *  A return value of RTIOSTREAM_ERROR indicates an error.
 */
static int initConnectionData(int connectionID, 
                          int isServer, 
                          CommsProtocol protocol, 
                          SOCKET sock, 
                          int blockingRecvTimeout,
                          int maxPacketSize, 
                          int serverPort, 
                          char * serverInfoFile, 
                          int verbosity, 
                          int isUsingSeqNum,
                          int udpSendBufSize,
                          int udpRecvBufSize) {
   int retVal = RTIOSTREAM_NO_ERROR;
   ConnectionData * connection = &connectionDataArray[connectionID];
  
   /* initialize the new ConnectionData */
   connection->isInUse = 1;
   connection->isServer = isServer;
   connection->blockingRecvTimeout = blockingRecvTimeout;
   connection->protocol = protocol;
   connection->verbosity = verbosity;
   connection->udpSendBufSize = udpSendBufSize;
   connection->udpRecvBufSize = udpRecvBufSize;
   /* initialize to NULL early so that calls to 
    * freeConnectionData on error will succeed */
   connection->udpData = NULL;
   connection->serverData = NULL;

   if (protocol == UDP_PROTOCOL) {      
      /* initialize the UDP data */
      connection->udpData = (UDPData *) malloc(sizeof(UDPData));
      if (connection->udpData == NULL) {
         printf("initConnectionData:UDPData malloc failed.\n");
         freeConnectionData(connection);
         retVal = RTIOSTREAM_ERROR;
         return retVal; 
      }
      /* initialize to NULL */
      connection->udpData->recvBuffer = NULL;
      connection->udpData->sendBuffer = NULL;
      connection->udpData->isUsingSeqNum = isUsingSeqNum;
      connection->udpData->maxPacketSize = maxPacketSize;
      /* send sequence numbers always start from 0 */
      connection->udpData->sendSeqNum = 0;
      /* initially, seed the expectedRecvSeqNum from the first 
       * received packet */
      connection->udpData->resetExpectedRecvSeqNum = 1;      
      connection->udpData->recvBuffer = createUDPPacketBuffer(maxPacketSize);
      if (connection->udpData->recvBuffer == NULL) {
         printf("initConnectionData:createUDPPacketBuffer failed.\n");
         freeConnectionData(connection);
         retVal = RTIOSTREAM_ERROR;
         return retVal; 
      }
      if (maxPacketSize > UDP_MAX_PACKET_SIZE) {
         /* packet size cannot exceed the maximum 
          * UDP packet size */
         printf("initConnectionData: udpmaxpacketsize must be less than %d\n", UDP_MAX_PACKET_SIZE);
         freeConnectionData(connection);
         retVal = RTIOSTREAM_ERROR;
         return retVal; 
      }
      if (connection->udpData->isUsingSeqNum) {
         /* packet size must be larger than the size
          * of the sequence number */
         if (maxPacketSize <= UDP_SEQ_NUM_SIZE) {
            printf("initConnectionData: udpmaxpacketsize must be larger than %d\n", UDP_SEQ_NUM_SIZE);
            freeConnectionData(connection);
            retVal = RTIOSTREAM_ERROR;
            return retVal; 
         }
         /* send buffer will be required in order to add the sequence
          * number to the outgoing data */
         connection->udpData->sendBuffer = createUDPPacketBuffer(maxPacketSize);
         if (connection->udpData->sendBuffer == NULL) {
            printf("initConnectionData:createUDPPacketBuffer failed.\n");
            freeConnectionData(connection);
            retVal = RTIOSTREAM_ERROR;
            return retVal; 
         }
      }
   }

   if (isServer) {
      /* initialize server data */
      connection->serverData = (ServerData *) malloc(sizeof(ServerData));
      if (connection->serverData == NULL) {
         printf("initConnectionData:ServerData malloc failed.\n");
         freeConnectionData(connection);
         retVal = RTIOSTREAM_ERROR;
         return retVal; 
      }
      connection->serverData->port = serverPort;
      connection->serverData->serverInfoFile = serverInfoFile;
      /* provided sock is the listening sock */
      connection->serverData->listenSock = sock;
      /* later call to serverAcceptSocket will set sock */ 
      connection->sock = INVALID_SOCKET;
   }
   else {
      /* store the provided socket */
      connection->sock = sock;
   }

   if (verbosity) {
      if (connection->protocol == TCP_PROTOCOL) {
         printf("Connection id %d, protocol: TCP/IP\n", connectionID);
      }
      else {
         printf("Connection id %d, protocol: UDP/IP\n", connectionID);
         printf("Connection id %d, maxPacketSize: %d\n", connectionID, 
                                                         connection->udpData->maxPacketSize);
         printf("Connection id %d, isUsingSeqNum: %d\n", connectionID, 
                                                         connection->udpData->isUsingSeqNum);
      }
      {
         /* display the size of the socket receive buffer */
         rtiostream_socklen_t optionLen = sizeof(int);
         int optionValue;
         getsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *) &optionValue, &optionLen);
         printf("Connection id %d, udpSendBufSize: %d\n", connectionID, optionValue);
         getsockopt(sock, SOL_SOCKET, SO_RCVBUF, (char *) &optionValue, &optionLen);
         printf("Connection id %d, udpRecvBufSize: %d\n", connectionID, optionValue);
      }
      printf("Connection id %d, blockingRecvTimeout: %d\n", connectionID, 
                                                            connection->blockingRecvTimeout);
      if (connection->isServer) {
         printf("Connection id %d, type: server\n", connectionID);         
         if (connection->serverData->serverInfoFile != NULL) {
            printf("Connection id %d, server info file: %s\n", connectionID, 
                                                               connection->serverData->serverInfoFile);
         }
      }
      else {
         printf("Connection id %d, type: client\n", connectionID);
      }
      printf("Connection id %d, socket id %d\n", connectionID, (int) sock);
      /* relevant to both clients and servers */
      printf("Connection id %d, server port: %d\n", connectionID, serverPort);
   }
   return retVal;
}

/* Function: getConnectionData =================================================
 * Abstract:
 *  Retrieves a ConnectionData instance given its connectionID 
 *  (as returned by initConnectionData) 
 *
 * NOTE: An invalid connectionID will lead to a NULL pointer being returned 
 */
static ConnectionData * getConnectionData(int connectionID) {
   /* return NULL for invalid or uninitialized connectionIDs */
   ConnectionData * connection = NULL;
   if ((connectionID >= 0) && (connectionID < MAX_NUM_CONNECTIONS)) {
      if (connectionDataArray[connectionID].isInUse) {
         connection = &connectionDataArray[connectionID];
      }
   }
   return connection;
}

/* Function: getConnectionID =================================================
 * Abstract:
 *  Returns a connectionID corresponding to a ConnectionData that is not 
 *  already in use.
 *
 *  Returns RTIOSTREAM_ERROR if all available ConnectionData instances are 
 *  already in use.
 */
static int getConnectionID(void) {
   int connectionID;
   int foundUnusedConnectionData = 0;
   /* linear search for an unused ConnectionData */
   for (connectionID = 0; connectionID < MAX_NUM_CONNECTIONS; connectionID++) {      
      if (!connectionDataArray[connectionID].isInUse) {
         foundUnusedConnectionData = 1;
         break;
      }
   }
   if (!foundUnusedConnectionData) {
      /* all ConnectionData's are in use */
      printf("getConnectionID: All %d available connections are in use.\n", MAX_NUM_CONNECTIONS);
      connectionID = RTIOSTREAM_ERROR;
   }
   return connectionID;
}

/* Function: freeConnectionData =================================================
 * Abstract:
 *  Frees memory associated with the ConnectionData referenced by connectionID.
 *  Marks the ConnectionData instance as no longer in use.
 */
static void freeConnectionData(ConnectionData * connection) {
   /* mark the ConnectionData as not in use */
   connection->isInUse = 0;
   /* free dynamic memory */
   if (connection->protocol == UDP_PROTOCOL) {
      freeUDPPacketBuffer(&connection->udpData->recvBuffer);
      if (connection->udpData->isUsingSeqNum) {
         freeUDPPacketBuffer(&connection->udpData->sendBuffer);
      }
      free(connection->udpData);
      connection->udpData = NULL;
   }
   if (connection->isServer) {
      free(connection->serverData);
      connection->serverData = NULL;
   }
}

/* Function: createUDPPacketBuffer =================================================
 * Abstract:
 *  Allocates storage for and initializes a UDPPacketBuffer
 */
static UDPPacketBuffer * createUDPPacketBuffer(int maxPacketSize) {
   /* initialize the UDP packet buffer */
   UDPPacketBuffer * udpPacketBuffer = (UDPPacketBuffer *) malloc(sizeof(UDPPacketBuffer));      
   if (udpPacketBuffer == NULL) {
      printf("createUDPPacketBuffer:UDPPacketBuffer malloc failed.\n");
      return udpPacketBuffer; 
   }
   /* allocate the buffer */
   udpPacketBuffer->buffer = (char *) calloc(maxPacketSize, sizeof(char));
   if (udpPacketBuffer->buffer == NULL) {
      printf("createUDPPacketBuffer:UDPPacketBuffer buffer malloc failed.\n");
      /* free everything we allocated */
      free(udpPacketBuffer);
      udpPacketBuffer = NULL;
      return udpPacketBuffer; 
   }
   resetUDPPacketBuffer(udpPacketBuffer);
   return udpPacketBuffer;
}

/* Function: freeUDPPacketBuffer =================================================
 * Abstract:
 *  Frees memory associated with the referenced UDPPacketBuffer 
 */
static void freeUDPPacketBuffer(UDPPacketBuffer ** udpPacketBuffer) {
   if (*udpPacketBuffer != NULL) {
      /* free the buffer */
      free((*udpPacketBuffer)->buffer);
      (*udpPacketBuffer)->buffer = NULL;
      /* free the container */
      free(*udpPacketBuffer);
      *udpPacketBuffer = NULL;
   }
}

/* Function: resetUDPPacketBuffer =================================================
 * Abstract:
 *  Resets the referenced UDP Packet Buffer so that it is ready to receive fresh data
 */
static void resetUDPPacketBuffer(UDPPacketBuffer * udpPacketBuffer) {
   udpPacketBuffer->dataPtr = udpPacketBuffer->buffer;
   udpPacketBuffer->dataAvail = 0;
}

/* Function: socketDataPending =================================================
 * Abstract:
 *  Returns true, via the 'pending' arg, if data is pending on the comm line.
 *  Returns false otherwise.
 *
 *  RTIOSTREAM_NO_ERROR is returned on success, RTIOSTREAM_ERROR on failure.
 */
static int socketDataPending(
    const SOCKET sock,
    ConnectionData * connection,
    int    *outPending, 
    int timeoutSecs)
{
    int retVal = RTIOSTREAM_NO_ERROR;
    int             pending;
    
    #ifdef USE_SELECT
    /* Variables used with select() on Win32 & VXWorks */
    struct timeval  tval;
    struct timeval * tvalPtr;
    fd_set          ReadFds;
    #else
    /* Variables used with poll() on Linux */
    struct pollfd PollReadFd[1];
    int tvalPoll;
    #endif
    
    
    if (connection->protocol == UDP_PROTOCOL) {
       /* first check the UDP buffer */
       UDPPacketBuffer * udpPacketBuffer = connection->udpData->recvBuffer;
       if (udpPacketBuffer->dataAvail) {
          *outPending = 1;
          return retVal;
       }
    }

    #ifdef USE_SELECT          /* Beginning of block for using select */
    FD_ZERO(&ReadFds);
    
    #if defined(_WIN32) && defined(_MSC_VER)
      /*temporarily disable warning C4127 caused by using FD_SET*/
      #pragma warning(push)
      #pragma warning(disable: 4127)
    #endif

    /* Using fd_set structure ReadFds on Win32/VXWorks */
    FD_SET(sock, &ReadFds);
    
    #if defined(_WIN32) && defined(_MSC_VER)
      /*enable the warning C4127*/
      #pragma warning(pop)
    #endif

    /* Select() uses a timeval structure for specifying timeout */
    switch (timeoutSecs) {
       case BLOCKING_RECV_TIMEOUT_NEVER:
          /* specify null pointer for blocking */
          tvalPtr = NULL;
          break;
       case BLOCKING_RECV_TIMEOUT_10MS:
          /* set up the 10ms time-val */
          tval.tv_sec  = 0;
          tval.tv_usec = 10000;
          tvalPtr = &tval;
          break;
       default:
          /* set up the time-val */
          tval.tv_sec  = timeoutSecs;
          tval.tv_usec = 0;
          tvalPtr = &tval;
          break;
    }
    #else                       /* Else block for using select */
    /* Poll() uses an int specifying timeout in milliseconds */
    switch (timeoutSecs) {
       case BLOCKING_RECV_TIMEOUT_NEVER:
          /* specify negative value for blocking */
          tvalPoll = -1;
          break;
       case BLOCKING_RECV_TIMEOUT_10MS:
          /* set up the 10ms time-val */
          tvalPoll = 10;
          break;
       default:
          /* set up the time-val */
          tvalPoll = timeoutSecs*1000;
          break;
    }
    #endif                      /* End of block for using select */

    
    #ifdef USE_SELECT
    /* Use select() on Win32 and VxWorks */
    /*
     * Casting the first arg to int removes warnings on windows 64-bit
     * platform.  It is safe to cast a SOCKET to an int here because on
     * Linux SOCKET is typedef'd to int and on windows the first argument
     * to select is ignored (so it doesn't matter what the value is).
     */
    pending = select((int)(sock + 1), &ReadFds, NULL, NULL, tvalPtr);
    #else
    /* Use poll() on Linux to avoid issues with sockets >= FD_SETSIZE */
    PollReadFd[0].fd = sock;
    PollReadFd[0].events = POLLIN;
    pending = poll( PollReadFd, 1, tvalPoll);
    #endif
    
    if (pending == SOCK_ERR) {
        retVal = RTIOSTREAM_ERROR;
    }

    *outPending = (pending==1);
    return(retVal);    

} /* end socketDataPending */ 

/* Function: initialUDPServerRecvfrom =====================================================
 * Abstract:
 *  Reads data from the client via "recvfrom" into the UDP packet buffer.
 *  The client sockaddr is returned via clientSA and clientSALen.
 *
 *  RTIOSTREAM_NO_ERROR is returned on success, RTIOSTREAM_ERROR is returned on
 *  failure.
 */
static int initialUDPServerRecvfrom(ConnectionData * connection,
                                    struct sockaddr * clientSA,
                                    rtiostream_socklen_t * clientSALen) {
   int nRead;
   int retVal;
   UDPPacketBuffer * udpPacketBuffer = connection->udpData->recvBuffer;
   /* reset */ 
   resetUDPPacketBuffer(udpPacketBuffer);
   /* initialize ahead of call to recvfrom */
   *clientSALen = sizeof(*clientSA);
   /* read into UDP buffer from the listenSock
    * and get sockaddr of the client */
   nRead = recvfrom(connection->serverData->listenSock, 
         udpPacketBuffer->dataPtr, 
         connection->udpData->maxPacketSize, 
         0U, 
         clientSA, 
         clientSALen);

   if (nRead == SOCK_ERR) {
      retVal = RTIOSTREAM_ERROR;
   } else {
      /* set dataAvail */
      udpPacketBuffer->dataAvail = nRead;
      /* handle optional sequence number */
      retVal = processUDPRecvSeqNum(connection);
   }
   return retVal;
}

/* Function: processUDPRecvSeqNum =====================================================
 * Abstract:
 *  Processes sequence numbers in received UDP datagrams.
 *
 *  RTIOSTREAM_NO_ERROR is returned on success, RTIOSTREAM_ERROR is returned on
 *  failure.
 */
static int processUDPRecvSeqNum(ConnectionData * connection) {
   int retVal = RTIOSTREAM_NO_ERROR;
   if (connection->udpData->isUsingSeqNum) {
      UDPPacketBuffer * udpPacketBuffer = connection->udpData->recvBuffer;
      /* process sequence number */
      udpSeqNum_T recvSeqNum;
      if (udpPacketBuffer->dataAvail < UDP_SEQ_NUM_SIZE) {
         printf("No receive sequence number found.\n");
         retVal = RTIOSTREAM_ERROR;
         return retVal; 
      }
      /* read sequence number from the buffer 
       *
       * sequence number is always transmitted / received in 
       * host Endian */
      memcpy(&recvSeqNum,
            udpPacketBuffer->dataPtr,
            UDP_SEQ_NUM_SIZE);
      udpPacketBuffer->dataPtr += UDP_SEQ_NUM_SIZE;
      udpPacketBuffer->dataAvail -= UDP_SEQ_NUM_SIZE;
      if (connection->verbosity) {
        printf("Received UDP packet with sequence number: %u\n", recvSeqNum);
      }
      if (connection->udpData->resetExpectedRecvSeqNum) {
         /* reset the expected sequence number */
         connection->udpData->expectedRecvSeqNum = recvSeqNum + 1;
         connection->udpData->resetExpectedRecvSeqNum = 0;
      }
      else {
         /* compare with expected receive seq num */
         if (recvSeqNum != connection->udpData->expectedRecvSeqNum) {
            printf("UDP packet sequence number mismatch. Expected #: %d, Actual #: %d\n", 
                  connection->udpData->expectedRecvSeqNum, recvSeqNum);
            retVal = RTIOSTREAM_ERROR;
         } 
         else {
            /* increment expected receive seq num */
            connection->udpData->expectedRecvSeqNum++;
         }
      }
   }
   return retVal;
}

/* Function: socketDataGet =====================================================
 * Abstract:
 *  Attempts to gets the specified number of bytes from the specified socket.
 *  The number of bytes read is returned via the 'sizeRecvd' parameter.
 *  RTIOSTREAM_NO_ERROR is returned on success, RTIOSTREAM_ERROR is returned on
 *  failure.
 *
 * NOTES:
 *  o it is not an error for 'sizeRecvd' to be returned as 0
 *  o this function blocks if no data is available
 */
static int socketDataGet(ConnectionData * connection,
    char          *dst,
    const size_t   size,
    size_t        *sizeRecvd)
{
    int nRead = 0;
    int retVal = RTIOSTREAM_NO_ERROR; 
    /* Ensure size is not out of range for socket API recv function */
    int sizeLim = (int) MIN(size, INT_MAX);

    if (connection->protocol == TCP_PROTOCOL) {
       nRead = recv(connection->sock, dst, sizeLim, 0U);
       if (nRead == SOCK_ERR) {
          retVal = RTIOSTREAM_ERROR;
       } else {
          retVal = RTIOSTREAM_NO_ERROR;
       }
    }
    else { 
       UDPPacketBuffer * udpPacketBuffer = connection->udpData->recvBuffer;
       /* receive more data in to the buffer if required */
       if (udpPacketBuffer->dataAvail == 0) {         
          /* reset */ 
          resetUDPPacketBuffer(udpPacketBuffer);
          /* read into buffer */
          nRead = recv(connection->sock, 
                       udpPacketBuffer->dataPtr, 
                       connection->udpData->maxPacketSize, 
                       0U);
          
          if (nRead == SOCK_ERR) {
             retVal = RTIOSTREAM_ERROR;
          } else {
             udpPacketBuffer->dataAvail = nRead;
             /* handle optional sequence number */
             retVal = processUDPRecvSeqNum(connection);             
             if (retVal == RTIOSTREAM_ERROR) {
                return retVal;
             }
          }
       }
       /* get data from the buffer */
       /* for the special case where we request a  */
       /* size of 0 bytes, return the whole buffer */
       if (udpPacketBuffer->dataAvail) {
          if (size == 0) {
            nRead = udpPacketBuffer->dataAvail;
          } else {
            nRead = MIN(udpPacketBuffer->dataAvail, sizeLim);
          }
          memcpy(dst, (void *) udpPacketBuffer->dataPtr, nRead);
          udpPacketBuffer->dataAvail -= nRead;
          udpPacketBuffer->dataPtr += nRead;
       }
    }

    if (retVal!=RTIOSTREAM_ERROR) {
        *sizeRecvd = (size_t) nRead;
    }

    return retVal;
} /* end socketDataGet */ 


/* Function: socketDataSet =====================================================
 * Abstract:
 *  Utility function to send data via the specified socket
 */
static int socketDataSet(
    ConnectionData * connection,
    const void *src,
    const size_t size,
    size_t *sizeSent)
{
    int nSent;    
    int retVal = RTIOSTREAM_NO_ERROR;
    const void *sendSrc = src;    

    /* Ensure size is not out of range for socket API send function */
    int sizeLim = (int) MIN(size, INT_MAX);

    if (connection->protocol == UDP_PROTOCOL) {
       /* limit sends according to max packet size */
       int maxPacketSize = connection->udpData->maxPacketSize;
       if (connection->udpData->isUsingSeqNum) {          
          int transferAmount;
          UDPPacketBuffer * udpPacketBuffer = connection->udpData->sendBuffer;
          /* need to apply sequence number and then increment it */
          resetUDPPacketBuffer(udpPacketBuffer); 
          /* set data src */          
          sendSrc = udpPacketBuffer->dataPtr;
          /* add sequence number to the buffer 
           *
           * sequence number is always transmitted / received in 
           * host Endian */
          memcpy(udpPacketBuffer->dataPtr, 
                 &connection->udpData->sendSeqNum,
                 UDP_SEQ_NUM_SIZE);
          udpPacketBuffer->dataPtr += UDP_SEQ_NUM_SIZE;
          udpPacketBuffer->dataAvail += UDP_SEQ_NUM_SIZE;                
          /* copy the data - don't overflow the packet buffer */
          transferAmount = MIN(sizeLim, maxPacketSize - udpPacketBuffer->dataAvail);
          memcpy(udpPacketBuffer->dataPtr,
                 src, 
                 transferAmount);
          udpPacketBuffer->dataAvail += transferAmount;                                   
          sizeLim = udpPacketBuffer->dataAvail;
       }
       else {
          sizeLim = MIN(maxPacketSize, sizeLim);
       }
    }

    nSent = send(connection->sock, (send_buffer_t)sendSrc, sizeLim, 0);
    if (nSent == SOCK_ERR) {
        retVal = RTIOSTREAM_ERROR;
    } else { 
        if ((connection->protocol == UDP_PROTOCOL) &&
            (connection->udpData->isUsingSeqNum) && 
            (nSent > 0)) {
           if (nSent < (int) UDP_SEQ_NUM_SIZE) {
              /* expected the sequence number to have transmitted */
              retVal = RTIOSTREAM_ERROR;
              return retVal;
           }
           else {
              if (connection->verbosity) {
                 printf("Sent UDP packet with sequence number: %u\n", connection->udpData->sendSeqNum);
              }
              /* increment sequence number */
              connection->udpData->sendSeqNum++;
              nSent -= UDP_SEQ_NUM_SIZE;
           }          
        }
        *sizeSent = (size_t)nSent;
    }

    return retVal;
}

/* Function: serverStreamRecv =================================================
 * Abstract:
 *  Send data from the server-side
 */
static int serverStreamRecv( 
    ConnectionData * connection, 
    void * dst,
    size_t size,
    size_t * sizeRecvd)
{
    int retVal = RTIOSTREAM_NO_ERROR;
    *sizeRecvd = 0;

    if (connection->sock == INVALID_SOCKET) {
       /* Attempt to open connection */
       serverAcceptSocket(connection);
    }

    if (connection->sock != INVALID_SOCKET) {
        int pending;
        if (connection->blockingRecvTimeout != BLOCKING_RECV_TIMEOUT_NEVER) {
           /* only call costly "select" if necessary */
           retVal = socketDataPending(connection->sock, 
                                      connection,
                                      &pending, 
                                      connection->blockingRecvTimeout);
        }
        else {
           /* block in "recv" if necessary */
           pending = 1;
        }

        if ( (pending !=0) && (retVal==RTIOSTREAM_NO_ERROR) && (size>0) ) {
           
            retVal = socketDataGet(connection, (char *)dst, size, sizeRecvd);
            
            if (*sizeRecvd == 0) {
                
                if (errno == RTIOSTREAM_ECONNRESET) {
                    /* If we are closing the connection and we received this
                     * error, it means the other side of the connection was
                     * already closed.  Since we are expecting this, we can
                     * ignore this particular error.
                     */
                    retVal = RTIOSTREAM_NO_ERROR;
                } else {
                    /* Connection closed gracefully by client */
                }

                close(connection->sock);
                connection->sock = INVALID_SOCKET;
            }
        }
        
        if ( retVal == RTIOSTREAM_ERROR ) {
            close(connection->sock);
            connection->sock = INVALID_SOCKET;
        }
    }

    return retVal;
}

/* Function: serverOpenSocket =================================================
 * Abstract:
 *  Opens the listening socket to be used for accepting an incoming connection.
 */
static SOCKET serverOpenSocket(int port, char * serverInfoFile, CommsProtocol protocol, int udpSendBufSize, int udpRecvBufSize)
{

    struct sockaddr_in serverAddr;
    int sockStatus;
    rtiostream_socklen_t sFdAddSize     = (rtiostream_socklen_t) sizeof(struct sockaddr_in);
    SOCKET lFd;
    int option;     

    /*
    * Create a TCP or UDP based socket.
    */
    memset((void *) &serverAddr,0,(size_t)sFdAddSize);
    serverAddr.sin_family      = AF_INET;
    serverAddr.sin_port        = htons((unsigned short int) port);
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (protocol == TCP_PROTOCOL) {
        lFd = socket(AF_INET, SOCK_STREAM, 0);
    }
    else {
        lFd = socket(AF_INET, SOCK_DGRAM, 0);
    }
    
    if (lFd == INVALID_SOCKET) {
        printf("socket() call failed.\n");
    } else {
        /*
        * Listening socket should always use the SO_REUSEADDR option
        * ("Unix Network Programming - Networking APIs:Sockets and XTI",
        *   Volume 1, 2nd edition, by W. Richard Stevens).
        */
        option = 1;
        sockStatus = 
            setsockopt(lFd,SOL_SOCKET,SO_REUSEADDR,(char*)&option,sizeof(option));
        if (sockStatus == SOCK_ERR) {
            printf("setsocketopt() call failed.\n");
            close(lFd);
            lFd = INVALID_SOCKET;
        }
        if (protocol == TCP_PROTOCOL)
        {     
            if (lFd != INVALID_SOCKET)
            {
                /* Disable Nagle's Algorithm*/ 
                option = 1;
                sockStatus = 
                    setsockopt(lFd,IPPROTO_TCP,TCP_NODELAY,(char*)&option,sizeof(option));
                if (sockStatus == SOCK_ERR) { 
                    printf("setsocketopt() TCP_NODELAY call failed.\n");
                    close(lFd); 
                    lFd = INVALID_SOCKET; 
                }
            }
        } 
        if (protocol == UDP_PROTOCOL)
        {
            if (lFd != INVALID_SOCKET)
            {

                /* increase the UDP socket send size to increase the 
                * transfer rate */
                sockStatus = 
                    setsockopt(lFd, SOL_SOCKET, SO_SNDBUF,(char*)&udpSendBufSize, sizeof(int));
                if (sockStatus == SOCK_ERR) { 
                    printf("setsocketopt() SO_SNDBUF call failed.\n");
                    close(lFd); 
                    lFd = INVALID_SOCKET; 
                }   
            }

            if (lFd != INVALID_SOCKET)
            {
                /* increase the UDP socket receive size to decrease the 
                * possibility of buffer overflow */
                sockStatus = 
                    setsockopt(lFd, SOL_SOCKET, SO_RCVBUF,(char*)&udpRecvBufSize, sizeof(int));
                if (sockStatus == SOCK_ERR) { 
                    printf("setsocketopt() SO_RCVBUF call failed.\n");
                    close(lFd); 
                    lFd = INVALID_SOCKET; 
                } 
            }
        }
    }

    if (lFd != INVALID_SOCKET) {
        sockStatus = bind(lFd, (struct sockaddr *) &serverAddr, sFdAddSize);
        if (sockStatus == SOCK_ERR) {
            printf("bind() call failed: %s\n", strerror(errno));
            close(lFd);
            lFd = INVALID_SOCKET;
        }
    }

    if (lFd != INVALID_SOCKET) {
        if (port == 0) {
            /* port 0 specifies dynamic free port allocation
            * reuse serverAddr to store the actual address / port */
            sockStatus = getsockname(lFd, (struct sockaddr *) &serverAddr, &sFdAddSize);           
            if (sockStatus == SOCK_ERR) {
                fprintf(stderr,"getsockname() call failed: %s\n", strerror(errno));
                close(lFd);
                lFd = INVALID_SOCKET;               
            } else { 
                if(serverInfoFile != NULL) {
                    FILE* fh;

                    /* Open file in append mode to save info already stored in the file*/
                    fh = fopen(serverInfoFile,"a"); 
#ifdef __LCC64__
                    /* This is needed due to an issue with LCC64, see the following geck:  g919889 */
                    fseek ( fh, 0 , SEEK_END );
#endif
                    if (fh == NULL) {
                        fprintf(stderr,"Unable to open output file to write server port number: %s\n", strerror(errno));
                        lFd = INVALID_SOCKET;
                    }

                    (void)fprintf(fh, "Server Port Number: %u\n", ntohs(serverAddr.sin_port));
                    fclose(fh);
                } else {
                    /* write the server port number to stdout */
                    SERVER_PORT_PRINTF("Server Port Number: %u\n", ntohs(serverAddr.sin_port));
                }
            }                 
        }
    }
    if (protocol == TCP_PROTOCOL) {
        if (lFd != INVALID_SOCKET) {
            sockStatus = listen(lFd, 2);
            if (sockStatus == SOCK_ERR) {
                printf("listen() call failed.\n");
                close(lFd);
                lFd = INVALID_SOCKET;
            }
        }
    }
    return lFd;
}
/* Function: serverAcceptSocket =================================================
 * Abstract:
 *  Called when the target is not currently connected to the host, this 
 *  function attempts to open the connection.  
 *
 *  In the case of sockets, this is a passive operation in that the host
 *  initiates contact, the target simply listens for connection requests.
 *
 * NOTES:
 
 * Blocks according to blockingRecvTimeout. When
 * polling, there may be no open requests pending.  In this case, this
 * function returns without making a connection; this is not an error.
 */
static void serverAcceptSocket(ConnectionData * connection)
{
   struct sockaddr_in clientAddr;
   rtiostream_socklen_t     sFdAddSize     = sizeof(struct sockaddr_in);
   SOCKET  cFd            = INVALID_SOCKET;
   int error             = RTIOSTREAM_NO_ERROR;
   int pending;    

   /* Check that the listening socket is still valid and open a new socket if
    * not */
   if (connection->serverData->listenSock == INVALID_SOCKET) {
      connection->serverData->listenSock = serverOpenSocket(connection->serverData->port,
            connection->serverData->serverInfoFile, 
            connection->protocol,
            connection->udpSendBufSize, connection->udpRecvBufSize);
   }

   /* pass listenSock rather than sock */
   error = socketDataPending(connection->serverData->listenSock, 
         connection,
         &pending, 
         connection->blockingRecvTimeout);

   if ( (pending > 0) && (error==RTIOSTREAM_NO_ERROR) ) {
      if (connection->protocol == TCP_PROTOCOL) {
         /*
          * Wait to accept a connection on the comm socket.
          */
         cFd = accept(connection->serverData->listenSock, 
                     (struct sockaddr *)&clientAddr,
                     &sFdAddSize);

         if (cFd == INVALID_SOCKET) {
            printf("accept() for comm socket failed.\n");
            error = RTIOSTREAM_ERROR;
         } 

         if (error == RTIOSTREAM_ERROR) {
            close(connection->serverData->listenSock);
            connection->serverData->listenSock = INVALID_SOCKET;
         } 
      }
      else {
         /* UDP - data is pending */
         struct sockaddr clientSA;
         rtiostream_socklen_t clientSALen;
         /* new connection, make sure we reset expectedRecvSeqNum, 
          * if sequence numbers are in use */
         connection->udpData->resetExpectedRecvSeqNum = 1;
         /* Do the initial UDP server "recvfrom" to get the 
          * client sockaddr.   Data read will be placed 
          * ready in the UDP packet buffer. */
         error = initialUDPServerRecvfrom(connection, &clientSA, &clientSALen); 
         if (error == RTIOSTREAM_ERROR) {
            close(connection->serverData->listenSock);
            connection->serverData->listenSock = INVALID_SOCKET;
            printf("initialUDPServerRecvfrom() failed.\n");
         }
         else {
            /* connect exclusively to the client so we no longer
             * have to use recvfrom / sendto */
            if (connect(connection->serverData->listenSock, 
                        &clientSA, 
                        clientSALen) == SOCK_ERR) {
               close(connection->serverData->listenSock);
               connection->serverData->listenSock = INVALID_SOCKET;
               printf("Server connect() failed.\n");
            } 
         } 
         /* for UDP, the socket and listening socket are the same */
         cFd = connection->serverData->listenSock;
      }
   }
   /* set sock */
   connection->sock = cFd;
} 


/* Function: nameLookup =======================
 * Lookup target network name.
 */
#if (!defined(VXWORKS))
static unsigned long nameLookup(char * hostName) {

    struct hostent * hp = NULL;
    struct in_addr * iaddr = NULL;
    unsigned long addr = INADDR_NONE;

    /*
     * Default to localhost if hostname not specified.
     */
    if (hostName == NULL) {
        static char localhost[] = "localhost";
        hostName = localhost;
    }
    
    /*
     * See if the address is an IPV4 dot separated address:
     */
    addr = inet_addr(hostName);

    if (addr == INADDR_NONE) {
        /* Since the address is not an IPV4 dot separated address, 
         * do a name lookup to get this: 
         */
        hp = gethostbyname(hostName);
        if (hp == NULL) {
          printf("gethostbyname() call failed.\n");
  	  addr = INADDR_NONE;
        } else {
          iaddr = (struct in_addr *) hp->h_addr;
          addr = iaddr->s_addr;
        }
    }
    return(addr);
}
#endif

/* Function: processArgs ====================================================
 * Abstract:
 *  Process the arguments specified by the user when opening the rtIOStream.
 *      
 *  If any unrecognized options are encountered, ignore them.
 *
 * Returns zero if successful or RTIOSTREAM_ERROR if 
 * an error occurred.
 *
 *  o IMPORTANT!!!
 *    As the arguments are processed, their strings should be NULL'd out in
 *    the argv array. 
 */
static int processArgs(
    const int       argc,
    void         *  argv[],
    char        **  hostName, 
    unsigned int *  portNum,
    unsigned int *  isClient,
    int          *  isBlocking,
    int          *  recvTimeout, 
    char        **  serverInfoFile, 
    CommsProtocol * protocol, 
    int           * maxPacketSize, 
    int           * verbosity, 
    int           * isUsingSeqNum,
    int           * udpSendBufSize,
    int           * udpRecvBufSize)
{
    int        retVal    = RTIOSTREAM_NO_ERROR;
    int        count           = 0;

    while(count < argc) {
        const char *option = (char *)argv[count];
        count++;

        if (option != NULL) {

            if ((strcmp(option, "-hostname") == 0) && (count != argc)) {

                *hostName = (char *)argv[count];
                count++;
                argv[count-2] = NULL;
                argv[count-1] = NULL;

            } else if ((strcmp(option, "-port") == 0) && (count != argc)) {
                char       tmpstr[2];
                int itemsConverted;
                const char *portStr = (char *)argv[count];

                count++;     
                
                itemsConverted = sscanf(portStr,"%d%1s", (int *) portNum, tmpstr);
                if ( (itemsConverted != 1) || 
                     ( ((*portNum != 0) && (*portNum < 255)) || (*portNum > 65535)) 
                    ) {
                    
                    retVal = RTIOSTREAM_ERROR;
                } else {

                    argv[count-2] = NULL;
                    argv[count-1] = NULL;
                }           
                
            } else if ((strcmp(option, "-client") == 0) && (count != argc)) {
                
                *isClient = ( strcmp( (char *)argv[count], "1") == 0 );

                count++;
                argv[count-2] = NULL;
                argv[count-1] = NULL;

            } else if ((strcmp(option, "-blocking") == 0) && (count != argc)) {
                
                *isBlocking = ( strcmp( (char *)argv[count], "1") == 0 );

                count++;
                argv[count-2] = NULL;
                argv[count-1] = NULL;

            } else if ((strcmp(option, "-verbose") == 0) && (count != argc)) {
                int verbosityVal;
                int itemsConverted;
                const char *verbosityStr = (char *)argv[count];
                count++;
                itemsConverted = sscanf(verbosityStr,"%d", &verbosityVal);

                if ((itemsConverted != 1) || (verbosityVal < 0)) {
                    retVal = RTIOSTREAM_ERROR;
                } else {
                    *verbosity = (VerbosityLevel) verbosityVal;
                    argv[count-2] = NULL;
                    argv[count-1] = NULL;
               }

            } else if ((strcmp(option, "-recv_timeout_secs") == 0) && (count != argc)) {
                char       tmpstr[2];
                int itemsConverted;
                const char *timeoutSecsStr = (char *)argv[count];

                count++;     
                
                itemsConverted = sscanf(timeoutSecsStr,"%d%1s", (int *) recvTimeout, tmpstr);
                if ( itemsConverted != 1 ) {
                    retVal = RTIOSTREAM_ERROR;
                } else {

                    argv[count-2] = NULL;
                    argv[count-1] = NULL;
                }           

            } else if((strcmp(option, "-server_info_file") == 0) && (count != argc)) {
                *serverInfoFile= (char *) argv[count];
                
                count++;
                argv[count-2] = NULL;
                argv[count-1] = NULL;
            } else if ((strcmp(option, "-protocol") == 0) && (count != argc)) {
              char * protocolStr = (char *) argv[count];
              count++;
              argv[count-2] = NULL;
              argv[count-1] = NULL;
              /* initialize dependent properties */
              *isUsingSeqNum = 0;
              /* process protocolStr */
              if (strcmp(protocolStr, TCP_PROTOCOL_STRING) == 0) {
                 *protocol = TCP_PROTOCOL;                
              }
              else if (strcmp(protocolStr, UDP_PROTOCOL_STRING) == 0) {
                 *protocol = UDP_PROTOCOL;
              }
              else if (strcmp(protocolStr, UDP_PACKET_LOSS_DETECTON_PROTOCOL_STRING) == 0) {
                 *protocol = UDP_PROTOCOL;
                 /* enable sequence number protocol */
                 *isUsingSeqNum = 1;
              }
              else {
                 /* unrecognized protocol */
                 retVal = RTIOSTREAM_ERROR;
              }            
            } else if ((strcmp(option, "-udpmaxpacketsize") == 0) && (count != argc)) {
               char       tmpstr[2];
               int itemsConverted;
               const char *maxUPDSizeStr = (char *)argv[count];

               count++;     

               itemsConverted = sscanf(maxUPDSizeStr,"%d%1s", maxPacketSize, tmpstr);
               if ( itemsConverted != 1 ) {
                  retVal = RTIOSTREAM_ERROR;
               } else {
                  argv[count-2] = NULL;
                  argv[count-1] = NULL;
               } 
             }else if ((strcmp(option, "-udpsendbuffersize") == 0) && (count != argc)) {
               char       tmpstr[2];
               int itemsConverted;
               const char *udpSendBufSizeStr = (char *)argv[count];

               count++;     

               itemsConverted = sscanf(udpSendBufSizeStr,"%d%1s", udpSendBufSize, tmpstr);
               if ( itemsConverted != 1 ) {
                  retVal = RTIOSTREAM_ERROR;
               } else {
                  argv[count-2] = NULL;
                  argv[count-1] = NULL;
               } 
           }else if ((strcmp(option, "-udpreceivebuffersize") == 0) && (count != argc)) {
               char       tmpstr[2];
               int itemsConverted;
               const char *udpRecvBufSizeStr = (char *)argv[count];

               count++;     

               itemsConverted = sscanf(udpRecvBufSizeStr,"%d%1s", udpRecvBufSize, tmpstr);
               if ( itemsConverted != 1 ) {
                  retVal = RTIOSTREAM_ERROR;
               } else {
                  argv[count-2] = NULL;
                  argv[count-1] = NULL;
               } 
            } else {
                /* issue a warning for the unexpected argument: exception 
                 * is first argument which might be the executable name (
                 * SIL/PIL and extmode use-cases). */
                if ((count!=1) || (strncmp(option, "-", 1)==0)) {
                    printf("The argument '%s' passed to rtiostream_tcpip is "
                            "not valid and will be ignored.\n", option);
                }
            }
        }
    }
    return retVal;
}

/* Function: clientOpenSocket =================================================
 * Abstract:
 *  Open a connection as Client
 */
#if (!defined(VXWORKS))
static SOCKET clientOpenSocket(char * hostName, unsigned int portNum, CommsProtocol protocol, int udpSendBufSize, int udpRecvBufSize) {
    
    struct sockaddr_in sa;
    unsigned long addr = INADDR_NONE;
    int errStatus = RTIOSTREAM_NO_ERROR;
    SOCKET cSock = INVALID_SOCKET;

    addr = nameLookup(hostName);

    if (addr!=INADDR_NONE) {
        
         sa.sin_addr.s_addr = addr;
         sa.sin_family = AF_INET; /*hp->h_addrtype;*/
         sa.sin_port   = htons((unsigned short) portNum);

        /*
         * Create the sockets & make connections.
         */
        if (protocol == TCP_PROTOCOL) {
           cSock = socket(PF_INET,SOCK_STREAM,0);
        } 
        else {
           cSock = socket(PF_INET,SOCK_DGRAM,0);
        }
        
        if (cSock == INVALID_SOCKET) {
            errStatus = RTIOSTREAM_ERROR;
            printf("socket() call failed for comm socket.\n");
        }
    } else {
      errStatus = RTIOSTREAM_ERROR;
    }
 
    if (protocol == UDP_PROTOCOL)
    {
        if (errStatus!=RTIOSTREAM_ERROR)
        {
            /* increase the UDP socket send size to increase the 
            * transfer rate */
           int sockStatus = 
                setsockopt(cSock, SOL_SOCKET, SO_SNDBUF,(char*)&udpSendBufSize, sizeof(int));
           if (sockStatus == SOCK_ERR) { 
              printf("setsocketopt() SO_SNDBUF call failed.\n");
              cSock = INVALID_SOCKET;
              errStatus = RTIOSTREAM_ERROR;
           }    
        }
        if (errStatus!=RTIOSTREAM_ERROR)
        {
            /* increase the UDP socket receive size to decrease the 
            * possibility of buffer overflow */
           int sockStatus = 
                setsockopt(cSock, SOL_SOCKET, SO_RCVBUF,(char*)&udpRecvBufSize, sizeof(int));
           if (sockStatus == SOCK_ERR) { 
              printf("setsocketopt() SO_RCVBUF call failed.\n");
              cSock = INVALID_SOCKET;
              errStatus = RTIOSTREAM_ERROR;
           } 
       }
    }

    if (errStatus!=RTIOSTREAM_ERROR) {
        if (connect(cSock, (struct sockaddr *)&sa, sizeof(sa)) == SOCK_ERR) {
            char tmp[1024];

            sprintf(tmp,
                    "Attempting to establish connection with hostname '%s' "
                    "through port %d.\n", 
                    hostName,
                    ntohs(sa.sin_port));
            cSock = INVALID_SOCKET;
            printf("%s",tmp);
        } 
    }

    return cSock;
}
#endif

/* Function: waitForClientClose =============================================
 * Abstract:
 *
 * Allow the client to close its end of the socket connection before the server
 * closes its own socket.
 *
 * The server will receive any outstanding data on the socket.   When the server
 * receives 0 bytes, it indicates that it has acknowledged that the client 
 * is closing its socket (this is essential for the client to complete
 * closing its socket without error) or that it timed out waiting for the client to 
 * close its socket.
 *
 */
static int waitForClientClose(ConnectionData * connection) {
   int retVal = RTIOSTREAM_NO_ERROR;
#define TMP_BUF_SIZE (40)
   char * tmpBuf[TMP_BUF_SIZE];
   size_t numRecvd;          
   /* cache the original blockingRecvTimeout */
   int savedBlockingRecvTimeout = connection->blockingRecvTimeout;      
   /* wait time for client to close its socket */
   connection->blockingRecvTimeout = BLOCKING_RECV_TIMEOUT_SOCK_SHUTDOWN;
   do {         
      retVal = serverStreamRecv(connection, (void *) tmpBuf, TMP_BUF_SIZE, &numRecvd);
   } while ((numRecvd > 0) && (retVal == RTIOSTREAM_NO_ERROR));
   /* restore blockingRecvTimeout */
   connection->blockingRecvTimeout = savedBlockingRecvTimeout;
#undef TMP_BUF_SIZE
   return retVal;
}

/***************** VISIBLE FUNCTIONS ******************************************/

/* Function: rtIOStreamOpen =================================================
 * Abstract:
 *  Open the connection with the target.
 */
int rtIOStreamOpen(int argc, void * argv[])
{
    char               *xHostName = NULL; /* default */
    char               *serverInfoFile = NULL; /* default */
    unsigned int        xPortNum     = (SERVER_PORT_NUM); /* default */
    unsigned int        isClient = 0; /* default */
    CommsProtocol       protocol = DEFAULT_PROTOCOL;
    int                 isBlockingRecv = EXT_BLOCKING; /* default */
    int                 blockingRecvTimeout = DEFAULT_BLOCKING_RECV_TIMEOUT; /* rogue value */
    int                 maxPacketSize = DEFAULT_MAX_UDP_PACKET_SIZE;
    int                 verbosity = DEFAULT_VERBOSITY;
    int                 isUsingSeqNum = DEFAULT_IS_USING_SEQ_NUM;
    int                 udpSendBufSize = DEFAULT_UDP_SOCKET_SEND_SIZE_REQUEST;
    int                 udpRecvBufSize = DEFAULT_UDP_SOCKET_RECEIVE_SIZE_REQUEST;
    int result = RTIOSTREAM_NO_ERROR;
    int streamID;
    SOCKET sock = INVALID_SOCKET;

    /* determine the streamID for this new connection */
    streamID = getConnectionID();
    if (streamID == RTIOSTREAM_ERROR) {
       result = RTIOSTREAM_ERROR;
       return result;
    }

    result = processArgs(argc, argv, 
                         &xHostName, 
                         &xPortNum, 
                         &isClient, 
                         &isBlockingRecv,
                         &blockingRecvTimeout, 
                         &serverInfoFile, 
                         &protocol, 
                         &maxPacketSize, 
                         &verbosity, 
                         &isUsingSeqNum,
                         &udpSendBufSize,
                         &udpRecvBufSize);

    if (result == RTIOSTREAM_ERROR) {
       return result;
    }

    if (verbosity) {
       printf("rtIOStreamOpen\n");
    }

    if (isBlockingRecv) {
       /* blocking: if blockingRecvTimeout has not been set, initialize to the client or
        * server specific default */
       if ((blockingRecvTimeout == DEFAULT_BLOCKING_RECV_TIMEOUT) ||
           (blockingRecvTimeout < BLOCKING_RECV_TIMEOUT_10MS)) {
          if (isClient) {
             blockingRecvTimeout = DEFAULT_BLOCKING_RECV_TIMEOUT_SECS_CLIENT;
          }
          else {
             blockingRecvTimeout = DEFAULT_BLOCKING_RECV_TIMEOUT_SECS_SERVER;
          }
       }
    }
    else {
       /* not blocking: set the timeout to return immediately */
       blockingRecvTimeout = BLOCKING_RECV_TIMEOUT_NOWAIT;
    }

#ifdef VXWORKS /* UDP is not supported on VxWorks */
    if (protocol == UDP_PROTOCOL) {
       result = RTIOSTREAM_ERROR;
       return result;
    }
#endif

#ifdef _WIN32
    {
        WSADATA data;
        if (WSAStartup((MAKEWORD(1,1)), &data)) {
            result = RTIOSTREAM_ERROR;
            printf("WSAStartup() call failed.\n");
        }
    }
#endif

    if (result != RTIOSTREAM_ERROR) {
        if (isClient == 1) {
#if (!defined(VXWORKS)) /* Client side connection not supported on VxWorks */
           sock = clientOpenSocket(xHostName, xPortNum, protocol,udpSendBufSize,udpRecvBufSize);
           if (sock == INVALID_SOCKET) {
              result = RTIOSTREAM_ERROR;
           }
#endif
        } else {           
           sock = serverOpenSocket(xPortNum, serverInfoFile, protocol,udpSendBufSize,udpRecvBufSize);            
           if (sock == INVALID_SOCKET) {
              result = RTIOSTREAM_ERROR;
           }
        }   
    }

    if (result != RTIOSTREAM_ERROR) {
       int isServer;
       if (isClient == 1) {
          isServer = 0;   
       }
       else {
          isServer = 1;
       }
       result = initConnectionData(streamID, 
             isServer, 
             protocol, 
             sock, 
             blockingRecvTimeout,
             maxPacketSize, 
             xPortNum, 
             serverInfoFile, 
             verbosity, 
             isUsingSeqNum,
             udpSendBufSize,
             udpRecvBufSize);
    }
    
    if (result != RTIOSTREAM_ERROR) {
       result = streamID;
    }
    else {
       if (sock != INVALID_SOCKET) {
          /* cleanup */
          close(sock);
       }
    }
    return result;
}

/* Function: rtIOStreamSend =====================================================
 * Abstract:
 *  Sends the specified number of bytes on the comm line. Returns the number of
 *  bytes sent (if successful) or a negative value if an error occurred. As long
 *  as an error does not occur, this function is guaranteed to set the requested
 *  number of bytes; the function blocks if tcpip's send buffer doesn't have
 *  room for all of the data to be sent
 */
int rtIOStreamSend(
    int streamID,
    const void *src,
    size_t size,
    size_t *sizeSent)
{
    int retVal = RTIOSTREAM_NO_ERROR;
    ConnectionData * connection = getConnectionData(streamID);
    *sizeSent = 0;

    if (connection == NULL) {
       retVal = RTIOSTREAM_ERROR;
       return retVal;
    }

    if (connection->isServer) {
        if (connection->sock == INVALID_SOCKET) {
            serverAcceptSocket(connection);
        }

        if (connection->sock != INVALID_SOCKET) {
#ifndef VXWORKS
           retVal = socketDataSet(connection, src, size, sizeSent);
#else           
           /*
            * VXWORKS send prototype does not have src as const.  This suppresses
            * the compiler warning.
            */

           retVal = socketDataSet(connection, (char *)src, size, sizeSent);
#endif
        }
    } else { /* Client stream */
        retVal = socketDataSet(connection, src, size, sizeSent);
    }

    if (connection->verbosity) {
        if ((*sizeSent > 0) || (connection->verbosity >= VERBOSITY_LEVEL_2)) {
            size_t currElement;
            printf("rtIOStreamSend (connection id %d): size = %lu, sizeSent = %lu: ", 
                   streamID, 
                   (unsigned long) size, 
                   (unsigned long) *sizeSent);
        
            for (currElement = 0; currElement < *sizeSent; currElement++) {
                printf("%02x ", ((const unsigned char *) src)[currElement]);
            }
            printf("\n");
        }       
    }

    return retVal;
}


/* Function: rtIOStreamRecv ================================================
 * Abstract: receive data
 *
 */
int rtIOStreamRecv(
    int      streamID,
    void   * dst, 
    size_t   size,
    size_t * sizeRecvd) 
{
    int retVal = RTIOSTREAM_NO_ERROR;
    ConnectionData * connection = getConnectionData(streamID);

    *sizeRecvd = 0;

    if (connection == NULL) {
       retVal = RTIOSTREAM_ERROR;
       return retVal;
    }

    if (connection->isServer) {
        retVal = serverStreamRecv(connection, dst, size, sizeRecvd); 
    } else { /* Client stream */
        int pending;
        if (connection->blockingRecvTimeout != BLOCKING_RECV_TIMEOUT_NEVER) {
           /* only call costly "select" if necessary */
           retVal = socketDataPending(connection->sock, 
                                      connection,
                                      &pending, 
                                      connection->blockingRecvTimeout);
        }
        else {
           /* block in "recv" if necessary */
           pending = 1;
        }
        if (pending) {
            retVal = socketDataGet(connection, (char *)dst, size, sizeRecvd);
        }
    }

    if (connection->verbosity) {
        if ((*sizeRecvd > 0 ) || (connection->verbosity >= VERBOSITY_LEVEL_2)) {
            size_t currElement;
            printf("rtIOStreamRecv (connection id %d): size = %lu, sizeRecvd = %lu: ", 
                   streamID,
                   (unsigned long) size,
                   (unsigned long) *sizeRecvd);

            for (currElement = 0; currElement < *sizeRecvd; currElement++) {
                printf("%02x ", ((const unsigned char *) dst)[currElement]);
            }
            printf("\n");
        }
    }
    
    return retVal;
}

/* Function: rtIOStreamClose ================================================
 * Abstract: close the connection.
 *
 */
int rtIOStreamClose(int streamID)
{
    int retVal = RTIOSTREAM_NO_ERROR;
    ConnectionData * connection = getConnectionData(streamID);
    if (connection == NULL) {
       retVal = RTIOSTREAM_ERROR;
       return retVal;
    }

    if (connection->verbosity) {
       printf("rtIOStreamClose (connection id %d)\n", streamID);
    }

    if (connection->isServer) {
        /* Only if the client actually made a connection */
        if (connection->sock != INVALID_SOCKET) {
            if (connection->protocol == TCP_PROTOCOL) {
                /* graceful shutdown */
                retVal = waitForClientClose(connection);
            }
            
            /* close the socket */
            close(connection->sock);
            connection->sock = INVALID_SOCKET;
        }
       if (connection->protocol == TCP_PROTOCOL) {
          /* TCP: additionally close the listening socket
           *
           * for UDP, sock and listenSock are the same
           * socket - avoid closing it twice */
          close(connection->serverData->listenSock);
       }
       /* set to INVALID_SOCKET for all protocol types */
       connection->serverData->listenSock = INVALID_SOCKET;       
    } else {
       SOCKET cSock = connection->sock;
       close(cSock);

    }
    freeConnectionData(connection);
    return retVal;
}

