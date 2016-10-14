#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#ifdef WIN32
#include <windows.h>
#include <process.h>
#else
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#endif // WIN32
#include "marvelmind_nav/marvelmind_hedge.h"

uint8_t positionDatagramHeader[]=POSITION_DATAGRAM_HEADER;

//////////////////////////////////////////////////////////////////////////////
// Calculate CRC (Modbus) for array of bytes
// buf: input buffer
// len: size of buffer
// returncode: CRC value
//////////////////////////////////////////////////////////////////////////////
uint16_t CalcCrcModbus_(uint8_t * buf, int len)
{
    uint16_t crc = 0xFFFF;
    int pos;
    for (pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos]; // XOR byte into least sig. byte of crc
        int i;
        for (i = 8; i != 0; i--) // Loop over each bit
        {
            if ((crc & 0x0001) != 0) // If the LSB is set
            {
                crc >>= 1; // Shift right and XOR 0xA001
                crc ^= 0xA001;
            }
            else  // Else LSB is not set
                crc >>= 1; // Just shift right
        }
    }
    return crc;
}

#ifdef WIN32
#define SERIAL_PORT_HANDLE HANDLE
#define PORT_NOT_OPENED INVALID_HANDLE_VALUE
//////////////////////////////////////////////////////////////////////////////
// Open Serial Port (Windows only)
// portFileName:  alias of port (e.g. "COM3"). Add prefix "\\\\.\\" before alias
//             to open higher ports (e.g. COM12)
// baudrate:   baudRate rate (e.g. 19200)
// verbose:    print errors
// returncode: valid handle if port is successfully opened or
//             INVALID_HANDLE_VALUE on error
//////////////////////////////////////////////////////////////////////////////
HANDLE OpenSerialPort_ (const char * portFileName, uint32_t baudrate,
                        bool verbose)
{
    HANDLE ttyHandle = CreateFile( TEXT(portFileName), GENERIC_READ, 0,
                                   NULL, OPEN_EXISTING, 0, NULL);
    if (ttyHandle==INVALID_HANDLE_VALUE)
    {
        if (verbose)
         {
			puts(portFileName);
            puts ("Error: unable to open serial connection "
                  "(possibly serial port is not available)");
		 }
        return INVALID_HANDLE_VALUE;
    }
    COMMTIMEOUTS timeouts= {3000,3000,3000,3000,3000};
    bool returnCode=SetCommTimeouts (ttyHandle, &timeouts);
    if (!returnCode)
    {
        if (verbose) puts ("Error: unable to set serial port timeouts");
        CloseHandle (ttyHandle);
        return INVALID_HANDLE_VALUE;
    }
    DCB dcb= {0};
    returnCode=GetCommState (ttyHandle, &dcb);
    if (!returnCode)
    {
        if (verbose) puts ("Error: unable to get serial port parameters");
        CloseHandle (ttyHandle);
        return INVALID_HANDLE_VALUE;
    }
    dcb.BaudRate = baudrate;
    dcb.fAbortOnError=true;
    returnCode=SetCommState (ttyHandle, &dcb);
    if (!returnCode)
    {
        if (verbose) puts ("Error: unable to set serial port parameters");
        CloseHandle (ttyHandle);
        return INVALID_HANDLE_VALUE;
    }
    return ttyHandle;
}
#else
//////////////////////////////////////////////////////////////////////////////
// Converts baudrate value to baudRate code (Linux only)
// baudrate:   value of baudRate rate (e.g. 19200)
// verbose:    show errors
// returncode: code of baudRate rate (e.g. B19200)
//////////////////////////////////////////////////////////////////////////////
uint32_t _GetBaudCode (uint32_t baudrate, bool verbose)
{
    switch (baudrate)
    {
    case 50:
        return B50;
    case 75:
        return B75;
    case 110:
        return B110;
    case 134:
        return B134;
    case 150:
        return B150;
    case 200:
        return B200;
    case 300:
        return B300;
    case 600:
        return B600;
    case 1200:
        return B1200;
    case 1800:
        return B1800;
    case 2400:
        return B2400;
    case 4800:
        return B4800;
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    default:
        if (verbose)
            printf ("Warning: unsupported baudrate %u. Using 9600.\n",
                baudrate);
        return B9600;
    }
}

#define SERIAL_PORT_HANDLE int
#define PORT_NOT_OPENED -1
//////////////////////////////////////////////////////////////////////////////
// Open Serial Port (Linux only)
// portFileName:  alias of port (e.g. "/dev/ttyACM0")
// baudrate:   baudRate rate (e.g. 19200)
// verbose:    show errors
// returncode: valid handle if port is successfully opened or -1 on error
//////////////////////////////////////////////////////////////////////////////
int OpenSerialPort_ (const char * portFileName, uint32_t baudrate, bool verbose)
{
    int ttyHandle = open(portFileName, O_RDWR| O_NONBLOCK | O_NDELAY );
    if (ttyHandle<0)
    {
        if (verbose)
         {
			puts(portFileName);
            puts ("Error: unable to open serial connection "
                  "(possibly serial port is not available)");
		 }
        return -1;
    }
    struct termios ttyCtrl;
    memset (&ttyCtrl, 0, sizeof ttyCtrl);
    if ( tcgetattr ( ttyHandle, &ttyCtrl ) != 0 )
    {
        if (verbose) puts ("Error: unable to get serial port parameters");
        return -1;
    }

    uint32_t baudCode=_GetBaudCode(baudrate, verbose);
    cfsetospeed (&ttyCtrl, baudCode);
    cfsetispeed (&ttyCtrl, baudCode);
    // 8N1, no flow control
    ttyCtrl.c_cflag     &=  ~(PARENB|CSTOPB|CSIZE|CRTSCTS);
    ttyCtrl.c_cflag     |=  CS8;
    // no signaling chars, no echo, no canonical processing
    ttyCtrl.c_lflag     =   0;
    ttyCtrl.c_oflag     =   0; // no remapping, no delays
    ttyCtrl.c_cc[VMIN]      =   0; // read doesn't block
    ttyCtrl.c_cc[VTIME]     =   30; // 3 seconds read timeout
    ttyCtrl.c_cflag     |=  CREAD | CLOCAL; // turn on READ & ignore ctrl lines
    ttyCtrl.c_iflag     &=  ~(IXON | IXOFF | IXANY);// turn off s/w flow ctrl
    ttyCtrl.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    ttyCtrl.c_oflag     &=  ~OPOST; // make raw
    tcflush(ttyHandle, TCIFLUSH ); // Flush port
    if (tcsetattr (ttyHandle, TCSANOW, &ttyCtrl) != 0)
    {
        if (verbose) puts ("Error: unable to set serial port parameters");
        return -1;
    }
    return ttyHandle;
}
#endif



enum
{
    RECV_HDR,
    RECV_DGRAM
};

//////////////////////////////////////////////////////////////////////////////
// Thread function started by MarvelmindHedge_start
//////////////////////////////////////////////////////////////////////////////
void
#ifndef WIN32
*
#endif // WIN32
Marvelmind_Thread_ (void* param)
{
    struct MarvelmindHedge * hedge=(struct MarvelmindHedge*) param;
    struct PositionDatagram_ positionDatagram;
    uint8_t recvState=RECV_HDR; // current state of receive data
    uint8_t nBytesInBlockReceived=0; // bytes received

    uint8_t lastValues_next=0;

    SERIAL_PORT_HANDLE ttyHandle=OpenSerialPort_(hedge->ttyFileName,
                                 hedge->baudRate, hedge->verbose);
    if (ttyHandle==PORT_NOT_OPENED) hedge->terminationRequired=true;
    else if (hedge->verbose) printf ("Opened serial port %s with baudrate %u\n",
                                         hedge->ttyFileName, hedge->baudRate);

    while (hedge->terminationRequired==false)
    {
        uint8_t receivedChar;
        bool readSuccessed=true;
#ifdef WIN32
        uint32_t nBytesRead;
        readSuccessed=ReadFile (ttyHandle, &receivedChar, 1, &nBytesRead, NULL);
#else
        int32_t nBytesRead;
        nBytesRead=read(ttyHandle, &receivedChar, 1);
        if (nBytesRead<0) readSuccessed=false;
#endif
        if (nBytesRead && readSuccessed)
        {
			//printf("%x %d %d ", receivedChar & 0xff, nBytesRead, readSuccessed);
			
            *((char*)&positionDatagram+nBytesInBlockReceived)=receivedChar;
            switch (recvState)
            {
            case RECV_HDR:
                if (receivedChar==positionDatagramHeader[nBytesInBlockReceived])
                {
                    // correct header byte
                    nBytesInBlockReceived++;
                    if (nBytesInBlockReceived>=sizeof(positionDatagramHeader))
                    {
                        recvState=RECV_DGRAM;
                    }
                }
                else
                {
                    // ...or incorrect
                    nBytesInBlockReceived=0;
                }
                break;
            case RECV_DGRAM:
                nBytesInBlockReceived++;
                if (nBytesInBlockReceived>=sizeof(struct PositionDatagram_))
                {
                    // parse dgram
                    uint16_t blockCrc=
                        CalcCrcModbus_((uint8_t*)&positionDatagram,
                                       sizeof(struct PositionDatagram_)-2);
                    if (blockCrc==positionDatagram.crc)
                    {
                        // add to positionBuffer
#ifdef WIN32
                        EnterCriticalSection(&hedge->lock_);
#else
                        pthread_mutex_lock (&hedge->lock_);
#endif
                        hedge->positionBuffer[lastValues_next].timestamp=
                            positionDatagram.timestamp;
                        hedge->positionBuffer[lastValues_next].x=
                            positionDatagram.x;
                        hedge->positionBuffer[lastValues_next].y=
                            positionDatagram.y;
                        hedge->positionBuffer[lastValues_next].z=
                            positionDatagram.z;
                        hedge->positionBuffer[lastValues_next].flags=
                            positionDatagram.flags;
                        lastValues_next++;
                        if (lastValues_next>=hedge->maxBufferedPositions)
                            lastValues_next=0;
                        if (hedge->lastValuesCount_<hedge->maxBufferedPositions)
                            hedge->lastValuesCount_++;
                        hedge->haveNewValues_=true;
#ifdef WIN32
                        LeaveCriticalSection(&hedge->lock_);
#else
                        pthread_mutex_unlock (&hedge->lock_);
#endif
                        // callback
                        if (hedge->receiveDataCallback)
                        {
                            struct PositionValue position=
                            {
                                positionDatagram.timestamp,
                                positionDatagram.x,
                                positionDatagram.y,
                                positionDatagram.z,
                                positionDatagram.flags
                            };
                            hedge->receiveDataCallback (position);
                        }
                    }
                    // and repeat
                    recvState=RECV_HDR;
                    nBytesInBlockReceived=0;
                }
            }
        }
    }
#ifndef WIN32
    return NULL;
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Create an initialize MarvelmindHedge structure
// returncode: pointer to structure on success or NULL on error
//////////////////////////////////////////////////////////////////////////////
struct MarvelmindHedge * createMarvelmindHedge ()
{
    struct MarvelmindHedge * hedge=malloc (sizeof (struct MarvelmindHedge));
    if (hedge)
    {
        hedge->ttyFileName=DEFAULT_TTY_FILENAME;
        hedge->baudRate=9600;
        hedge->maxBufferedPositions=1;
        hedge->positionBuffer=NULL;
        hedge->verbose=false;
        hedge->receiveDataCallback=NULL;
        hedge->lastValuesCount_=0;
        hedge->haveNewValues_=false;
        hedge->terminationRequired= false;
#ifdef WIN32
        InitializeCriticalSection(&hedge->lock_);
#else
        pthread_mutex_init (&hedge->lock_, NULL);
#endif
    }
    else puts ("Not enough memory");
    return hedge;
}

//////////////////////////////////////////////////////////////////////////////
// Initialize and start work thread
//////////////////////////////////////////////////////////////////////////////
void startMarvelmindHedge (struct MarvelmindHedge * hedge)
{
    hedge->positionBuffer=
        malloc(sizeof (struct PositionValue)*hedge->maxBufferedPositions);
    if (hedge->positionBuffer==NULL)
    {
        if (hedge->verbose) puts ("Not enough memory");
        hedge->terminationRequired=true;
        return;
    }
#ifdef WIN32
    _beginthread (Marvelmind_Thread_, 0, hedge);
#else
    pthread_create (&hedge->thread_, NULL, Marvelmind_Thread_, hedge);
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Write average position coordinates
// hedge:      MarvelmindHedge structure
// position:   pointer to PositionValue for write coordinates
// returncode: true if position is valid
//////////////////////////////////////////////////////////////////////////////
bool getPositionFromMarvelmindHedge (struct MarvelmindHedge * hedge,
                                     struct PositionValue * position)
{
    uint8_t i;
    int16_t avg_x=0, avg_y=0, avg_z=0;
    uint32_t max_timestamp=0;
    uint8_t flags= 0;
    bool position_valid;
#ifdef WIN32
    EnterCriticalSection(&hedge->lock_);
#else
    pthread_mutex_lock (&hedge->lock_);
#endif
    if (hedge->lastValuesCount_)
    {
        uint8_t real_values_count=hedge->maxBufferedPositions;
        if (hedge->lastValuesCount_<real_values_count)
            real_values_count=hedge->lastValuesCount_;
        position_valid=true;
        for (i=0; i<real_values_count; i++)
        {
            avg_x+=hedge->positionBuffer[i].x;
            avg_y+=hedge->positionBuffer[i].y;
            avg_z+=hedge->positionBuffer[i].z;
            if (hedge->positionBuffer[i].timestamp>max_timestamp)
                max_timestamp=hedge->positionBuffer[i].timestamp;
                
            flags= hedge->positionBuffer[i].flags;
            if (flags&(1<<0)) 
              {
				position_valid=false;
			  }
        }
        avg_x/=real_values_count;
        avg_y/=real_values_count;
        avg_z/=real_values_count;
        
        if (!position_valid)
          flags|= (1<<0);// coordiantes not available
    }
    else position_valid=false;
#ifdef WIN32
    LeaveCriticalSection(&hedge->lock_);
#else
    pthread_mutex_unlock (&hedge->lock_);
#endif
    position->x=avg_x;
    position->y=avg_y;
    position->z=avg_z;
    position->timestamp=max_timestamp;
    position->flags= flags;
    return position_valid;
}

//////////////////////////////////////////////////////////////////////////////
// Print average position coordinates
// onlyNew: print only new positions
//////////////////////////////////////////////////////////////////////////////
void printPositionFromMarvelmindHedge (struct MarvelmindHedge * hedge,
    bool onlyNew)
{
    if (hedge->haveNewValues_ || (!onlyNew))
    {
        struct PositionValue position;
        getPositionFromMarvelmindHedge (hedge, &position);
        printf ("X: %d, Y: %d, Z: %d at time T: %u\n", position.x,
                position.y, position.z, position.timestamp);
        hedge->haveNewValues_=false;
    }
}

//////////////////////////////////////////////////////////////////////////////
// Stop work thread
//////////////////////////////////////////////////////////////////////////////
void stopMarvelmindHedge (struct MarvelmindHedge * hedge)
{
    hedge->terminationRequired=true;
    if (hedge->verbose) puts ("stopping");
#ifdef WIN32
    WaitForSingleObject (hedge->thread_, INFINITE);
#else
    pthread_join (hedge->thread_, NULL);
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Destroy structures to free memory (You must call stopMarvelmindHedge
// first)
//////////////////////////////////////////////////////////////////////////////
void destroyMarvelmindHedge (struct MarvelmindHedge * hedge)
{
    if (hedge->positionBuffer) free (hedge->positionBuffer);
    free (hedge);
}
