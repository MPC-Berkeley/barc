#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

struct PositionValue
{
    uint8_t address;
    uint32_t timestamp;
    int32_t x, y, z;// coordinates in millimeters
    uint8_t flags;

    bool highResolution;

    bool ready;
    bool processed;
};

struct StationaryBeaconPosition
{
    uint8_t address;
    int32_t x, y, z;// coordinates in millimeters

    bool updatedForMsg;
    bool highResolution;
};
#define MAX_STATIONARY_BEACONS 30
struct StationaryBeaconsPositions
{
    uint8_t numBeacons;
    struct StationaryBeaconPosition beacons[MAX_STATIONARY_BEACONS];

    bool updated;
};

struct MarvelmindHedge
{
// serial port device name (physical or USB/virtual). It should be provided as
// an argument:
// /dev/ttyACM0 - typical for Linux / Raspberry Pi
// /dev/tty.usbmodem1451 - typical for Mac OS X
    const char * ttyFileName;

// Baud rate. Should be match to baudrate of hedgehog-beacon
// default: 9600
    uint32_t baudRate;

// maximum count of measurements of coordinates stored in buffer
// default: 3
    uint8_t maxBufferedPositions;

// buffer of measurements
    struct PositionValue * positionBuffer;
    
    struct StationaryBeaconsPositions positionsBeacons;

// verbose flag which activate console output
//		default: False
    bool verbose;

//	pause flag. If True, class would not read serial data
    bool pause;

//  If True, thread would exit from main loop and stop
    bool terminationRequired;

//  receiveDataCallback is callback function to recieve data
    void (*receiveDataCallback)(struct PositionValue position);

// private variables
    uint8_t lastValuesCount_;
    uint8_t lastValues_next;
    bool haveNewValues_;
#ifdef WIN32
    HANDLE thread_;
    CRITICAL_SECTION lock_;
#else
    pthread_t thread_;
    pthread_mutex_t lock_;
#endif
};

#define POSITION_DATAGRAM_ID 0x0001
#define BEACONS_POSITIONS_DATAGRAM_ID 0x0002
#define POSITION_DATAGRAM_HIGHRES_ID 0x0011
#define BEACONS_POSITIONS_DATAGRAM_HIGHRES_ID 0x0012

struct MarvelmindHedge * createMarvelmindHedge ();
void destroyMarvelmindHedge (struct MarvelmindHedge * hedge);
void startMarvelmindHedge (struct MarvelmindHedge * hedge);

void printPositionFromMarvelmindHedge (struct MarvelmindHedge * hedge,
                                       bool onlyNew);
bool getPositionFromMarvelmindHedge (struct MarvelmindHedge * hedge,
                                     struct PositionValue * position);
                                     
void printStationaryBeaconsPositionsFromMarvelmindHedge (struct MarvelmindHedge * hedge,
                                                         bool onlyNew);
bool getStationaryBeaconsPositionsFromMarvelmindHedge (struct MarvelmindHedge * hedge,
                                              struct StationaryBeaconsPositions * positions);
void clearStationaryBeaconUpdatedFlag(struct MarvelmindHedge * hedge, uint8_t address);
                                     
void stopMarvelmindHedge (struct MarvelmindHedge * hedge);

#ifdef WIN32
#define DEFAULT_TTY_FILENAME "\\\\.\\COM3"
#else
#define DEFAULT_TTY_FILENAME "/dev/ttyACM0"
#endif // WIN32
