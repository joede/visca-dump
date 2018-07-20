/* -*- Mode: C -*-
 * --------------------------------------------------------------------------
 * Small tool to dump the communication between a VISCA master and a
 * VISCA slave (camera). The master is referred to as "sender" and the slave
 * as "receiver".
 *
 *
 * Compile: gcc -g -Wall -lezV24 -o visca-dump visca-dump.c
 * Run:     ./visca-dump -r /dev/ttyUSB0 -s /dev/ttyUSB1
 * --------------------------------------------------------------------------
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include <ezV24/ezV24.h>



/*+=========================================================================+*/
/*|                      CONSTANT AND MACRO DEFINITIONS                     |*/
/*`========================================================================='*/

#define VERSION                          "0.1"

#define AVG_OUTLIER                      1000           // [ms]
#define SZ_INTERFACE_NAME                10

/* general VISCA definitions */
#define VISCA_TERMINATOR                 0xFF
#define VISCA_MIN_SIZE                   3
#define VISCA_MAX_SIZE                   16

/* API error codes */
#define VISCA_SUCCESS                    0x00
#define VISCA_PENDING                    0x01
#define VISCA_BAD_HEADER                 0xFB
#define VISCA_OVERFLOW                   0xFC
#define VISCA_TIMEDOUT                   0xFD
#define VISCA_HAVE_NO_DATA               0xFE
#define VISCA_FAILURE                    0xFF

/* response types */
#define VISCA_TYPE_RESPONSE_CLEAR        0x40   // ???
#define VISCA_TYPE_RESPONSE_ADDRESS      0x30
#define VISCA_TYPE_RESPONSE_ACK          0x40
#define VISCA_TYPE_RESPONSE_COMPLETED    0x50
#define VISCA_TYPE_RESPONSE_ERROR        0x60

/* Generic definitions */
#define VISCA_ON                         0x02
#define VISCA_OFF                        0x03
#define VISCA_RESET                      0x00
#define VISCA_UP                         0x02
#define VISCA_DOWN                       0x03


/*             .-----------------------------------------------.             */
/* ___________/  local macro declaration                        \___________ */
/*            `-------------------------------------------------'            */

/*+=========================================================================+*/
/*|                          LOCAL TYPEDECLARATIONS                         |*/
/*`========================================================================='*/


/* Index of the command in the array with known sequences. This index counts
 * from 1!
 */
enum COMMAND_SEQUENCE
{
    CMD_IfClear=1,
    CMD_Power,
    CMD_Zoom,
    CMD_Focus,
    CMD_Iris,
    CMD_WBTrigger,
    CMD_FocusTrigger,
    CMD_WB,
    CMD_DZoom,
    CMD_FocusMode,
    CMD_AE,
    CMD_ZoomDirect,
    CMD_Freeze,
    CMD_Title,
    CMD_PowerInq,
    CMD_FocusModeInq,
    CMD_FocusPositionInq,
    CMD_AEModeInq,
    CMD_ZoomPosInq,
    CMD_IrisPosInq,
    CMD_FreezeModeInq,
    CMD_SetAdress,
    CMD_EXT_Turn,
    CMD_EXT_Pairing,

    RPL_Address,
    RPL_Ack,
    RPL_Ack1,
    RPL_Ack2,
    RPL_Word,
    RPL_Byte,
    RPL_Done,
    RPL_Done1,
    RPL_Done2,
    RPL_NotExecutable,
    RPL_NotExecutable_Sock2,
    RPL_IS_ERROR,
    RPL_IS_ERROR_Sock2,
    CMD_MAX_SEQUENCES
};

struct tagVISCA_SEQUENCE
{
    uint8_t seq[VISCA_MAX_SIZE];        // the sequence of bytes
    uint8_t length;                     // length of the sequence incl. parameter
    uint8_t comparable;                 // comparable length (without parameters)
};
typedef struct tagVISCA_SEQUENCE T_VISCA_Sequence;


/* INTERFACE STRUCTURE */
typedef struct tagVISCA_INTERFACE
{
    // RS232 port
    v24_port_t  *uart;
    char name[SZ_INTERFACE_NAME+1];

    // VISCA data:
    int address;
    int broadcast;

    // RS232 input buffer, ...
    uint8_t buffer[VISCA_MAX_SIZE];
    int num;
    int type;
    struct timeval received;

    // Status:
    bool timedout;
    bool valid;
    long unknown;               // number of unknown packets
    long cnt;                   // number of valid packets
} T_VISCAInterface;

typedef struct tagAVARAGE
{
    long double current;        // avarage time as [ms]
    long double sum;            // sum of the time
    long cnt;                   // number of packets received
} T_Avarage;




/*+=========================================================================+*/
/*|                            PUBLIC VARIABLES                             |*/
/*`========================================================================='*/

/*+=========================================================================+*/
/*|                             LOCAL VARIABLES                             |*/
/*`========================================================================='*/

static char SenderPortName[V24_SZ_PORTNAME] = {'\0'};
static T_VISCAInterface sender;

static char ReceiverPortName[V24_SZ_PORTNAME] = {'\0'};
static T_VISCAInterface receiver;

static T_Avarage avg_ack   = {0.0L,0.0L,0L};
static T_Avarage avg_done  = {0.0L,0.0L,0L};

static unsigned int MyOpenFlags = V24_STANDARD;
static int MyTimeOut = 0;

/* Sequence pattern (counting from 0)
 */
static T_VISCA_Sequence sequences[CMD_MAX_SEQUENCES+1] =
{
    {{0x01, 0x00, 0x01},       3, 3},  // CMD_IfClear          |
    {{0x01, 0x04, 0x00},       4, 3},  // CMD_Power            | on=0x02  off=0x03
    {{0x01, 0x04, 0x07},       4, 3},  // CMD_Zoom             | stop:q=0;s=0  wide:q=3 speed=s  tele=q=2 speed=s
    {{0x01, 0x04, 0x08},       4, 3},  // CMD_Focus            | stop:q=0;s=0  far:q=2  speed=s  near:q=3 speed=s
    {{0x01, 0x04, 0x0B},       4, 3},  // CMD_Iris             | up=0x02  down=0x03
    {{0x01, 0x04, 0x10, 0x05}, 4, 4},  // CMD_WBTrigger        | One Push WB Trigger
    {{0x01, 0x04, 0x18, 0x01}, 4, 4},  // CMD_FocusTrigger     | one-push AF trigger
    {{0x01, 0x04, 0x35},       4, 3},  // CMD_WB               | normal_auto=0x00  one-push-mode=0x03
    {{0x01, 0x04, 0x36, 0x00}, 4, 4},  // CMD_DZoom            | Optical-Digital Zoom Combined
    {{0x01, 0x04, 0x38},       4, 3},  // CMD_FocusMode        | AF_on=0x02  manual_focus=0x03
    {{0x01, 0x04, 0x39},       4, 3},  // CMD_AE               | full_auto=0x00  manual_mode=0x03
    {{0x01, 0x04, 0x47},       7, 3},  // CMD_ZoomDirect       | set direct position
    {{0x01, 0x04, 0x62},       4, 3},  // CMD_Freeze           | on=0x02  off=0x03
    {{0x01, 0x04, 0x74, 0x03}, 4, 3},  // CMD_Title            | off
    {{0x09, 0x04, 0x00},       3, 3},  // CMD_PowerInq         |
    {{0x09, 0x04, 0x38},       3, 3},  // CMD_FocusModeInq     |
    {{0x09, 0x04, 0x48},       3, 3},  // CMD_FocusPositionInq |
    {{0x09, 0x04, 0x39},       3, 3},  // CMD_AEModeInq        |
    {{0x09, 0x04, 0x47},       3, 3},  // CMD_ZoomPosInq       |
    {{0x09, 0x04, 0x4B},       3, 3},  // CMD_IrisPosInq       |
    {{0x09, 0x04, 0x62},       3, 3},  // CMD_FreezeModeInq    |
    {{0x30, 0x01},             2, 2},  // CMD_SetAdress        | Adressvergabe. (normalerweise Broadcast mit 88)
    {{0x77, 0x01},             3, 2},  // CMD_EXT_Turn         | dir: 0=stop 1=left 2=right
    {{0x77, 0x02},             2, 2},  // CMD_EXT_Pairing      |
    {{0x30, 0x02},             2, 2},  // RPL_Address          | SOP=0x90
    {{0x40},                   1, 1},  // RPL_Ack              | SOP=0x90
    {{0x41},                   1, 1},  // RPL_Ack1             | SOP=0x90
    {{0x42},                   1, 1},  // RPL_Ack2             | SOP=0x90
    {{0x50},                   5, 1},  // RPL_Word             | SOP=0x90
    {{0x50},                   2, 1},  // RPL_Byte             | SOP=0x90
    {{0x50},                   1, 1},  // RPL_Done             | SOP=0x90
    {{0x51},                   1, 1},  // RPL_Done1            | SOP=0x90
    {{0x52},                   1, 1},  // RPL_Done2            | SOP=0x90
    {{0x61, 0x41},             2, 2},  // RPL_NotExecutable    | SOP=0x90
    {{0x62, 0x41},             2, 2},  // RPL_NotExecutable    | SOP=0x90
    {{0x61},                   2, 1},  // RPL Error            | SOP=0x90
    {{0x62},                   2, 1},  // RPL Error            | SOP=0x90
    {{0x00},0,0}
};

/* Sequence names (index returned by findCommand is used)
 */
static const char* SequenceNames[CMD_MAX_SEQUENCES+1] =
{
    "??",
    "CMD: IfClear",            // CMD_IfClear          |
    "CMD: Power",              // CMD_Power            | on=0x02  off=0x03
    "CMD: Zoom",               // CMD_Zoom             | stop:q=0;s=0  wide:q=3 speed=s  tele=q=2 speed=s
    "CMD: Focus",              // CMD_Focus            | stop:q=0;s=0  far:q=2  speed=s  near:q=3 speed=s
    "CMD: Iris",               // CMD_Iris             | up=0x02  down=0x03
    "CMD: WBTrigger",          // CMD_WBTrigger        | One Push WB Trigger
    "CMD: FocusTrigger",       // CMD_FocusTrigger     | one-push AF trigger
    "CMD: WB",                 // CMD_WB               | normal_auto=0x00  one-push-mode=0x03
    "CMD: DZoom",              // CMD_DZoom            | Optical-Digital Zoom Combined
    "CMD: FocusMode",          // CMD_FocusMode        | AF_on=0x02  manual_focus=0x03
    "CMD: AE",                 // CMD_AE               | full_auto=0x00  manual_mode=0x03
    "CMD: ZoomDirect",         // CMD_ZoomDirect       | set direct position
    "CMD: Freeze",             // CMD_Freeze           | on=0x02  off=0x03
    "CMD: Title",              // CMD_Title            | off
    "CMD: PowerInq",           // CMD_PowerInq         |
    "CMD: FocusModeInq",       // CMD_FocusModeInq     |
    "CMD: FocusPositionInq",   // CMD_FocusPositionInq |
    "CMD: AEModeInq",          // CMD_AEModeInq        |
    "CMD: ZoomPosInq",         // CMD_ZoomPosInq       |
    "CMD: IrisPosInq",         // CMD_IrisPosInq       |
    "CMD: FreezeModeInq",      // CMD_FreezeModeInq    |
    "CMD: SetAdress",          // CMD_SetAdress        | Adressvergabe. (normalerweise Broadcast mit 88)
    "CMD: EXT_Turn",           // CMD_EXT_Turn         | dir: 0=stop 1=left 2=right
    "CMD: EXT_Pairing",        // CMD_EXT_Pairing      |
    "RPL: Address",            // RPL_Address          |
    "RPL: Ack",                // RPL_Ack              |
    "RPL: Ack Sock1",          // RPL_Ack1             |
    "RPL: Ack Sock2",          // RPL_Ack2             |
    "RPL: Word",               // RPL_Word             |
    "RPL: Byte",               // RPL_Byte             |
    "RPL: Done",               // RPL_Done             |
    "RPL: Done Sock1",         // RPL_Done1            |
    "RPL: Done Sock2",         // RPL_Done2            |
    "RPL: Not Executable",     // RPL_NotExecutable    |
    "RPL: Not Executable",     // RPL_NotExecutable    |
    "RPL: **ERROR**",          // RPL Error            |
    "RPL: **ERROR**"           // RPL Error            |
};


/*+=========================================================================+*/
/*|                      PROTOTYPES OF LOCAL FUNCTIONS                      |*/
/*`========================================================================='*/

void dumpPacketStreams ( void );
void dumpViscaPacket ( T_VISCAInterface *interface, long int diff );
void dumpBadPacket ( T_VISCAInterface *interface );
void dumpErrorMessage ( int rc );

static uint8_t getViscaPacket ( T_VISCAInterface *interface );
static int findCommand ( const uint8_t *sequence, uint8_t len );
static bool setupInterface( T_VISCAInterface *intf, const char *PortName, const char *IntfName );
static const char *logTime ( const struct timeval *tick, bool full );
static const char *milliSeconds ( const struct timeval *tick );
static long int addToAvarage ( const struct timeval *from, const struct timeval *to, T_Avarage *avg );
static bool parseArguments ( int argc, char *argv[] );
static void usage (void);
static void installSignalhandler (void);
static void mySignalHandler (int reason);


/*+=========================================================================+*/
/*|                     IMPLEMENTATION OF THE FUNCTIONS                     |*/
/*`========================================================================='*/



int main( int argc, char *argv[] )
{
    int rc;

    fprintf(stderr,"visca-dump %s -- dump VISCA communication using two ports\ncompiled: "__DATE__"\n\n",VERSION);
    if ( !parseArguments(argc,argv) )
        return 2;

    if ( *SenderPortName == '\0' )
    {
        fputs("ERROR: you have to specify a portname for a sender using parm `-s'!\n", stderr);
        return 1;
    }
    if ( *ReceiverPortName == '\0' )
    {
        fputs("ERROR: you have to specify a portname for a receiver using parm `-r'!\n", stderr);
        return 1;
    }
    installSignalhandler();
    sender.uart = receiver.uart = NULL;

    if ( !setupInterface(&sender,SenderPortName,"CTL") )
    {
        fprintf(stderr,"ERROR: can't open sender port `%s'!\n",SenderPortName);
        return 1;
    }
    if ( !setupInterface(&receiver,ReceiverPortName,"CAM") )
    {
        fprintf(stderr,"ERROR: can't open receiver port `%s'!\n",ReceiverPortName);
        return 1;
    }


    printf("==============================================\n");
    dumpPacketStreams();
    printf("==============================================\n");


    /* At the end of all the stuff, we have close the port. ;-)
     */
    if ( sender.uart )
    {
        rc = v24ClosePort(sender.uart);
        if ( rc != V24_E_OK )
            dumpErrorMessage(rc);
        else
            fputs("INFO: sender port closed!\n", stderr);
    }
    if ( receiver.uart )
    {
        rc = v24ClosePort(receiver.uart);
        if ( rc != V24_E_OK )
            dumpErrorMessage(rc);
        else
            fputs("INFO: receiver port closed!\n", stderr);
    }
    return 0;
}


void dumpPacketStreams ( void )
{
    long int diff;
    bool wait_response=false;
    bool done=false;
    uint8_t rc;
    long int sender_errors = 0;
    long int receiver_errors = 0;
    int dumps = 0;

    do
    {
        // if we have data from the sender/controller, dump it
        if ( v24HaveData(sender.uart) )
        {
            rc = getViscaPacket(&sender);
            if ( rc==VISCA_SUCCESS )
            {
                wait_response = true;
                dumpViscaPacket(&sender,0L);
            }
            else
            {
                if ( sender.cnt > 0)                    // count errors only after a
                    sender_errors++;                    // communication is established
                if ( rc!=VISCA_BAD_HEADER )
                {
                    dumpBadPacket(&sender);
                }
            }
        }
        // if we have data from the receiver/camera, dump it
        if ( v24HaveData(receiver.uart) )
        {
            rc = getViscaPacket(&receiver);
            if ( rc==VISCA_SUCCESS )
            {
                if ( wait_response )
                {
                    if ( receiver.type==VISCA_TYPE_RESPONSE_ACK )
                    {
                        diff=addToAvarage(&sender.received,&receiver.received,&avg_ack);
                    }
                    else
                    {
                        diff=addToAvarage(&sender.received,&receiver.received,&avg_done);
                        wait_response = false;
                    }
                }
                else
                    diff = 0L;
                dumpViscaPacket(&receiver,diff);
                dumps++;
                if ( dumps >=100 )
                {
                    dumps = 0;
                    printf("~~~~~~~~~~~~~~~~~~~ ack=%Lf (%ld) | done=%Lf (%ld) [ms] | unknown=%ld/%ld | errors=%ld/%ld\n",
                           avg_ack.current,avg_ack.cnt,
                           avg_done.current,avg_done.cnt,
                           sender.unknown,receiver.unknown,
                           sender_errors,receiver_errors);
                }
            }
            else
            {
                if ( receiver.cnt > 0)                  // count errors only after a
                    receiver_errors++;                  // communication is established
                if ( rc!=VISCA_BAD_HEADER )
                {
                    dumpBadPacket(&receiver);
                }
            }
        }
    }
    while ( !done );
}

/* Dump a VISCA packet and it's statistic information. The 'received' timestamp
 * is used as time reference using "logTime(false)". The packet data is dumped
 * as raw data in HEX. The data is used to find the command and to determine
 * it's name. The parameters aren't explained.
 *
 * Each packet is logged in an single line:
 *
 * (1)             (2)  (3)                                                (4)  (5)     (6)
 * _v_____________ _v__ _v_____________________________________________    _v__ _v____  _v________________
 * "HH:MM:SS[mmmm] NNN: xx xx xx xx xx xx xx xx xx xx xx xx xx xx xx xx - {ssss/sss.ss} tttttttttttttttttt"
 *
 * The logged fields are:
 *
 * (1) the timestamp including milliseconds.
 * (2) the "name" of the interface. The sender is called "CTL" and the receiver "CAM".
 * (3) the raw dump of the packet
 * (4) the time difference in [ms] between the command from the sender and the reply.
 * (5) the average reply duration. The type (A=ACK D=DATA) is appended here.
 * (6) the name of the command if found.
 */
void dumpViscaPacket ( T_VISCAInterface *interface, long int diff )
{
    long double avg;
    char type;
    int cmd;
    int i;

    if ( !interface->valid )
        return;
    printf("%s %3.3s: ",logTime(&(interface->received),false),interface->name);
    for ( i=0; i<VISCA_MAX_SIZE; i++ )
    {
        if ( i < interface->num )
            printf("%2.2X ",interface->buffer[i]);
        else
            printf("   ");
    }
    if ( interface->type==VISCA_TYPE_RESPONSE_ACK )
    {
        avg = avg_ack.current;
        type = 'A';
    }
    else
    {
        avg = avg_done.current;
        type = 'D';
    }

    if ( diff )
        printf("{%4.4ld/%6.2Lf%c} ",diff,avg,type);
    else
        printf("{    /       } ");

    cmd = findCommand(interface->buffer,interface->num);
    printf(" - %s\n",SequenceNames[cmd]);
    if ( cmd==0 )
        interface->unknown++;
}

/* Simply a raw dump of the chunk of received data.
 */
void dumpBadPacket ( T_VISCAInterface *interface )
{
    int i;

    printf("%s %3.3s: ",logTime(&(interface->received),false),interface->name);
    for ( i=0; i<VISCA_MAX_SIZE; i++ )
    {
        if ( i < interface->num )
            printf("%2.2X ",interface->buffer[i]);
        else
            printf("   ");
    }
    printf("ERROR\n");
}

void dumpErrorMessage ( int rc )
{
    switch ( rc )
    {
        case V24_E_OK: fputs("error-msg: V24_E_OK\n",stderr); break;
        case V24_E_ILLBAUD: fputs("error-msg: V24_E_ILLBAUD\n",stderr); break;
        case V24_E_ILLDATASZ: fputs("error-msg: V24_E_ILLDATASZ\n",stderr); break;
        case V24_E_ILLHANDLE: fputs("error-msg: V24_E_ILLHANDLE\n",stderr); break;
        case V24_E_ILLTIMEOUT: fputs("error-msg: V24_E_ILLTIMEOUT\n",stderr); break;
        case V24_E_OPEN_LOCK: fputs("error-msg: V24_E_OPEN\n",stderr); break;
        case V24_E_CREATE_LOCK: fputs("error-msg: V24_E_CREATE_LOCK\n",stderr); break;
        case V24_E_KILL_LOCK: fputs("error-msg: V24_E_KILL_LOCK\n",stderr); break;
        case V24_E_LOCK_EXIST: fputs("error-msg: V24_E_LOCK_EXIST\n",stderr); break;
        case V24_E_NOMEM: fputs("error-msg: V24_E_NOMEM\n",stderr); break;
        case V24_E_NULL_POINTER: fputs("error-msg: V24_E_NULL_POINTER\n",stderr); break;
        case V24_E_OPEN: fputs("error-msg: V24_E_OPEN\n",stderr); break;
        case V24_E_READ: fputs("error-msg: V24_E_READ\n",stderr); break;
        case V24_E_WRITE: fputs("error-msg: V24_E_WRITE\n",stderr); break;
        case V24_E_NOT_IMPLEMENTED: fputs("error-msg: V24_E_NOT_IMPLEMENTED\n",stderr); break;
        case V24_E_DBG_STALE_LOCK: fputs("debug-msg: V24_E_DBG_STALE_LOCK\n",stderr); break;
        default:  fputs("error-msg undefined?!?!\n",stderr); break;
    }
}



/*+=========================================================================+*/
/*|                    IMPLEMENTATION OF LOCAL FUNCTIONS                    |*/
/*`========================================================================='*/


/* Receive a VISCA packet. The packet data is stored in `interface->buffer`.
 * The functions returns the "API error codes". If no error occures, teh returned
 * code is `VISCA_SUCCESS`.
 *
 * The timestamp of the first byte received is written to `interface->received`.
 */
static uint8_t getViscaPacket ( T_VISCAInterface *interface )
{
    int cnt_read;
    int pos;

    interface->timedout = false;
    interface->valid = false;

    /* read first byte, the header
     */
    pos = 0;
    cnt_read = v24Read(interface->uart,interface->buffer,sizeof(uint8_t));
    if ( cnt_read <= 0 )
    {
        fprintf(stderr,"ERROR(%s): timeout! No data.\n",interface->name);
        interface->timedout = true;
        return VISCA_TIMEDOUT;
    }
    if ( !(interface->buffer[pos] & 0x80) )
    {
        fprintf(stderr,"ERROR(%s): bad header!\n",interface->name);
        return VISCA_BAD_HEADER;
    }
    gettimeofday(&(interface->received),NULL);

    while ( interface->buffer[pos] != VISCA_TERMINATOR )
    {
        pos++;
        if ( pos >= VISCA_MAX_SIZE )
        {
            fprintf(stderr,"ERROR(%s): overflow! Abort.\n",interface->name);
            interface->num = pos;
            return VISCA_OVERFLOW;
        }
        cnt_read = v24Read(interface->uart,&interface->buffer[pos],sizeof(uint8_t));
        if ( cnt_read <= 0 )
        {
            fprintf(stderr,"ERROR(%s): timeout! Abort.\n",interface->name);
            interface->num = pos + 1;
            interface->timedout = true;
            return VISCA_TIMEDOUT;  //  wie hier mittendrin reagieren? Mit Timeout Error raus
        }
    }
    interface->num = pos + 1;
    if ( interface->num < VISCA_MIN_SIZE )
    {
        fprintf(stderr,"ERROR(%s): pkt to small!\n",interface->name);
        return VISCA_FAILURE;
    }
    interface->type = interface->buffer[1] & 0xF0;
    interface->valid = true;
    interface->cnt++;
    return VISCA_SUCCESS;
}

/* Find a sequence in the list. The first byte of a sequence (the SOP) is skipped.
 * The length of a packet must be longer than 2. The array hold a "comparable length"
 * which is intended to skip the variable content of the parameters. So only the fix
 * part of the sequence is compared.
 *
 * This functions returns a *sequence id* counting from 1! A value of 0 means
 * the sequences was not found. The returned value can be used as index int
 * `SequenceNames`.
 */
static int findCommand ( const uint8_t *sequence, uint8_t len )
{
    uint8_t i;
    int rc;

    if ( len <= 2 )
    {
        fprintf(stderr,"ERROR: findCommand: to short (%d)\n",len);
        return -1;
    }
    for ( i=0; i<CMD_MAX_SEQUENCES; i++ )
    {
        if ( sequences[i].length == len-2 )
        {
            rc = memcmp(&sequence[1],sequences[i].seq,sequences[i].comparable);
            if ( rc==0 )
            {
                //~ __DEBUG("findCommand: found %d\n",i);
                return i+1;
            }
        }
    }
#ifdef DEBUG
    fprintf(stderr,"DEBUG: findCommand: seq not found!\n");
    //~ dbg_ReportDump(sequence,len);
#endif
    return 0;
}

/* Setup the serial interfaces using the ezV24 library.
 */
static bool setupInterface ( T_VISCAInterface *intf, const char *PortName, const char *IntfName )
{
    int rc;

    if ( intf==NULL  || PortName==NULL )
    {
        fputs("ERROR: setupInterface(): invalid parameter!\n", stderr);
        return false;
    }

    /* First we have to open the port.
     */
    intf->uart = v24OpenPort(PortName,MyOpenFlags);
    if ( intf->uart == NULL )
    {
        fputs("ERROR: sorry, open failed!\n", stderr);
        return false;
    }
    fprintf(stderr,"INFO: port '%s' opened!\n",PortName);
    intf->timedout = false;
    intf->valid = false;
    intf->unknown = 0;
    intf->cnt = 0;

    /* than we have to configure the port.
     */
    rc = v24SetParameters(intf->uart, V24_B9600, V24_8BIT, V24_NONE);
    if ( rc != V24_E_OK )
    {
        dumpErrorMessage(rc);
        v24ClosePort(intf->uart);
        return false;
    }
    if ( MyTimeOut > 0 )
    {
        rc=v24SetTimeouts(intf->uart,MyTimeOut*10);
        if ( rc==V24_E_NOT_IMPLEMENTED )
            fputs("INFO: setup of timeout is not available!\n",stderr);
        else if ( rc!=V24_E_OK )
        {
            dumpErrorMessage(rc);
            v24ClosePort(intf->uart);
            return false;
        }
        else
            fprintf(stderr,"INFO: timeout is set to %dsec\n",MyTimeOut);
    }

    /* remember the name of the interface
     */
    if ( IntfName )
    {
        strncpy(intf->name,IntfName,SZ_INTERFACE_NAME);
        intf->name[SZ_INTERFACE_NAME] = '\0';
    }
    else
        intf->name[0] = '\0';
    return true;
}

/* return a timestamp as string. The returned pointer references a static
 * buffer, so the next call to this functions modifies the content!
 */
static const char *logTime ( const struct timeval *tick, bool full )
{
    static char time_str[50];
    struct tm *t;

    t = localtime(&(tick->tv_sec));
    if ( t )
    {
        if ( full )
        {
            snprintf(time_str,sizeof(time_str),
                     "%2.2d.%2.2d.%4.4d %2.2d:%2.2d:%2.2d[%4.4d]",
                     t->tm_mday,t->tm_mon,t->tm_year+1900,
                     t->tm_hour,t->tm_min,t->tm_sec,
                     (int)(tick->tv_usec/1000));
            time_str[sizeof(time_str)-1]='\0';
        }
        else
        {
            snprintf(time_str,sizeof(time_str),
                     "%2.2d:%2.2d:%2.2d[%4.4d]",
                     t->tm_hour,t->tm_min,t->tm_sec,
                     (int)(tick->tv_usec/1000));
            time_str[sizeof(time_str)-1]='\0';
        }
    }
    else
    {
        strcpy(time_str,"NULL");
    }
    return time_str;
}

/* return only the [ms] as string. The returned pointer references a static
 * buffer, so the next call to this functions modifies the content!
 */
static const char *milliSeconds ( const struct timeval *tick )
{
    static char time_str[50];

    if ( tick )
    {
        snprintf(time_str,sizeof(time_str)-1,
                 "[%4.4d]",(int)(tick->tv_usec/1000));
        time_str[sizeof(time_str)-1]='\0';
    }
    else
    {
        strcpy(time_str,"[----]");
    }
    return time_str;
}

/* Calculate difference in timestamps and add this to an avarage value. The
 * difference is returned.
 */
static long int addToAvarage ( const struct timeval *from, const struct timeval *to, T_Avarage *avg )
{
    long int diff;

    if ( !from || !to || !avg )
    {
        fputs("error: addToAvarage(): NULL parameter\n", stderr);
        return 0.0;
    }
    if ( from->tv_sec > to->tv_sec )
    {
        fputs("error: addToAvarage(): 'from' later than 'to'\n", stderr);
        return 0.0;
    }
    diff = (long int)(to->tv_sec-from->tv_sec)*1000L+(long int)(to->tv_usec-from->tv_usec)/1000L;
    if ( diff < AVG_OUTLIER )
    {
        avg->sum += (long double)diff;
        avg->cnt++;
        avg->current = avg->sum/avg->cnt;
    }
    else
    {
        fputs("error: addToAvarage(): skip outlier\n", stderr);
    }
    return diff;
}

/* Parse the command line arguments.
 */
static bool parseArguments ( int argc, char *argv[] )
{
    int Done = 0;
    optind = 1;   /* start without prog-name */
    do
    {
        switch ( getopt(argc, argv, "lDht:r:s:") )
        {
            case 'r':
                if ( optarg )
                {
                    strncpy(ReceiverPortName, optarg, V24_SZ_PORTNAME);
                    ReceiverPortName[V24_SZ_PORTNAME] = '\0';
                    fprintf(stderr, "info: receiver port `%s'\n", ReceiverPortName);
                }
                else
                {
                    fputs("error: missing parameter for -r\n", stderr);
                    return false;
                }
                break;
            case 's':
                if ( optarg )
                {
                    strncpy(SenderPortName, optarg, V24_SZ_PORTNAME);
                    SenderPortName[V24_SZ_PORTNAME] = '\0';
                    fprintf(stderr, "info: sender port `%s'\n", SenderPortName);
                }
                else
                {
                    fputs("error: missing parameter for -s\n", stderr);
                    return false;
                }
                break;
            case 't':
                if ( optarg )
                {
                    MyTimeOut=atoi(optarg);
                    if ( MyTimeOut==0 && *optarg!='0' )
                        fputs("warning: invalid timeout parm ingnored!\n",stderr);
                }
                break;
            case 'l':
                MyOpenFlags |= V24_LOCK;
                fputs("info: open with V24_LOCK\n", stderr);
                break;
            case 'D':
                MyOpenFlags |= V24_DEBUG_ON;
                fputs("info: open with V24_DEBUG_ON\n", stderr);
                break;
            case 'h':     // user want's help
            case '?':     // getopt3() reports invalid option
                usage();
                return false;
            default:
                Done = 1;
        }
    } while (!Done);
    return true;
}

static void usage ( void )
{
    fprintf(stderr, "SYNOPSIS\n");
    fprintf(stderr, "\tvisca-dump [options]\n");
    fprintf(stderr, "\nDESCRIPTION\n");
    fprintf(stderr, "\tThis program dumps the VISCA communication of a controller and a\n");
    fprintf(stderr, "\tcamera. The TX line of both devices must be connected to a UART.\n");
    fprintf(stderr, "\nOPTIONS\n");
    fprintf(stderr, "-h\tdisplay this help page.\n");
    fprintf(stderr, "-r dev\tserial port <dev> connected to the receiver (camera).\n");
    fprintf(stderr, "-s dev\tserial port <dev> connected to the sender (controller).\n");
    fprintf(stderr, "-t sec\tset timeout to <sec> seconds.\n");
    fprintf(stderr, "-l\tV24: lock the serial port.\n");
    fprintf(stderr, "-D\tV24: enable debugging.\n");
}


static void installSignalhandler ( void )
{
    signal(SIGINT, mySignalHandler);
    signal(SIGTERM, mySignalHandler);
}


static void mySignalHandler ( int reason )
{
    if ( sender.uart )
        v24ClosePort(sender.uart);
    if ( receiver.uart )
        v24ClosePort(receiver.uart);
    fprintf(stderr,"**ABORT**\n");
    exit(99);
}


/* ==[End of file]========================================================== */
