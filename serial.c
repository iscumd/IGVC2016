#ifndef __SERIAL_C__
#define __SERIAL_C__

#include "serial.h"

#ifndef DEBUG
#define DEBUG(x)    fprintf(stderr, "%s = %g\n", #x,  ((double) x))     //cout << endl << #x << " = " << x << endl
#endif

#if !defined(LINUX)
// Flow control flags
#define FC_DTRDSR 0x01
#define FC_RTSCTS 0x02
#define FC_XONXOFF 0x04

// ascii definitions
#define ASCII_BEL 0x07
#define ASCII_BS 0x08
#define ASCII_LF 0x0A
#define ASCII_CR 0x0D
#define ASCII_XON 0x11
#define ASCII_XOFF 0x13

// variables used with the com port

#ifndef Read_Timeout
#define Read_Timeout 50
#endif

#ifndef NNDEBUB
#define NNDEBUG(x) fprintf(stderr, "%s = %g\n", #x, ((double) x))
#endif

//#define SerialPort(PortNum, BaudRate) initSerial(PortNum, BaudRate, int dataBit, int parity, int cts, int rts, int dsr, int dtr)

//------------------------------------------------------------------------------
HANDLE initSerial(int PortNum, int BaudRate, int dataBit, int parity,
                  int cts, int rts, int dsr, int dtr)
{
    HANDLE hCom;
    char serfilename[512];
    BOOL bPortReady;            // Not used except for debugging
    DCB dcb;
    COMMTIMEOUTS CommTimeouts;

    sprintf(serfilename, "%s%d", "\\\\.\\com", PortNum);

    hCom = CreateFile(serfilename, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, //FILE_FLAG_OVERLAPPED,
                      0);

    //  DEBUG(hCom);
    if (hCom == INVALID_HANDLE_VALUE) {
        //     fprintf(stderr, "Error: COM%d(%s):%d\n", PortNum, serfilename, GetLastError());
//        if (GetLastError() == ERROR_FILE_NOT_FOUND)
//            printf("Serial port %d does not exist \n", PortNum);
        return hCom;
    }

    bPortReady = SetupComm(hCom, 32, 128);      // set buffer sizes
    bPortReady = GetCommState(hCom, &dcb);
    dcb.BaudRate = BaudRate;
    dcb.ByteSize = dataBit;
    dcb.StopBits = ONESTOPBIT;
    //dcb.fNull  = FALSE;
    if (parity == 0)
        dcb.Parity = NOPARITY;
    else if (parity == 1)
        dcb.Parity = ODDPARITY;
    else if (parity == 2)
        dcb.Parity = EVENPARITY;
    else if (parity == 3)
        dcb.Parity = MARKPARITY;
    else
        dcb.Parity = NOPARITY;  //default

    dcb.fAbortOnError = TRUE;

/**set XON/XOFF**/
    dcb.fOutX = FALSE;          // XON/XOFF off for transmit
    dcb.fInX = FALSE;           // XON/XOFF off for receive
/**set RTSCTS**/
    if (cts == 1)
        dcb.fOutxCtsFlow = TRUE;
    else
        dcb.fOutxCtsFlow = FALSE;       //default is FALSE
    //dcb.fOutxCtsFlow = FALSE;   // turn off CTS flow control
    if (rts == 0)
        dcb.fRtsControl = FALSE;
    else if (rts == 1)
        dcb.fRtsControl = TRUE;
    else if (rts == 2)
        dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
    else
        dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;        //default is HANDSHAKE
    //dcb.fRtsControl = RTS_CONTROL_ENABLE;//
/** set DSRDTR**/
    if (dsr == 1)
        dcb.fOutxDsrFlow = TRUE;
    else
        dcb.fOutxDsrFlow = FALSE;       //default is FALSE
    //dcb.fOutxDsrFlow = FALSE;   // turn off DSR flow control
    if (dtr == 0)
        dcb.fDtrControl = DTR_CONTROL_DISABLE;  //
    else if (dtr == 1)
        dcb.fDtrControl = DTR_CONTROL_ENABLE;   //
    else if (dtr == 2)
        dcb.fDtrControl = DTR_CONTROL_HANDSHAKE;        //
    else
        dcb.fDtrControl = DTR_CONTROL_DISABLE;  //default DISABLE

    bPortReady = SetCommState(hCom, &dcb);

    if (bPortReady == 0) {
        DEBUGN(GetLastError());
    }
    //DEBUGN(bPortReady);

// Communication timeouts are optional

    bPortReady = GetCommTimeouts(hCom, &CommTimeouts);

    if (bPortReady == 0) {
        DEBUGN(GetLastError());
    }
    //DEBUGN(bPortReady);

    CommTimeouts.ReadIntervalTimeout = Read_Timeout;    //Was 5000;
    CommTimeouts.ReadTotalTimeoutConstant = Read_Timeout;       //Was 5000;
    CommTimeouts.ReadTotalTimeoutMultiplier = Read_Timeout / 5; //Was 1000;
    CommTimeouts.WriteTotalTimeoutConstant = 5000;
    CommTimeouts.WriteTotalTimeoutMultiplier = 1000;

    bPortReady = SetCommTimeouts(hCom, &CommTimeouts);

    if (bPortReady == 0) {
        DEBUGN(GetLastError());
    }

    return hCom;
}

//------------------------------------------------------------------------------
void SerialClose(HANDLE hCom)
{
    CloseHandle(hCom);
}

//------------------------------------------------------------------------------
void SerialReInit(HANDLE * hCom, int PortNum, int BaudRate, int dataBit,
                  int parity, int cts, int rts, int dsr, int dtr)
{
    SerialClose(*hCom);
    *hCom =
        initSerial(PortNum, BaudRate, dataBit, parity, cts, rts, dsr, dtr);
}

//------------------------------------------------------------------------------
int SerialGetc(HANDLE hCom)
{
    char rxchar;
    BOOL bReadRC;
    static DWORD iBytesRead;
    DWORD dwError = 0;

    while (1) {
        rxchar = '0';
        bReadRC = ReadFile(hCom, &rxchar, 1, &iBytesRead, NULL);
        ClearCommError(hCom, &dwError, NULL);
        if (bReadRC == 1)
            return rxchar;

        //DEBUGN(GetLastError());
    }
    //else return rxchar;          //SERIAL_ERROR;
}

//---------------------------------------------------------------------------------
int SerialGets(HANDLE hCom, char buffer[], int numdesired)
{
    BOOL bReadRC;
    static DWORD iBytesRead;
    DWORD dwError = 0;

//NNDEBUG(numdesired);
    bReadRC = ReadFile(hCom, buffer, numdesired, &iBytesRead, NULL);
    ClearCommError(hCom, &dwError, NULL);
//NNDEBUG(bReadRC);
    //return iBytesRead;
    if (bReadRC == 1)
        return iBytesRead;
    else
        return SERIAL_ERROR;

    /*if (iBytesRead == numdesired) return SERIAL_OK;
       else return SERIAL_ERROR; */
}

//------------------------------------------------------------------------------
void SerialPutc(HANDLE hCom, char txchar)
{
    static DWORD iBytesWritten;
    WriteFile(hCom, &txchar, 1, &iBytesWritten, NULL);

    return;
}

//------------------------------------------------------------------------------
DWORD SerialPuts(HANDLE hCom, char *s)
{
    DWORD iBytesWritten;
    WriteFile(hCom, s, strlen(s), &iBytesWritten, NULL);

    return (iBytesWritten);
}

#else
// defined(LINUX)

#ifndef DEBUGN
#define DEBUGN(x) fprintf(stderr, "%s=%g\n", #x, ((double) x))
#endif

HANDLE initSerial(int PortNum, int BaudRate, int dataBit, int parity,
                  int cts, int rts, int dsr, int dtr)
{

    char portname[80];

    sprintf(portname, "/dev/ttyUSB%d", PortNum);
    fprintf(stderr, "Opening: %s\n", portname);
    return open_serial(portname, BaudRate, dataBit, parity);
}

int FlushRx(HANDLE fd)
{
    flush_serial(fd);
}

void SerialPuts(HANDLE fd, char s[])
{
    write(fd, s, strlen(s));
}

void SerialPutc(HANDLE hCom, char txchar)
{

    write(hCom, &txchar, 1);

}

int SerialGetc(HANDLE hCom)
{
    char rxchar;
    rxchar = '@';
    read(hCom, &rxchar, 1);
    return rxchar;
}

void SerialReInit(HANDLE * hCom, int PortNum, int BaudRate, int dataBit,
                  int parity, int cts, int rts, int dsr, int dtr)
{
    close(*hCom);
    *hCom =
        initSerial(PortNum, BaudRate, dataBit, parity, cts, rts, dsr, dtr);
}

int open_serial(char *port, int baud, int nbits, int parity)
{
    int fd;
    struct termios options;

    DEBUG(baud);
    DEBUG(nbits);
    DEBUGN(parity);
    fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
    if (fd == -1) {
        fprintf(stderr, "Open failure\n");
        return fd;
    }

    if (tcgetattr(fd, &options) == -1) {
        close(fd);
        fprintf(stderr, "Open failure\n");
        return -1;
    }
    cfsetispeed(&options, 9600);
    cfsetospeed(&options, baud);
    options.c_cflag &= ~(CRTSCTS | CSTOPB | CSIZE);
    options.c_cflag |= (nbits == 8 ? CS8 : CS7);
    if (parity == 0)
        options.c_cflag &= ~PARENB;
    else
        options.c_cflag |= PARENB;
    if (parity == 2)
        options.c_cflag &= ~PARODD;
    else
        options.c_cflag |= PARODD;

    options.c_oflag &= ~OPOST;
    options.c_oflag &= ~(NLDLY | CRDLY | TABDLY | BSDLY | VTDLY | FFDLY);

    fcntl(fd, F_SETFL, O_NONBLOCK);
//      fcntl(fd, F_SETFL, FASYNC);

    if (tcsetattr(fd, TCSANOW, &options) == -1) {
        close(fd);
        fprintf(stderr, "Open failure\n");
        return -1;
    }
    return fd;
}

void toggle_bit(int fd, int mask)
{
    int status;
    ioctl(fd, TIOCMGET, &status);
    status ^= mask;
    ioctl(fd, TIOCMSET, &status);
}

void flush_serial(int fd)
{
    struct termios options;
    tcgetattr(fd, &options);
    tcsetattr(fd, TCSAFLUSH, &options);
}

void set_bit(int fd, int mask)
{
    int status;
    ioctl(fd, TIOCMGET, &status);
    status |= mask;
    ioctl(fd, TIOCMSET, &status);
}

void clr_bit(int fd, int mask)
{
    int status;
    ioctl(fd, TIOCMGET, &status);
    status &= ~mask;
    ioctl(fd, TIOCMSET, &status);
}
#endif

//------------------------------------------------------------------------------
static int serial_usage[100];   // Keep track of serial usage 1 used 0 unused
int serial_in_use(int k)
{
    return serial_usage[k];
}

void register_serial(int k, int f)
{
    serial_usage[k] = f;
}

//==============================================================================

#endif //__SERIAL_C__
//   =============================
