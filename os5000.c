#ifndef __OS5000_C__
#define __OS5000_C__
#include "serial.h"
#include "os5000.h"

static HANDLE os5000 = INVALID_HANDLE_VALUE;


#define F(C) #C "%f"
#define F2(C) #C "%f" #C "x%f"  #C "y%f"
#define F3(C) F2(C)  #C "z%f"

#define FORMAT "$" F(C) F(P) F(R) F(T) F(D) F3(M) F3(A) F2(G) "L%d"

#define ARG_1(C)  &(osdata.C),
#define ARG_2(C)  &(osdata.C), &(osdata.C##x), &(osdata.C##y),
#define ARG_3(C)  ARG_2(C)  &(osdata.C##z),

#define ARGS   ARG_1(C)                     \
           ARG_1(P)                         \
           ARG_1(R)                         \
           ARG_1(T)                         \
           ARG_1(D)                         \
           ARG_3(M)                         \
           ARG_3(A)                         \
           ARG_2(G)                         \
            &(osdata.L)
#undef _ARG_1
#undef _ARG_2
#undef _ARG_3

void initOS5000()
{
    if (os5000 == INVALID_HANDLE_VALUE) {
        os5000 = initSerial(os5000port, os5000baud, 8, 2, 0, 0, 0, 0);
    }
}


static int parseOs5000(char *s)
{
    //fprintf(stderr, "Parsing: %s\n", s);
    return sscanf(s, FORMAT, ARGS);
}

int readOs5000()
{
    char buffer[1000], *bp;
    int c;
    if (os5000 == INVALID_HANDLE_VALUE) {
        os5000 = initSerial(os5000port, os5000baud, 8, 2, 0, 0, 0, 0);
    }
    if (os5000 == INVALID_HANDLE_VALUE) {
        printf("ouch\n");
        return -1;
    }
    while ((c = SerialGetc(os5000)) != '$') {
    }                           //wait for   '$'
    bp = buffer;
    *bp++ = c;
    while ((c = SerialGetc(os5000)) != '*') {
        if (bp < buffer + sizeof(buffer) - 1)
            *bp++ = c;
    }
    *bp = 0;
    if (parseOs5000(buffer) < 3)
        return -2;
    memcpy(&os5000data, &osdata, sizeof(osdata));
    FlushRx(os5000);
    return 0;
}


/*
int os5000port = 8;
int os5000baud = 115200;
int main()
{
    char *teststr =
        "$C271.3P15.2R1.5T30.6D0.0033M194.53Mx44.89My-85.35Mz168.94A1.044Ax0.276Ay0.027Az1.007G-0.01Gx0.00Gy-122.62L14045S0*26";
    int scount = 0;
    fprintf(stderr, "%s\n", FORMAT);
    fprintf(stderr, "%s\n", teststr);
    scount = parseOs5000(teststr);
    initOS5000();
    while (1) {
        readOs5000();
        DEBUGN(osdata.C);
    }
    system("pause");
}*/
#endif //__OS5000_C__
