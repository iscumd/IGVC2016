#ifndef __OS5000_H__
#define __OS5000_H__

#define ELEMENT(C)  C,
#define ELEMENT2(C)  C, C##x, C##y
#define ELEMENT3(C) ELEMENT2(C),  C##z,

struct _OSDATA_ {
    float
     ELEMENT(C)
     ELEMENT(P)
     ELEMENT(R)
     ELEMENT(T)
     ELEMENT(D)
     ELEMENT3(M)
     ELEMENT3(A)
     ELEMENT2(G);
    int L;
};

struct _OSDATA_ osdata;
extern struct _OSDATA_ os5000data;
extern int os5000port, os5000baud;

void initOS5000();
int readOs5000();
#undef ELEMENT
#undef ELEMENT2
#undef ELEMENT3


#endif //__OS5000_H__
