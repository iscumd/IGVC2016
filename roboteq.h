#ifndef __ROBOTEQ_H__
#define __ROBOTEQ_H__

#ifndef ABS
#define ABS(val) (val > 0 ? val: -val)
#endif

#ifndef SIGN
#define SIGN(x) ((x)>0? 1: -1)
#endif

#ifdef __cplusplus
class Roboteq {
private:
    int portNum;
    char LFWD;
    char RFWD;
    char LREV;
    char RREV;
    string modleName;
    int modleType;
    string wheelModeName;
    int wheelModeNum;

public:
    Roboteq();
    Roboteq(int p_portNum, string p_modleName);
    Roboteq(int p_portNum, string p_modleName, string wheelModeName);
    void init(int p_portNum);
    void Roboteq::sendSpeed(int leftspeed, int rightspeed);

};

#else
extern int roboteqPort;

void initRoboteq();
void sendSpeed(int leftspeed, int rightspeed);
#endif // __cplusplus

#endif // __ROBOTEQ_H__
