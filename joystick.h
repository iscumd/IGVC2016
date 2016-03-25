#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__
/*
 * structure to hold joystick data
 */

struct JoystickData {

    int num_of_axis;
    int num_of_buttons;
    char joystick_name[80];
    char button[20];
    int buttons;
    double axis[20];
    int hat;
};
#define AXISRANGE 32767.0

/*
 * There is no reason why we can't have more than one joy stick.
 * However, to keep the code simple we read only one joystick.
 *
 */
#ifndef BIT
#define BIT(k) ((1)<<(k))
#endif

#define A_BTN BIT(0)    // 0000 0000 0000 0001
#define B_BTN BIT(1)    // 0000 0000 0000 0010
#define X_BTN BIT(2)    // 0000 0000 0000 0100
#define Y_BTN BIT(3)    // 0000 0000 0000 1000
#define LB_BTN BIT(4)   // 0000 0000 0001 0000
#define RB_BTN BIT(5)   // 0000 0000 0010 0000
#define BACK_BTN BIT(6) // 0000 0000 0100 0000
#define START_BTN BIT(7)// 0000 0000 1000 0000
#define LTS_BTN BIT(8)  // 0000 0001 0000 0000
#define RTS_BTN BIT(9)  // 0000 0010 0000 0000

int joystick();//check if joystick is present and pool data.
extern struct JoystickData joy0;

#endif //__JOYSTICK_H__
