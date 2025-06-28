#ifndef INC_SETTINGS_H_
#define INC_SETTINGS_H_

// Select the microcontroller number (uncomment needed)
#define WHEEL_1
// #define WHEEL_2
// #define WHEEL_3
// #define WHEEL_4
// #define WHEEL_5
// #define WHEEL_6

// Speed of motor before gearbox in rad/s
#define MOTOR_SPEED 895.8775050487

// Gear ratio of reducer
#define GEAR_RATIO 59

/* Parameters for the current wheel-----------------------------------------*/
#ifdef WHEEL_1

#define SERVO_FLAG 1

// Adjusting the servo angle
#define SERVO_ADJUSTMENT 27

#define REVERCE 0

// CPR - counts per revolution. If you are using both channels and rising/falling edges of encoder, CPR=PPR*4.
#define CPR 1999

// Min and max duty-cycle of PWM
#define MIN_DUTY 0
#define MAX_DUTE 999

/* --- PID controllers settings ---*/

/* Current controller */
#define CUR_KP 0
#define CUR_KI 0
#define CUR_KD 0
#define CUR_DT 0
#define CUR_KT 0 // Back-calc antiwindup gain (currently use clamping so its useless)

/* Velocity controller */
#define VEL_KP 150
#define VEL_KI 1500
#define VEL_KD 0
#define VEL_DT 0.004
#define VEL_KT 0 // Back-calc antiwindup gain (currently use clamping so its useless)

/* Angle controller */
#define ANG_KP 3
#define ANG_KI 0
#define ANG_KD 0
#define ANG_DT 0.001
#define ANG_KT 0 // Back-calc antiwindup gain (currently use clamping so its useless)
#define ANG_TOLERANCE 0.02

#endif  /*------------------------------------------------------------------*/

#ifdef WHEEL_2

#define SERVO_FLAG 0

// Adjusting the servo angle
#define SERVO_ADJUSTMENT 27

#define REVERCE 0

// CPR - counts per revolution. If you are using both channels and rising/falling edges of encoder, CPR=PPR*4.
#define CPR 1999

// Min and max duty-cycle of PWM
#define MIN_DUTY 0
#define MAX_DUTE 999

/* --- PID controllers settings ---*/

/* Current controller */
#define CUR_KP 0
#define CUR_KI 0
#define CUR_KD 0
#define CUR_DT 0
#define CUR_KT 0 // Back-calc antiwindup gain (currently use clamping so its useless)

/* Velocity controller */
#define VEL_KP 150
#define VEL_KI 1500
#define VEL_KD 0
#define VEL_DT 0.004
#define VEL_KT 0 // Back-calc antiwindup gain (currently use clamping so its useless)

/* Angle controller */
#define ANG_KP 3
#define ANG_KI 0
#define ANG_KD 0
#define ANG_DT 0.001
#define ANG_KT 0 // Back-calc antiwindup gain (currently use clamping so its useless)
#define ANG_TOLERANCE 0.02

#endif  /*------------------------------------------------------------------*/

#ifdef WHEEL_3

#define SERVO_FLAG 1

// Adjusting the servo angle
#define SERVO_ADJUSTMENT 27

#define REVERCE 0

// CPR - counts per revolution. If you are using both channels and rising/falling edges of encoder, CPR=PPR*4.
#define CPR 1999

// Min and max duty-cycle of PWM
#define MIN_DUTY 0
#define MAX_DUTE 999

/* --- PID controllers settings ---*/

/* Current controller */
#define CUR_KP 0
#define CUR_KI 0
#define CUR_KD 0
#define CUR_DT 0
#define CUR_KT 0 // Back-calc antiwindup gain (currently use clamping so its useless)

/* Velocity controller */
#define VEL_KP 150
#define VEL_KI 1500
#define VEL_KD 0
#define VEL_DT 0.004
#define VEL_KT 0 // Back-calc antiwindup gain (currently use clamping so its useless)

/* Angle controller */
#define ANG_KP 3
#define ANG_KI 0
#define ANG_KD 0
#define ANG_DT 0.001
#define ANG_KT 0 // Back-calc antiwindup gain (currently use clamping so its useless)
#define ANG_TOLERANCE 0.02

#endif  /*------------------------------------------------------------------*/

#ifdef WHEEL_4

#define SERVO_FLAG 1

// Adjusting the servo angle
#define SERVO_ADJUSTMENT 27

#define REVERCE 0

// CPR - counts per revolution. If you are using both channels and rising/falling edges of encoder, CPR=PPR*4.
#define CPR 1999

// Min and max duty-cycle of PWM
#define MIN_DUTY 0
#define MAX_DUTE 999

/* --- PID controllers settings ---*/

/* Current controller */
#define CUR_KP 0
#define CUR_KI 0
#define CUR_KD 0
#define CUR_DT 0
#define CUR_KT 0 // Back-calc antiwindup gain (currently use clamping so its useless)

/* Velocity controller */
#define VEL_KP 150
#define VEL_KI 1500
#define VEL_KD 0
#define VEL_DT 0.004
#define VEL_KT 0 // Back-calc antiwindup gain (currently use clamping so its useless)

/* Angle controller */
#define ANG_KP 3
#define ANG_KI 0
#define ANG_KD 0
#define ANG_DT 0.001
#define ANG_KT 0 // Back-calc antiwindup gain (currently use clamping so its useless)
#define ANG_TOLERANCE 0.02

#endif  /*------------------------------------------------------------------*/

#ifdef WHEEL_5

#define SERVO_FLAG 0

// Adjusting the servo angle
#define SERVO_ADJUSTMENT 27

#define REVERCE 0

// CPR - counts per revolution. If you are using both channels and rising/falling edges of encoder, CPR=PPR*4.
#define CPR 1999

// Min and max duty-cycle of PWM
#define MIN_DUTY 0
#define MAX_DUTE 999

/* --- PID controllers settings ---*/

/* Current controller */
#define CUR_KP 0
#define CUR_KI 0
#define CUR_KD 0
#define CUR_DT 0
#define CUR_KT 0 // Back-calc antiwindup gain (currently use clamping so its useless)

/* Velocity controller */
#define VEL_KP 150
#define VEL_KI 1500
#define VEL_KD 0
#define VEL_DT 0.004
#define VEL_KT 0 // Back-calc antiwindup gain (currently use clamping so its useless)

/* Angle controller */
#define ANG_KP 3
#define ANG_KI 0
#define ANG_KD 0
#define ANG_DT 0.001
#define ANG_KT 0 // Back-calc antiwindup gain (currently use clamping so its useless)
#define ANG_TOLERANCE 0.02

#endif  /*------------------------------------------------------------------*/

#ifdef WHEEL_6

#define SERVO_FLAG 1

// Adjusting the servo angle
#define SERVO_ADJUSTMENT 27

#define REVERCE 0

// CPR - counts per revolution. If you are using both channels and rising/falling edges of encoder, CPR=PPR*4.
#define CPR 1999

// Min and max duty-cycle of PWM
#define MIN_DUTY 0
#define MAX_DUTE 999

/* --- PID controllers settings ---*/

/* Current controller */
#define CUR_KP 0
#define CUR_KI 0
#define CUR_KD 0
#define CUR_DT 0
#define CUR_KT 0 // Back-calc antiwindup gain (currently use clamping so its useless)

/* Velocity controller */
#define VEL_KP 150
#define VEL_KI 1500
#define VEL_KD 0
#define VEL_DT 0.004
#define VEL_KT 0 // Back-calc antiwindup gain (currently use clamping so its useless)

/* Angle controller */
#define ANG_KP 3
#define ANG_KI 0
#define ANG_KD 0
#define ANG_DT 0.001
#define ANG_KT 0 // Back-calc antiwindup gain (currently use clamping so its useless)
#define ANG_TOLERANCE 0.02

#endif  /*------------------------------------------------------------------*/

#endif /* INC_SETTINGS_H_ */