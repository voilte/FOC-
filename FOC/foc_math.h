
#ifndef _FOC_MATH_H
#define _FOC_MATH_H

#include "foc_type.h"
#include "arm_math.h"





#define     _PI_2               1.57079632679f
#define     _2PI                6.283185307179586f      /*2x圆周率*/
#define     _PI_3               1.047197551196597f      /*60° 弧度制*/
#define     SQRT_3              1.732050807568877f      /*根号3*/
#define     SQRT_3_DIV_2        0.866025403784438f      /*根号3 /2 */
#define     _1_DIV_SQRT_3       0.577350269189625f      /*1除根号3*/
#define     _2_DIV_SQRT_3       1.154700538379251f      /*2除根号3*/



/*限制范围*/
#define LIMIT_RANGE(var, min, max) ((var)<(min)?(min):((var)>(max)?(max):(var)))        /*范围限制宏*/
/*一阶滤波*/
#define FirstOrderRC_LPF(Yn_1,Xn,a) Yn_1 = (1-a)*Yn_1 + a*Xn;                           /* Yn:out;Xn:in;a:系数 */


int8_t FOC_SIGN_Judgment(float val);
extern alphabeta_t FOC_Clarke(abc_t input);
extern abc_t FOC_Rev_Clarke(alphabeta_t input);
extern qd_t Foc_Park(alphabeta_t input, float ElAngle);
extern alphabeta_t FOC_Rev_Park(qd_t input, float ElAngle);
extern float Limit_Angle(float angle);
float _MecAngle_to_ElAngle(float MecAngle, uint32_t pole_pairs);
float FOC_ERR(float ElAngle1, float ElAngle2);



#endif
