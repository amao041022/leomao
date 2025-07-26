

//#include "stm32f10x.h"
//#include "Delay.h"
//#include "OLED.h"
//#include "Servo.h"
//#include "Key.h"
//#include <math.h>

//// 白板与云台的几何配置
//#define DISTANCE_TO_BOARD 1.0f
//#define SQUARE_SIZE       0.66f

//// 云台角度约束（单位：度）
//#define PAN_CENTER  133.0f
//#define TILT_CENTER 115.0f

//#define PAN_MIN_ANGLE 0.0f
//#define PAN_MAX_ANGLE 180.0f
//#define TILT_MIN_ANGLE 0.0f
//#define TILT_MAX_ANGLE 180.0f

//float panAngle = PAN_CENTER;
//float tiltAngle = TILT_CENTER;

//// 坐标转角度，并控制云台
//void PointToAngleAndSet(float x, float y, float z)
//{
//    float panRad = atan2f(x, z);
//    float tiltRad = atan2f(y, sqrtf(x * x + z * z));

//    float panDeg = panRad * 180.0f / 3.1415926f;
//    float tiltDeg = tiltRad * 180.0f / 3.1415926f;

//    panAngle = PAN_CENTER + panDeg;
//    tiltAngle = TILT_CENTER - tiltDeg;

//    // 角度限制
//    if (panAngle > PAN_MAX_ANGLE) panAngle = PAN_MAX_ANGLE;
//    if (panAngle < PAN_MIN_ANGLE) panAngle = PAN_MIN_ANGLE;
//    if (tiltAngle > TILT_MAX_ANGLE) tiltAngle = TILT_MAX_ANGLE;
//    if (tiltAngle < TILT_MIN_ANGLE) tiltAngle = TILT_MIN_ANGLE;

//    Servo_SetPanAngle(panAngle);
//    Servo_SetTiltAngle(tiltAngle);
//}

// //画正方形
//void DrawSquarePath(void)
//{
//    float half = SQUARE_SIZE / 2.0f;
//    float z = DISTANCE_TO_BOARD;

//    float square[4][3] = {
//        {-half,  half, z},  // 左上 A
//        { half,  half, z},  // 右上 B
//        { half, -half, z},  // 右下 C
//        {-half, -half, z},  // 左下 D
//    };
//    for (int i = 0; i < 4; i++) {
//        PointToAngleAndSet(square[i][0], square[i][1], square[i][2]);
//        OLED_ShowString(3, 1, "Status: DRAW ");
//        OLED_ShowNum(1, 6, panAngle, 3);
//        OLED_ShowNum(2, 7, tiltAngle, 3);
//        Delay_ms(1000);
//    }

//    // 回到起点
//    PointToAngleAndSet(square[0][0], square[0][1], square[0][2]);
//    OLED_ShowString(3, 1, "Status: DONE ");
//    Delay_ms(1000);
//}


////分三十步
////void SmoothMoveTo(float targetPan, float targetTilt, int steps, int delayPerStep)
////{
////float deltaPan = (targetPan - panAngle) / steps;
////float deltaTilt = (targetTilt - tiltAngle) / steps;

////for (int i = 0; i < steps; i++) {
////	panAngle += deltaPan;
////	tiltAngle += deltaTilt;
////	Servo_SetPanAngle(panAngle);
////	Servo_SetTiltAngle(tiltAngle);

////	OLED_ShowNum(1, 6, panAngle, 3);
////	OLED_ShowNum(2, 7, tiltAngle, 3);
////	Delay_ms(100);  // 每小步停顿
////	//Delay_ms(delayPerStep);  // 每小步停顿
////}
////}

////void DrawSquarePath(void)
////{
////float half = SQUARE_SIZE / 2.0f;
////float z = DISTANCE_TO_BOARD;

////float square[5][3] = {
////	{-half,  half, z},  // A
////	{ half,  half, z},  // B
////	{ half, -half, z},  // C
////	{-half, -half, z},  // D
////	{-half,  half, z}, 
////		// 回到A
////};

////for (int i = 0; i < 4; i++) {
////	// 计算目标角度
////	float x = square[i+1][0];
////	float y = square[i+1][1];
////	float z = square[i+1][2];

////	float panRad = atan2f(x, z);
////	float tiltRad = atan2f(y, sqrtf(x * x + z * z));

////	float targetPan = PAN_CENTER + panRad * 180.0f / 3.1415926f;
////	float targetTilt = TILT_CENTER - tiltRad * 180.0f / 3.1415926f;

////	OLED_ShowString(3, 1, "Status: DRAW ");
////	SmoothMoveTo(targetPan, targetTilt, 30, 20);  // 30步，每步20ms
////}

////OLED_ShowString(3, 1, "Status: DONE ");
////}


//// 云台归中
//void ResetToCenter(void)
//{
//    panAngle = PAN_CENTER;
//    tiltAngle = TILT_CENTER;

//    Servo_SetPanAngle(panAngle);
//    Servo_SetTiltAngle(tiltAngle);

//    OLED_ShowString(3, 1, "Status: RESET");
//    OLED_ShowNum(1, 6, panAngle, 3);
//    OLED_ShowNum(2, 7, tiltAngle, 3);
//    Delay_ms(500);
//}

//int main(void)
//{
//    OLED_Init();
//    Servo_Init();
//    Key_Init();

//    ResetToCenter(); // 开机默认归中

//    OLED_ShowString(1, 1, "Pan:");
//    OLED_ShowString(2, 1, "Tilt:");
//    OLED_ShowString(3, 1, "Status: IDLE");

//    while (1)
//    {
//        uint8_t key = Key_GetNum();
//        if (key == 3)
//        {
//            DrawSquarePath();  // 画正方形
//        }
//        else if (key == 1)
//        {
//            ResetToCenter();   // 回中
//        }

//        // 实时显示当前角度
//        OLED_ShowNum(1, 6, panAngle, 3);
//        OLED_ShowNum(2, 7, tiltAngle, 3);

//        Delay_ms(20);
//    }
//}








////#include "stm32f10x.h"
////#include "Delay.h"
////#include "OLED.h"
////#include "Servo.h"
////#include "Key.h"
////#include <math.h>

////// 白板与云台的几何配置
////#define DISTANCE_TO_BOARD 1.0f
////#define SQUARE_SIZE       0.3f

////// 云台角度约束（单位：度）
////#define PAN_CENTER  135.0f
////#define TILT_CENTER 135.0f

////#define PAN_MIN_ANGLE 0.0f
////#define PAN_MAX_ANGLE 180.0f
////#define TILT_MIN_ANGLE 0.0f
////#define TILT_MAX_ANGLE 180.0f

////float panAngle = PAN_CENTER;
////float tiltAngle = TILT_CENTER;

////// 坐标转角度，并控制云台、‘
////void PointToAngleAndSet(float x, float y, float z)
////{
////    float panRad = atan2f(x, z);
////    float tiltRad = atan2f(y, sqrtf(x * x + z * z));

////    float panDeg = panRad * 180.0f / 3.1415926f;
////    float tiltDeg = tiltRad * 180.0f / 3.1415926f;

////    panAngle = PAN_CENTER - panDeg;  // 反转x轴方向
////    tiltAngle = TILT_CENTER + tiltDeg;  // 反转y轴方向

////    // 角度限制
////    if (panAngle > PAN_MAX_ANGLE) panAngle = PAN_MAX_ANGLE;
////    if (panAngle < PAN_MIN_ANGLE) panAngle = PAN_MIN_ANGLE;
////    if (tiltAngle > TILT_MAX_ANGLE) tiltAngle = TILT_MAX_ANGLE;
////    if (tiltAngle < TILT_MIN_ANGLE) tiltAngle = TILT_MIN_ANGLE;

////    Servo_SetPanAngle(panAngle);
////    Servo_SetTiltAngle(tiltAngle);
////}

//// //画正方形
//////void DrawSquarePath(void)
//////{
//////    float half = SQUARE_SIZE / 2.0f;
//////    float z = DISTANCE_TO_BOARD;

//////    float square[4][3] = {
//////        {-half,  half, z},  // 左上 A
//////        { half,  half, z},  // 右上 B
//////        { half, -half, z},  // 右下 C
//////        {-half, -half, z},  // 左下 D
//////    };
//////    for (int i = 0; i < 4; i++) {
//////        PointToAngleAndSet(square[i][0], square[i][1], square[i][2]);
//////        OLED_ShowString(3, 1, "Status: DRAW ");
//////        OLED_ShowNum(1, 6, panAngle, 3);
//////        OLED_ShowNum(2, 7, tiltAngle, 3);
//////        Delay_ms(1000);
//////    }

//////    // 回到起点
//////    PointToAngleAndSet(square[0][0], square[0][1], square[0][2]);
//////    OLED_ShowString(3, 1, "Status: DONE ");
//////    Delay_ms(1000);
//////}


//////分三十步
////void SmoothMoveTo(float targetPan, float targetTilt, int steps, int delayPerStep)
////{
////    float deltaPan = (targetPan - panAngle) / steps;
////    float deltaTilt = (targetTilt - tiltAngle) / steps;

////    for (int i = 0; i < steps; i++) {
////        panAngle += deltaPan;
////        tiltAngle += deltaTilt;
////        Servo_SetPanAngle(panAngle);
////        Servo_SetTiltAngle(tiltAngle);

////        OLED_ShowNum(1, 6, panAngle, 3);
////        OLED_ShowNum(2, 7, tiltAngle, 3);
////        Delay_ms(delayPerStep);  // 每小步停顿
////		//Delay_ms(100);  // 每小步停顿
////    }
////}

////void DrawSquarePath(void)
////{
////    float half = SQUARE_SIZE / 2.0f;
////    float z = DISTANCE_TO_BOARD;

////    float square[6][3] = {
////        {-half,  half, z},  // A
////        { half,  half, z},  // B
////        { half, -half, z},  // C
////        {-half, -half, z},  // D
////        {-half,  half, z},  // 回到A
////				
////    };

////    // 分别处理每条边，确保最后一条边正确绘制
////    // A->B
////    float x1 = square[1][0], y1 = square[1][1], z1 = square[1][2];
////    float panRad1 = atan2f(x1, z1);
////    float tiltRad1 = atan2f(y1, sqrtf(x1 * x1 + z1 * z1));
////    float targetPan1 = PAN_CENTER + panRad1 * 180.0f / 3.1415926f;
////    float targetTilt1 = TILT_CENTER - tiltRad1 * 180.0f / 3.1415926f;
////    OLED_ShowString(3, 1, "Status: DRAW AB");
////    SmoothMoveTo(targetPan1, targetTilt1, 30, 20);
////    
////    // B->C
////    float x2 = square[2][0], y2 = square[2][1], z2 = square[2][2];
////    float panRad2 = atan2f(x2, z2);
////    float tiltRad2 = atan2f(y2, sqrtf(x2 * x2 + z2 * z2));
////    float targetPan2 = PAN_CENTER + panRad2 * 180.0f / 3.1415926f;
////    float targetTilt2 = TILT_CENTER - tiltRad2 * 180.0f / 3.1415926f;
////    OLED_ShowString(3, 1, "Status: DRAW BC");
////    SmoothMoveTo(targetPan2, targetTilt2, 30, 20);
////    
////    // C->D
////    float x3 = square[3][0], y3 = square[3][1], z3 = square[3][2];
////    float panRad3 = atan2f(x3, z3);
////    float tiltRad3 = atan2f(y3, sqrtf(x3 * x3 + z3 * z3));
////    float targetPan3 = PAN_CENTER + panRad3 * 180.0f / 3.1415926f;
////    float targetTilt3 = TILT_CENTER - tiltRad3 * 180.0f / 3.1415926f;
////    OLED_ShowString(3, 1, "Status: DRAW CD");
////    SmoothMoveTo(targetPan3, targetTilt3, 30, 20);
////    
////    // D->A (最后一条边，增加步数确保绘制完成)
////    float x4 = square[4][0], y4 = square[4][1], z4 = square[4][2];
////    float panRad4 = atan2f(x4, z4);
////    float tiltRad4 = atan2f(y4, sqrtf(x4 * x4 + z4 * z4));
////    float targetPan4 = PAN_CENTER + panRad4 * 180.0f / 3.1415926f;
////    float targetTilt4 = TILT_CENTER - tiltRad4 * 180.0f / 3.1415926f;
////    OLED_ShowString(3, 1, "Status: DRAW DA");
////    SmoothMoveTo(targetPan4, targetTilt4, 30, 20);  // 与其他边保持一致的参数
////    Delay_ms(200);  // 适度延迟确保稳定

////    // 额外确保回到A点
////    PointToAngleAndSet(square[0][0], square[0][1], square[0][2]);
////    Delay_ms(500);  // 等待稳定

////    OLED_ShowString(3, 1, "Status: DONE ");
////}


////// 云台归中
////void ResetToCenter(void)
////{
////    panAngle = PAN_CENTER;
////    tiltAngle = TILT_CENTER;

////    Servo_SetPanAngle(panAngle);
////    Servo_SetTiltAngle(tiltAngle);

////    OLED_ShowString(3, 1, "Status: RESET");
////    OLED_ShowNum(1, 6, panAngle, 3);
////    OLED_ShowNum(2, 7, tiltAngle, 3);
////    Delay_ms(500);
////}

////int main(void)
////{
////    OLED_Init();
////    Servo_Init();
////    Key_Init();

////    ResetToCenter(); // 开机默认归中

////    OLED_ShowString(1, 1, "Pan:");
////    OLED_ShowString(2, 1, "Tilt:");
////    OLED_ShowString(3, 1, "Status: IDLE");

////    while (1)
////    {
////        uint8_t key = Key_GetNum();
////        if (key == 3)
////        {
////            DrawSquarePath();  // 画正方形
////        }
////        else if (key == 1)
////        {
////            ResetToCenter();   // 回中
////        }

////        // 实时显示当前角度
////        OLED_ShowNum(1, 6, panAngle, 3);
////        OLED_ShowNum(2, 7, tiltAngle, 3);

////        Delay_ms(20);
////    }
////}



#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Servo.h"
#include "Key.h"
#include "UART.h"
#include "pid.h"
#include <string.h>
#include <stdlib.h>

// 图像中心
#define CAM_WIDTH 160
#define CAM_HEIGHT 120
#define CENTER_X (CAM_WIDTH / 2)
#define CENTER_Y (CAM_HEIGHT / 2)

// 云台角度约束
#define PAN_MIN 0.0f
#define PAN_MAX 180.0f
#define TILT_MIN 45.0f
#define TILT_MAX 135.0f
#define PAN_CENTER 90.0f
#define TILT_CENTER 90.0f

float panAngle = PAN_CENTER;
float tiltAngle = TILT_CENTER;

PID pid_pan;
PID pid_tilt;

extern uint8_t ParseComplete;
extern char RxBuffer[32];

int main(void)
{
    OLED_Init();
    Servo_Init();
    Key_Init();
    UART_Init();

    // 初始化 PID
    PID_Init(&pid_pan, 0.4f, 0.0f, 0.2f);
    PID_Init(&pid_tilt, 0.5f, 0.0f, 0.2f);
    pid_pan.setpoint = CENTER_X;
    pid_tilt.setpoint = CENTER_Y;

    // 初始归中
    Servo_SetPanAngle(panAngle);
    Servo_SetTiltAngle(tiltAngle);

    OLED_ShowString(1, 1, "Pan:");
    OLED_ShowString(2, 1, "Tilt:");
    OLED_ShowString(3, 1, "Status: ON ");
    OLED_ShowString(4, 1, "X:    Y:");

    while (1)
    {
        uint8_t key = Key_GetNum();

        if (key == 2) // Key2 复位
        {
            panAngle = PAN_CENTER;
            tiltAngle = TILT_CENTER;
            Servo_SetPanAngle(panAngle);
            Servo_SetTiltAngle(tiltAngle);
            OLED_ShowString(3, 1, "Status: RESET");
            Delay_ms(500);
        }

        if (ParseComplete)
        {
            ParseComplete = 0;

            int blob_x = -1, blob_y = -1;
            char *x_str = strstr(RxBuffer, "X");
            char *y_str = strstr(RxBuffer, "Y");

            if (x_str && y_str && strlen(x_str) >= 4 && strlen(y_str) >= 4)
            {
                blob_x = atoi(x_str + 1);
                blob_y = atoi(y_str + 1);
            }

            OLED_ShowNum(4, 3, blob_x, 3);
            OLED_ShowNum(4, 10, blob_y, 3);

            if (blob_x >= 0 && blob_y >= 0)
            {
                OLED_ShowString(3, 1, "Status: TRACK");

                float control_pan = PID_Compute(&pid_pan, blob_x);
                float control_tilt = PID_Compute(&pid_tilt, blob_y);

                panAngle += control_pan;
                tiltAngle += control_tilt;

                // 限制角度
                if (panAngle < PAN_MIN) panAngle = PAN_MIN;
                if (panAngle > PAN_MAX) panAngle = PAN_MAX;
                if (tiltAngle < TILT_MIN) tiltAngle = TILT_MIN;
                if (tiltAngle > TILT_MAX) tiltAngle = TILT_MAX;

                Servo_SetPanAngle(panAngle);
                Servo_SetTiltAngle(tiltAngle);
            }
        }

        OLED_ShowNum(1, 6, panAngle, 3);
        OLED_ShowNum(2, 7, tiltAngle, 3);
        Delay_ms(20);
    }
}




