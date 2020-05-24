#pragma once

#define CtrlRateHz 400

//控制系统互斥锁
bool LockCtrl( double TIMEOUT );
void UnlockCtrl();

//上次控制时间
extern TIME last_XYCtrlTime;
extern TIME last_ZCtrlTime;

//MSafe任务句柄
extern TaskHandle_t MSafeTaskHandle;
//强制Safe控制
extern bool ForceMSafeCtrl;

void init_ControlSystem();