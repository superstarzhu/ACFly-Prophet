#pragma once

/*
  获取电池电流大小,单位：A
*/
float Get_MainBaterry_Current();

/*
  获取电池电压,单位；V
*/
float Get_MainBaterry_Voltage();

/*
  获取VDDA基准电压,单位；V
*/
float Get_VDDA_Voltage();

/*
  获取温度，单位：℃
*/
float Get_Temperature();

void init_drv_ADC(void);