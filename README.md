# ACFly-Prophet
    单参数飞行器飞行控制器       
    Single parameter multi rotor flight controller for fast secondary development environment

## 单参数调节实现稳定飞行 Single parameter adjustment for stable flight
    只需调节b参数实现稳定飞行，飞机动力越强劲b参数越大，测试得到的机型参数：     
    Only need to adjust the b parameter to achieve stable flight        
    The stronger the aircraft power, the greater the b parameter, the model parameters obtained by the test:      
        F450+U2216 800kv+1147+4s b=4.5      
        F450+dji2312 940v+9450+4s b=7.5     
        dh600+4114 400kv+1555+6s b=5.5      
        Tarot X6+4108 320kv+1855+6s b=1.2     
        f330+x2212 1400kv+8038+3s b=7.5     
   
## 传感器异常处理智能切换 Sensor abnormal handling & intelligent switching
    实时智能检测传感器异常，如气压受气流干扰波动、超声波跳变、GPS被干扰器干扰等     
    自动调节权重并切换使用最优的传感器进行融合修正，保证数据稳定性     
    Real-time intelligent detection of sensor abnormalities     
    Such as air pressure fluctuations caused by airflow interference, ultrasonic jumps, GPS interference by jammers, etc.       
    Automatically adjusts the weight and switches to use the best sensor for fusion to ensure data stability.

## 磁罗盘异常智能检测修正 Intelligent detection and correction of compass anomaly
    实时
