# ACFly-Prophet
    快速二次开发单参数多旋翼飞行控制器       
    Single parameter multi rotor flight controller for fast secondary development environment
    QQ群：180319060
    Shop：https://item.taobao.com/item.htm?spm=a230r.1.14.1.7b6f12cf2dA8yr&id=591615647197&ns=1&abbucket=11#detail

## 单参数调节实现稳定飞行 Single parameter adjustment for stable flight
    只需调节b参数实现稳定飞行，飞机动力越强劲b参数越大      
    在250-2000轴距，4-8轴，500g-40kg多旋翼飞行器上测试过此调参方法的稳定性        
    测试得到的机型参数：    
    Only need to adjust the b parameter to achieve stable flight        
    The stronger the aircraft power, the greater the b parameter   
    The stability of this control method has been tested on 
        250-2000 wheelbase, 4-8 axis, 500g-40kg multi-rotor aircraft
    the model parameters obtained by the test:      
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
    实时检测罗盘异常（磁干扰等），罗盘异常或无罗盘时采用位置传感器修正航向
    Real-time detection of compass anomalies (magnetic interference, etc.) 
    When the compass is abnormal or no compass, the position sensor is used to correct the course

## 快速飞行控制及传感器二次开发 Fast secondary development of flight control and sensor
### 飞行控制二次开发
        Modes 飞行控制模式 Flight control modes       
        在模式中调用控制接口即可完成飞行控制二次开发
        Call the control system interface in your modes to complete the secondary development of flight control
            ControlSystem.hpp 控制系统接口 Control system interface
                提供直线飞行、控制速度等函数接口
                Provide straight-line flight, speed control and other function interfaces
            NavCmdProcess.hpp 控制流程接口 Control processes interface
                提供航线飞行等控制流程（与Mavlink兼容）
                Provide route and other control processes (compatible with Mavlink)
            Receiver.hpp
                提供遥控信号获取接口
                Provide remote control signal acquisition interface
            MeasurementSystem.hpp
                提供解算（姿态位置速度等）信息获取接口
                Provide solution (attitude, position, speed, etc.) information acquisition interface
            MSafe安全模式可以在用户二次开发代码有问题时自动接管，保障二次开发飞行安全
            MSafe safety mode can automatically take over when the user has problems with the secondary development code
            This can ensure the safety of secondary development flight
