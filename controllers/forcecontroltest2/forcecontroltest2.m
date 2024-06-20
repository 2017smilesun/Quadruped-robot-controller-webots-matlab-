TIME_STEP = 64;
%%
%初始化关节句柄
     fr_joint_gamma=wb_robot_get_device('FR_hip_joint');
     fr_joint_alfa=wb_robot_get_device('FR_thigh_joint');
     fr_joint_beta=wb_robot_get_device('FR_calf_joint');
     fl_joint_gamma=wb_robot_get_device('FL_hip_joint');
     fl_joint_alfa=wb_robot_get_device('FL_thigh_joint');
     fl_joint_beta=wb_robot_get_device('FL_calf_joint');
     rl_joint_gamma=wb_robot_get_device('RL_hip_joint');
     rl_joint_alfa=wb_robot_get_device('RL_thigh_joint');
     rl_joint_beta=wb_robot_get_device('RL_calf_joint');
     rr_joint_gamma=wb_robot_get_device('RR_hip_joint');
     rr_joint_alfa=wb_robot_get_device('RR_thigh_joint');
     rr_joint_beta=wb_robot_get_device('RR_calf_joint');
     joint_device = [fr_joint_gamma;fr_joint_alfa;fr_joint_beta;fl_joint_gamma;fl_joint_alfa;fl_joint_beta;
                     rr_joint_gamma;rr_joint_alfa;rr_joint_beta;rl_joint_gamma;rl_joint_alfa;rl_joint_beta];
%%     
%初始化关节角度传感器
     FR_hip_joint_sensor=wb_robot_get_device('FR_hip_joint_sensor');
     FR_thigh_joint_sensor=wb_robot_get_device('FR_thigh_joint_sensor');
     FR_calf_joint_sensor=wb_robot_get_device('FR_calf_joint_sensor');
     FL_hip_joint_sensor=wb_robot_get_device('FL_hip_joint_sensor');
     FL_thigh_joint_sensor=wb_robot_get_device('FL_thigh_joint_sensor');
     FL_calf_joint_sensor=wb_robot_get_device('FL_calf_joint_sensor');
     RL_hip_joint_sensor=wb_robot_get_device('RL_hip_joint_sensor');
     RL_thigh_joint_sensor=wb_robot_get_device('RL_thigh_joint_sensor');
     RL_calf_joint_sensor=wb_robot_get_device('RL_calf_joint_sensor');
     RR_hip_joint_sensor=wb_robot_get_device('RR_hip_joint_sensor');
     RR_thigh_joint_sensor=wb_robot_get_device('RR_thigh_joint_sensor');
     RR_calf_joint_sensor=wb_robot_get_device('RR_calf_joint_sensor');
     joint_sensor_device = [FR_hip_joint_sensor;FR_thigh_joint_sensor;FR_calf_joint_sensor;FL_hip_joint_sensor;FL_thigh_joint_sensor;
                            FL_calf_joint_sensor;RR_hip_joint_sensor;RR_thigh_joint_sensor; RR_calf_joint_sensor;RL_hip_joint_sensor;
                            RL_thigh_joint_sensor;RL_calf_joint_sensor];
%%
%关节角度传感器使能   
   wb_position_sensor_enable(FR_hip_joint_sensor,TIME_STEP);
   wb_position_sensor_enable(FR_thigh_joint_sensor,TIME_STEP);
   wb_position_sensor_enable(FR_calf_joint_sensor,TIME_STEP);
   wb_position_sensor_enable(FL_hip_joint_sensor,TIME_STEP);
   wb_position_sensor_enable(FL_thigh_joint_sensor,TIME_STEP);
   wb_position_sensor_enable(FL_calf_joint_sensor,TIME_STEP);
   wb_position_sensor_enable(RL_hip_joint_sensor,TIME_STEP);
   wb_position_sensor_enable(RL_thigh_joint_sensor,TIME_STEP);
   wb_position_sensor_enable(RL_calf_joint_sensor,TIME_STEP);
   wb_position_sensor_enable(RR_hip_joint_sensor,TIME_STEP);
   wb_position_sensor_enable(RR_thigh_joint_sensor,TIME_STEP);
   wb_position_sensor_enable(RR_calf_joint_sensor,TIME_STEP);

%%
%%IMU及陀螺仪传感器初始化及使能   
   IMU = wb_robot_get_device('trunk_imu inertial');
   wb_inertial_unit_enable(IMU,TIME_STEP);
   
   Gyro = wb_robot_get_device('trunk_imu gyro');
   wb_gyro_enable(Gyro,TIME_STEP);
%%
%supervisor初始化
   trunk_body = wb_supervisor_node_get_from_def('trunk');
   
   FR_foot = wb_supervisor_node_get_from_def('FR_foot');
   FL_foot = wb_supervisor_node_get_from_def('FL_foot');
   RR_foot = wb_supervisor_node_get_from_def('RR_foot');
   RL_foot = wb_supervisor_node_get_from_def('RL_foot');
   foot_DEF = [FR_foot;FL_foot;RR_foot;RL_foot];
%%
%初始化参数
   state = LowLevelState(joint_device,joint_sensor_device,IMU,Gyro,trunk_body,foot_DEF);
   Robot = QuadrupedRobot(state);
   BalaCtrl = BalanceCtrl( );

 Kpp = [300, 0, 0;
        0, 300, 0;
        0, 0, 300];
 Kdp = [30, 0, 0;
        0, 30, 0;
        0, 0, 30];
 Kpw = 400;
 Kdw = [15, 0, 0;
        0, 15, 0;
        0, 0, 15];
 
 x = 0;
 y = 0;
 z = 0;
 yawuser = 0;
 
 contact = [1;1;1;1];%%使机器人为四足站立
 aaa = 1;
    wb_keyboard_enable(TIME_STEP);
 bnb = 1;
 while wb_robot_step(TIME_STEP) ~= -1
    %%run
    if aaa == 1
       pcdInit_ = - Robot.getFootPosition(1, 'BODY');
       pcd_ = pcdInit_;         %%_pcd是机器人重心在世界坐标系下的目标位置
       RdInit_ = state.getRotMat();   %%_Rd是机器人在世界坐标系下的目标姿态
       
       aaa = aaa+1;
    end
    
    key = wb_keyboard_get_key();
    
    switch key
       case 85
          x = x + 0.0005
       case 73
          y = y + 0.0005
       case 79
          z = z + 0.0005
       case 80
          yawuser = yawuser + pi/80
       case 72
          x = x - 0.0005
       case 74
          y = y - 0.0005
       case 75
          z = z - 0.0005
       case 76
          yawuser = yawuser - pi/80
    end  
    
    pcd_user = [x;y;z;yawuser];

    
    pcd_(1) = pcdInit_(1) + pcd_user(1);
    pcd_(2) = pcdInit_(2) + pcd_user(2);
    pcd_(3) = pcdInit_(3) + pcd_user(3);%%获取机器人重心在世界坐标系下的期望位置
    
    yaw_ = pcd_user(4);
    Rd_ = rpyToRotMat(0, 0, yaw_) * RdInit_;  %%计算时角度要用弧度
    
    posBody_ = -Robot.getFootPosition(1, 'BODY');
    velBody_ = state.getBodyVel();
 
    B2G_RotMat = state.getRotMat();
    G2B_RotMat = B2G_RotMat';
 
    tau = calTau(Kpp,pcd_,posBody_,Kdp,velBody_,Kpw,Rd_,G2B_RotMat,Kdw,B2G_RotMat,state,Robot,BalaCtrl);
    state.setTau(tau);
 end
 %%函数
 function R_sb = rpyToRotMat(roll, pitch, yaw)
     R_sb = [cos(yaw)*cos(pitch) cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll) cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll);
             sin(yaw)*cos(pitch) sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll) sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);
             -sin(pitch)         cos(pitch)*sin(roll)                             cos(pitch)*cos(roll)                            ];
 end
 
 function tau_ = calTau(Kpp,pcd_,posBody_,Kdp,velBody_,Kpw,Rd_,G2B_RotMat,Kdw,B2G_RotMat,state,Robot,BalaCtrl)
     contact = [1;1;1;1];%%使机器人为四足站立
     ddPcd_ = Kpp*(pcd_ - posBody_) + Kdp * ([0;0;0] - velBody_); %计算期望机身加速度
     
     Gyro_cur = state.getGyroGlobal();
     dWbd_  = Kpw * rotMatToExp(Rd_*G2B_RotMat) + Kdw * ([0;0;0] - Gyro_cur);%计算期望机身角加速度
     
     posFeet2BGlobal_ = getPosFeet2BGlobal(state,Robot);%获取当前机身几何中心到各个足端的向量在世界坐标系下的坐标
     
     forceFeetGlobal_ = -BalaCtrl.calF(ddPcd_, dWbd_, B2G_RotMat, posFeet2BGlobal_, contact);%调用BlanceCtrl类的calF函数计算得到世界坐标系
                                                                                             %下足端对外界的作用力，用负号是因为是反作用力
     forceFeetBody_ = G2B_RotMat * forceFeetGlobal_; %通过坐标变换计算得到机身坐标系下足端对外界的作用力
     
     q_ = vec34ToVec12(state.getQ());
     tau_ = Robot.getTau(q_, forceFeetBody_);
     fprintf('forceFeetBody_的大小: %f %f %f\n%f %f %f\n%f %f %f\n%f %f %f\n',forceFeetBody_(1,1),forceFeetBody_(2,1),forceFeetBody_(3,1),forceFeetBody_(1,2),forceFeetBody_(2,2),forceFeetBody_(3,2),forceFeetBody_(1,3),forceFeetBody_(2,3),forceFeetBody_(3,3),forceFeetBody_(1,4),forceFeetBody_(2,4),forceFeetBody_(3,4)); 
 end
 
 function  qq = vec34ToVec12(Q) %3*4矩阵转为12维向量
     qq = zeros(12,1);
     qq(1:3) = Q(1:3,1);
     qq(4:6) = Q(1:3,2);
     qq(7:9) = Q(1:3,3);
     qq(10:12) = Q(1:3,4);
 end
 
 function feet2BPos = getPosFeet2BGlobal(state,Robot)%获取当前机身几何中心到各个足端的向量在世界坐标系下的坐标
     feet2BPos = zeros(3,4);
     for i = 1:4
         feet2BPos(:,i) = getFootPos(i,Robot,state) - getBodyPos(Robot);
     end
 end
 
 function FootPos = getFootPos(i,Robot,state)%获取世界坐标系下的足端坐标
     FootPos = getBodyPos(Robot) + state.getRotMat() * Robot.getFootPosition(i,'BODY');
 end
 
 function posBody_ = getBodyPos(Robot)%获取世界坐标系下的质心坐标
     posBody_ = -Robot.getFootPosition(1, 'BODY');
 end
 
  function exp = rotMatToExp(rm)
     cosValue = sum(diag(rm))/2-1/2;
     if cosValue > 1.0
         cosValue = 1.0;
     elseif cosValue < -1.0
         cosValue = -1.0;
     end
     
     angle = acos(cosValue);
     
     exp = zeros(3,1);
     if abs(angle) < 1e-5
         exp = [0;0;0];
     elseif abs(angle - pi) < 1e-5
         exp = angle * [rm(1,1)+1;rm(1,2);rm(1,3)]/sqrt(2*(1+rm(1,1)));
     else
         exp = angle/(2.0*sin(angle))*[rm(3,2)-rm(2,3);rm(1,3)-rm(3,1);rm(2,1)-rm(1,2)];
     end
 end