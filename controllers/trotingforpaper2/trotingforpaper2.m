TIME_STEP = 16;
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
   IMU = wb_robot_get_device('trunk_imu inertial');%获取IMU的数据
   wb_inertial_unit_enable(IMU,TIME_STEP);
   
   Gyro = wb_robot_get_device('trunk_imu gyro');%获取陀螺仪的数据
   wb_gyro_enable(Gyro,TIME_STEP);
%%
%触觉传感器初始化及使能
   touch_sensor1 = wb_robot_get_device('touch_sensor1');%获取触觉传感器的数据
   wb_touch_sensor_enable(touch_sensor1,TIME_STEP);
   touch_sensor2 = wb_robot_get_device('touch_sensor2');%获取触觉传感器的数据
   wb_touch_sensor_enable(touch_sensor2,TIME_STEP);
   touch_sensor3 = wb_robot_get_device('touch_sensor3');%获取触觉传感器的数据
   wb_touch_sensor_enable(touch_sensor3,TIME_STEP);
   touch_sensor4 = wb_robot_get_device('touch_sensor4');%获取触觉传感器的数据
   wb_touch_sensor_enable(touch_sensor4,TIME_STEP);
   touch_sensor_DEF = [touch_sensor1;touch_sensor2;touch_sensor3;touch_sensor4];
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
   state = LowLevelState(joint_device,joint_sensor_device,IMU,Gyro,trunk_body,foot_DEF);%初始化LowLevelState类
   Robot = QuadrupedRobot(state);%初始化QuadrupedRobot类
   BalaCtrl = BalanceCtrl( );
   
   %wb_keyboard_enable(TIME_STEP);
   
   gaitHeight = 0.07;

   Kpp = [650, 0,  0;
          0,  650, 0;
          0,  0,  650];%机身平衡器位置
   Kdp = [80, 0,  0;
          0,  80, 0;
          0,  0,  80];%机身平衡器速度
   Kpw = [200 0 0;
          0 200 0;
          0 0 200];%机身平衡器角度
   Kdw = [70 0  0;
          0  70 0;
          0  0  70];%机身平衡器角速度
   KpSwing = [150, 0,   0;
              0,   250, 0;
              0,   0,   80];%摆动腿跟踪位置
   KdSwing = [12 0  0;
              0  25 0;
              0  0  15];%摆动腿跟踪速度
   kyaw = 0.01;
   kx = 0.005;
   ky = 0.005;
   contact = [1;1;1;1];%初始接触标识
   phase = [0.5;0.5;0.5;0.5];%初始相位
   
   kp_roll = 200;%横滚角pd控制的p
   kd_roll = 20;%横滚角pd控制的d
   dYawCmdPast = 0;
   dt = 0.016;
   
   startT = wb_robot_get_time();%获取仿真时间记为仿真开始时间
   period = 0.45;%步态周期P
   bias = [0;0.5;0.5;0];%步态的偏移系数，四条腿偏移时间b与步态周期P的比值
   stRatio = 0.5;%触地系数r
   aaa = 1;
   posFeetGlobalGoal = zeros(3,4);
   Tswing = period * stRatio;%计算摆动相的周期
   Tstance = period * (1 - stRatio);%计算触地相的周期
   feetRadius = zeros(4,1);
   feetInitAngle = zeros(4,1);
   frequence = 1;
   flag = 1;
   flag_num = 1
   while wb_robot_step(TIME_STEP) ~= -1
       if aaa == 1 %初始化状态
           pcd = state.getBodyPos(); %获取机身在世界坐标系下的质心位置为初始目标位置，_pcd是机身在世界坐标系下的目标位置
           FeetPosIdeal = -Robot.getFeetPosIdeal();%获取足端中性落脚点在机身坐标系{b}下的坐标的负数
           %pcd(3) = FeetPosIdeal(3,1); %将足端中性落脚点在机身坐标系{b}下的坐标赋给质心的Z坐标
           pcd(3) = 0.292; %将足端中性落脚点在机身坐标系{b}下的坐标赋给质心的Z坐标
           vCmdBody = zeros(3,1);%将机身在机身坐标系下的目标速度设置为0
           yawCmd = state.getYaw();%获取机身在世界坐标系下的偏航角为初始目标偏航角
           Rd = state.getRotMat();%获取旋转矩阵
           wCmdGlobal = zeros(3,1);%初始化机身在世界坐标系下的目标旋转角速度向量为0
           velFeetGlobalGoal = zeros(3,4);
           foot1 = state.getFootPos(1);
           aaa = aaa + 1;
           filename = 'C:\Users\Desktop\excel file\data_1.xlsx';%数据保存地址
           sheetname = 'Gravel';
       end
       
       posBody = state.getBodyPos();%获得机身在世界坐标系下的质心位置
       velBody = state.getBodyVel();%获得机身在世界坐标系下的质心速度
       posFeet2BGlobal = state.getPosFeet2BGlobal();%获取足端在世界坐标系下相对于机身中心的位置向量
       posFeetGlobal = state.getFeetPos();%获取足端在世界坐标系下的位置坐标
       velFeetGlobal = state.getFeetVel();%获取足端在世界坐标系下的速度
       B2G_RotMat = state.getRotMat();%获得机身坐标系转为世界坐标系的旋转矩阵
       G2B_RotMat = B2G_RotMat';%获得世界坐标系转为机身坐标系的旋转矩阵
       yaw = state.getYaw();%获取机身在世界坐标系下的偏航角
       dYaw = state.getDYaw();%获取机器人绕y轴的角速度,即偏航角速度
       passT = wb_robot_get_time() - startT;%获得程序经过的时间
       
       
       
       userValuely = 0;
       userValuelx = 1.5;
       userValuerx = 0;
       %%↓是宇树的getUserCmd函数,就是将用户输入的值归一化后赋给机器人
       %移动         
       vCmdBody(1) = invNormalize(userValuelx, -0.4, 0.4); %invNormalize归一化函数,vCmdBody是机身在机身坐标系下的目标速度
       vCmdBody(2) = -invNormalize(userValuely, -0.3, 0.3);
       vCmdBody(3) = 0;
        
       %转动
       dYawCmd = -invNormalize(userValuerx, -0.5, 0.5);  %dYawCmd是机身在世界坐标系下的目标偏航角速度
       dYawCmd = 0.9 * dYawCmdPast + (1-0.9) * dYawCmd;
       dYawCmdPast = dYawCmd;
       %%↑是宇树的getUserCmd函数,就是将用户输入的值归一化后赋给控制器
       %%↓是宇树的calcCmd函数,计算机身的目标姿态Rd和目标角速度wCmdGlobal
       %移动
       vCmdGlobal = B2G_RotMat * vCmdBody; %获取机身在世界坐标系下的目标速度
       
       S_vCmd1 = saturation([velBody(1)-0.2,velBody(1)+0.2]);%saturation是饱和运算，主要设置上下限
       vCmdGlobal(1) = evaluate(S_vCmd1,vCmdGlobal(1));%实现饱和运算，高于上下限就变成上下限
       S_vCmd2 = saturation([velBody(2)-0.2,velBody(2)+0.2]);%saturation是饱和运算，主要设置上下限
       vCmdGlobal(2) = evaluate(S_vCmd2,vCmdGlobal(2));
        
       S_pcd1 = saturation([posBody(1) - 0.05, posBody(1) + 0.05]);
       pcd(1) = evaluate(S_pcd1, pcd(1) + vCmdGlobal(1) * dt);  
       S_pcd2 = saturation([posBody(2) - 0.05, posBody(2) + 0.05]);
       pcd(2) = evaluate(S_pcd2, pcd(2) + vCmdGlobal(2) * dt);  
      
       vCmdGlobal(3) = 0;

       %转动
       yawCmd = yawCmd + dYawCmd * dt;%计算转动的目标角度     
       Rd = rotz(yawCmd);%根据目标角度获取旋转矩阵
       wCmdGlobal(3) = dYawCmd;
       %%↑是宇树的calcCmd函数
       
       %↓以下是计算步态的GaitGenerator函数
       vxyGoal = vCmdGlobal;%获取机身在世界坐标系下的目标速度
       dYawGoal = wCmdGlobal(3);%获取机身在世界坐标系下的目标绕z轴角速度
       feetPosBody = Robot.getFeetPosIdeal();%获取四足机器人足端中性落脚点在机身坐标系的位置
       for a = 1:4
           feetRadius(a) = sqrt(feetPosBody(1, a)^2 + feetPosBody(2, a)^2);
           feetInitAngle(a) = atan2(feetPosBody(2, a), feetPosBody(1, a));
       end
       feetPos = zeros(3,4);
       feetVel = zeros(3,4);
       startP = state.getFeetPos();
       
       
       for a = 1:4
            normalT(a) = rem(passT + period - period * bias(a), period) / period;     %mod()函数用来求余数 
            if normalT(a) < stRatio%小于触地系数，也就是正在触地
               contact(a) = 1;
               phase(a) = normalT(a) / stRatio;
            elseif normalT(a) >= stRatio
               contact(a) = 0;
               phase(a) = (normalT(a) - stRatio) / (1 - stRatio);
            end
       end
       
       for a = 1:4
           if contact(a) == 1
               if phase(a) < 0.5
                   startP(:,a) = state.getFootPos(a);
               end
               feetPos(:,a) = startP(:,a);
               feetVel(:,a) = zeros(3,1);
           else
               endP(:,a) = calFootPos(a, vxyGoal, dYawGoal, phase(a),state,Tswing,Tstance,kx,ky,feetRadius,feetInitAngle,kyaw);%计算落脚点坐标
               feetPos(:,a) = getFootPos(a,startP,endP,gaitHeight, phase);
               feetVel(:,a) = getFootVel(a,startP,endP,phase,gaitHeight,Tswing);
           end
       end
       pastP = feetPos;
       posFeetGlobalGoal = feetPos;
       velFeetGlobalGoal = feetVel;
       phasePast = phase;
       
       %fprintf('contact的大小: %f %f %f\n',contact(1),contact(2),contact(3)); 
       %fprintf('pcd的大小: %f %f %f\n',pcd(1),pcd(2),pcd(3)); 
       Tau = calcTau(Kpp, pcd, posBody, Kdp, velBody, Kpw, Rd, G2B_RotMat, Kdw, state, B2G_RotMat,contact, KpSwing, posFeetGlobalGoal, KdSwing, velFeetGlobalGoal, Robot,BalaCtrl);
       
       R_angle = wb_inertial_unit_get_roll_pitch_yaw(IMU);
       Roll_angle = R_angle(1);
       Gyro_vel = wb_gyro_get_values(Gyro);
       Roll_angle_d = Gyro_vel(1);
       M_beta = kp_roll * ( 0 - Roll_angle) + kd_roll * ( 0 - Roll_angle_d);
       M_beta = -M_beta;
       
       for i = 1:4
           if contact(i) == 1
               Tau(3*(i-1)+1) = Tau(3*(i-1)+1) + M_beta/2;
           end
       end
       
       state.setTau(Tau);
       
       plot_state = 2;%这里改为1就可对仿真数据实时显示，但是会导致仿真变慢
       if plot_state == 1
           t = wb_robot_get_time();
           gyro = wb_inertial_unit_get_roll_pitch_yaw(IMU);
           x123 = 0;
           x234 = 0.128;
           subplot(2,2,1);
           plot(t,gyro(3),'-r.','MarkerSize',10);
           hold on;  %保持让上一个点显示
           %pause(0.001);   %暂停，就可以看到点的变化走向
           subplot(2,2,1);
           plot(t,x123,'-b.','MarkerSize',10);
           hold on;  %保持让上一个点显示
           subplot(2,2,2);
           plot(t,posBody(1),'-r.','MarkerSize',10);
           hold on;  %保持让上一个点显示
           subplot(2,2,2);
           plot(t,pcd(1),'-b.','MarkerSize',10);
           hold on;  %保持让上一个点显示
           subplot(2,2,3);
           plot(t,posBody(2),'-r.','MarkerSize',10);
           hold on;  %保持让上一个点显示
           subplot(2,2,3);
           plot(t,pcd(2),'-b.','MarkerSize',10);
           hold on;  %保持让上一个点显示
           subplot(2,2,4);
           plot(t,posBody(3),'-r.','MarkerSize',10);
           hold on;  %保持让上一个点显示
           plot(t,pcd(3),'-b.','MarkerSize',10);
           hold on;  %保持让上一个点显示
           drawnow;
       end
       current_foot = state.getFootPos(1);
       t = wb_robot_get_time();

       %获取四足机器人的各项数据，并保存到excel表中
       data(flag,5) = R_angle(1);
       data(flag,6) = R_angle(2);
       data(flag,7) = R_angle(3);
       pcd_current = state.getBodyPos();
       data(flag,2) = pcd_current(1);
       data(flag,3) = pcd_current(2);
       data(flag,4) = pcd_current(3);
       data(flag,8) = current_foot(3);
       data(flag,9) = pastP(3,1);
       data(flag,1) = t;
       flag = flag + 1;
       if t > 20 && flag_num == 1 %仿真时间大于20s时保存一次
       
           xlswrite(filename,data,sheetname);
           flag_num = 2;
       end
       
   end

    function tau = calcTau(Kpp, pcd, posBody, Kdp, velBody, Kpw, Rd, G2B_RotMat, Kdw, state, B2G_RotMat,contact, KpSwing, posFeetGlobalGoal, KdSwing, velFeetGlobalGoal, Robot,BalaCtrl)
        ddPcd = Kpp*(pcd - posBody) + Kdp * ([0;0;0] - velBody);
        dWbd  = Kpw*rotMatToExp(Rd*G2B_RotMat) + Kdw * ([0;0;0] - state.getGyroGlobal());
        posFeet2BGlobal = state.getPosFeet2BGlobal();
        forceFeetGlobal = - BalaCtrl.calF(ddPcd, dWbd, B2G_RotMat, posFeet2BGlobal, contact);
        posFeetGlobal = state.getFeetPos();
        velFeetGlobal = state.getFeetVel();
        for i = 1:4
            if contact(i) == 0
                forceFeetGlobal(:,i) = KpSwing*(posFeetGlobalGoal(:,i) - posFeetGlobal(:,i)) + KdSwing*(velFeetGlobalGoal(:,i)-velFeetGlobal(:,i));
            end
        end
        forceFeetBody = G2B_RotMat * forceFeetGlobal;
        q = vec34ToVec12(state.getQ());
        tau = Robot.getTau(q, forceFeetBody);
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
   
   function  qq = vec34ToVec12(Q)
     qq = zeros(12,1);
     qq(1:3) = Q(1:3,1);
     qq(4:6) = Q(1:3,2);
     qq(7:9) = Q(1:3,3);
     qq(10:12) = Q(1:3,4);
   end
   
   function vCmdBody_xy = invNormalize(value, min, max)
       minLim = -1;
       maxLim =  1;
       vCmdBody_xy = (value-minLim)*(max-min)/(maxLim-minLim) + min;
   end
   
   function footPos = calFootPos(legID, vxyGoalGlobal, dYawGoal, phase,state,Tswing,Tstance,kx,ky,feetRadius,feetInitAngle,kyaw)
       bodyVelGlobal = state.getBodyVel(); %获取机器人当前世界坐标系下的线速度
       bodyWGlobal = state.getGyroGlobal();  %获取机器人当前世界坐标系下的角速度
       nextStep = zeros(3,1);%
       
       nextStep(1) = bodyVelGlobal(1) * (1 - phase) * Tswing + bodyVelGlobal(1) * Tstance/2 + kx * (bodyVelGlobal(1) - vxyGoalGlobal(1));
       nextStep(2) = bodyVelGlobal(2) * (1 - phase) * Tswing + bodyVelGlobal(2) * Tstance/2 + ky * (bodyVelGlobal(2) - vxyGoalGlobal(2));
       nextStep(3) = 0;
         
       yaw = state.getYaw();
       dYaw = state.getDYaw();
       nextYaw = dYaw * (1 - phase) * Tswing + dYaw * Tstance/2 + kyaw * (dYawGoal - dYaw);
           
       nextStep(1) = nextStep(1) + feetRadius(legID) * cos(yaw + feetInitAngle(legID) + nextYaw);
       nextStep(2) = nextStep(2) + feetRadius(legID) * sin(yaw + feetInitAngle(legID) + nextYaw);
       footPos = state.getBodyPos() + nextStep; 
       footPos(3) = 0;
   end
   
   function footPos = getFootPos(i,startP,endP,gaitHeight, phase)  %获取对应足端在当前时刻的目标位置
       footPos = zeros(3,1);
            
       startP_icol = startP(:,i);
       endp_icol = endP(:,i);
       footPos(1) = cycloidXYPosition(startP_icol(1), endp_icol(1), phase(i));
       footPos(2) = cycloidXYPosition(startP_icol(2), endp_icol(2), phase(i));
       footPos(3) =  cycloidZPosition(startP_icol(3), gaitHeight, phase(i));
   end
   
   function footVel = getFootVel(i,startP,endP,phase,gaitHeight,Tswing)  %获取对应足端在当前时刻的目标速度
       footVel = zeros(3,1);
            
       startP_icol = startP(:,i);
       endp_icol = endP(:,i);
       footVel(1) = cycloidXYVelocity(startP_icol(1), endp_icol(1), phase(i),Tswing);
       footVel(2) = cycloidXYVelocity(startP_icol(2),endp_icol(2), phase(i),Tswing);
       footVel(3) =  cycloidZVelocity(gaitHeight, phase(i),Tswing);
   end
   
   function XYPosition = cycloidXYPosition(startd, endd, phaseXY)  %参照式9.14计算摆线轨迹在x,y轴的位置坐标,cycloid摆线
       phasePI = 2 * pi * phaseXY;
       XYPosition = (endd - startd) * (phasePI - sin(phasePI)) / (2*pi) +startd;
   end
        
   function XYVelocity = cycloidXYVelocity(startd, endd, phaseXY,Tswing)  %参照式9.15计算摆线轨迹在x,y轴的速度分量,cycloid摆线
       phasePI = 2 * pi * phaseXY;
       XYVelocity = (endd - startd) * (1 - cos(phasePI)) / Tswing();
   end
        
   function ZPosition = cycloidZPosition(startd, h, phaseZ)  %参照式9.14计算摆线轨迹在z轴的位置坐标,cycloid摆线
       phasePI = 2 * pi * phaseZ;
       ZPosition = h*(1 - cos(phasePI)) / 2 +startd;
   end
   
   function ZVelocity = cycloidZVelocity(h, phaseZ,Tswing)  %参照式9.15计算摆线轨迹在z轴的速度分量,cycloid摆线
       phasePI = 2 * pi * phaseZ;
       ZVelocity = h * pi * sin(phasePI) / Tswing();
   end
        % 本项目是基于宇树开源项目https://github.com/unitreerobotics/unitree_guide进行的，只实现了皮毛，还有很多很多的不足；
        % 
        % 感谢宇树公司出版的《四足机器人控制算法——建模、控制与实践》这本书，让我可以学会四足机器人的控制，并使我能够理解工程代码。
        % 
        %虽然说是“开源”，但更像是分享，因为自己只做了点皮毛，只是希望能帮助刚入门的同学。
        % 
        % 本人属于能懒就懒，代码写的也不规范，请大家轻喷
        %
        % 最后还是建议大家，理解后直接C+ROS入手，也看一下宇树的这本书，让人受益匪浅。

        %来源：B站——起名字不是问题