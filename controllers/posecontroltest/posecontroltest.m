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
%初始化参数
   state = LowLevelState(joint_device,joint_sensor_device,IMU,Gyro);
   Robot = QuadrupedRobot(state);
   
   
   vecOP = zeros(3,4);
   aaa = 1;
   
   bnb = 1;   
%%
while wb_robot_step(TIME_STEP) ~= -1
   
   if aaa == 1
       initVecOX_ = Robot.getFootPosition(1, 'BODY');%编号为1的足端在机身坐标系中的初始坐标，但该状态只在初始状态下成立，因为初始状态时
                                                 %世界坐标系与机身坐标系平行，此时编号为1的足端的坐标作为世界坐标系的原点
       fprintf('Current RF_foot position: %f  %f  %f\n',initVecOX_(1),initVecOX_(2),initVecOX_(3));
       initVecXP_ = Robot.getVecXP();  %根据initVecOX计算其他足端在世界坐标系中的初始坐标
       fprintf('Current ALL_motor angle: %f  %f  %f  %f  %f  %f  %f  %f  %f  %f  %f  %f\n',initVecXP_(1),initVecXP_(2),initVecXP_(3),initVecXP_(4),initVecXP_(5),initVecXP_(6),initVecXP_(7),initVecXP_(8),initVecXP_(9),initVecXP_(10),initVecXP_(11),initVecXP_(12));
       aaa = aaa+1;
   end
   

   
   if bnb <= 100
      userValue = [0;0;0;-0.001*bnb];%第一个参数是期望翻滚角row；第二个参数是期望俯仰角pitch；
                     %第三个参数是期望偏航角yaw；第四个参数是期望机身高度
      bnb = bnb + 1;
   elseif  bnb >100 && bnb <200
      userValue = [0;pi/900*(bnb-100);0;-0.05];%第一个参数是期望翻滚角row；第二个参数是期望俯仰角pitch；
                     %第三个参数是期望偏航角yaw；第四个参数是期望机身高度
      bnb = bnb + 1;
   end
     
   
   vecOP = calcOP(userValue(1),userValue(2),userValue(3),userValue(4),initVecOX_,initVecXP_);%根据指令计算其他足端在机身坐标系下的期望位置
   calcCmd(vecOP,Robot,state);%根据足端在机身坐标系的期望坐标，通过逆运动学换算成关节角度进行控制
   
   R_angle = wb_inertial_unit_get_roll_pitch_yaw(IMU);
   fprintf('现在的欧拉角: %f  %f  %f\n',R_angle(1),R_angle(2),R_angle(3));
   
   drawnow;
end
function vecOP = calcOP(row,pitch,yaw,height,initVecOX,initVecXP)%计算其他足端在机身坐标系下的期望位置
    vecXO = zeros(3,1);
    vecXO = -initVecOX;%获取1号足端在机身坐标系下的坐标并取反，作为四足机器人质心在世界坐标系下的初始期望坐标
    vecXO(3) = vecXO(3) + height;
    fprintf('Current motor angle: %f  %f  %f\n',vecXO(1),vecXO(2),vecXO(3));
    
    rotM = rpyToRotMat(row,pitch,yaw);
    
    Tsb = homoMatrix(vecXO,rotM);
    Tbs = inv(Tsb);
    
    vecOP = zeros(3,4);
    for i = 1:4
        tempVec4 = Tbs*HomoVec(initVecXP(:,i));%根据位姿变换矩阵与其他足端在世界坐标系中的初始齐次坐标计算其他足端应该在机身坐标系下的齐次坐标
        vecOP(:,i) = noHomoVec(tempVec4);%将上一步获得的齐次坐标变成三维向量
    end
end

 function R_sb = rpyToRotMat(roll, pitch, yaw)%旋转矩阵
     R_sb = [cos(yaw)*cos(pitch) cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll) cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll);
             sin(yaw)*cos(pitch) sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll) sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);
             -sin(pitch)         cos(pitch)*sin(roll)                             cos(pitch)*cos(roll)                            ];
 end
 
 function homoM = homoMatrix(vec,rot) %将旋转矩阵和平移向量拼成齐次变换矩阵
     zero_2 = [0 0 0];
     homoM = [rot  vec;
              zero_2 1];
 end
 
 function Vec_1 = HomoVec(Vec)  %给3维向量的末尾加一项1，使其变成四维齐次坐标
     Vec_1 = [Vec;1];
 end
 
 function Vec_1 = noHomoVec(Vec)  %去掉思维其次坐标最后一项，使其变成三维向量
     Vec_1 = Vec(1:3);
 end
 
 function calcCmd(vecOP,Robot,state)
     q = Robot.getQ(vecOP,'BODY');%已知四个足端在机身坐标系下的位置坐标，获取机器人全部12个关节的角度
     state.setQ(q);%对关节进行控制
 end

 
 
% 本项目是基于宇树开源项目https://github.com/unitreerobotics/unitree_guide进行的，只实现了皮毛，还有很多很多的不足；
% 
% 感谢宇树公司出版的《四足机器人控制算法——建模、控制与实践》这本书，让我可以学会四足机器人的控制，并使我能够理解工程代码。
% 
% 目前只开源运动学验证部分，待之后进行整理再发布其他。虽然说是“开源”，但更像是分享，因为自己只做了点皮毛，只是希望能帮助刚入门的同学。
% 
% 最后还是建议大家，理解后直接C+ROS入手，也看一下宇树的这本书，让人受益匪浅。


%来源：B站——起名字不是问题