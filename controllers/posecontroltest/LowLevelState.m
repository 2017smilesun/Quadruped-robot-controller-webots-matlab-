classdef LowLevelState < handle

    properties
        joint_device;joint_sensor_device;IMU;Gyro;
    end
    
    methods
        function obj = LowLevelState(joint_device,joint_sensor_device,IMU,Gyro)
           obj.joint_device = joint_device;
           obj.joint_sensor_device = joint_sensor_device;
           obj.IMU = IMU;
           obj.Gyro = Gyro;
        end

        function  feetpos_cur = getQ(obj)  %获取各关节位置
            feetpos_cur = zeros(3,4);
            for i = 1:4
                feetpos_cur(1,i) = wb_position_sensor_get_value(obj.joint_sensor_device(1+3*(i-1)));
                feetpos_cur(2,i) = wb_position_sensor_get_value(obj.joint_sensor_device(2+3*(i-1)));
                feetpos_cur(3,i) = wb_position_sensor_get_value(obj.joint_sensor_device(3+3*(i-1)));
            end
        end
        
        function  feetvel_cur = getQd(obj)  %%获取各关节速度
            feetvel_cur = zeros(3,4);
            for i = 1:4
                feetvel_cur(1,i) = wb_motor_get_velocity(obj.joint_sensor_device(1+3*(i-1)));
                feetvel_cur(2,i) = wb_motor_get_velocity(obj.joint_sensor_device(2+3*(i-1)));
                feetvel_cur(3,i) = wb_motor_get_velocity(obj.joint_sensor_device(3+3*(i-1)));
            end
        end
        
        function RotMat = getRotMat(obj)%获取旋转矩阵
            R_angle = wb_inertial_unit_get_roll_pitch_yaw(obj.IMU);
            yaw = R_angle(3);
            pitch = R_angle(2);
            roll = R_angle(1);
            RotMat = [cos(yaw)*cos(pitch) cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll) cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll);
                      sin(yaw)*cos(pitch) sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll) sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);
                      -sin(pitch)         cos(pitch)*sin(roll)                             cos(pitch)*cos(roll)                            ];
        end
        
        function Gyro_vel = getGyro(obj)
            Gyro_vel = wb_gyro_get_values(obj.Gyro);
        end
        
        function setQ(obj,q)%对各关节进行控制
            for i = 1:12
                wb_motor_set_position(obj.joint_device(i), q(i));
                wb_motor_set_velocity(obj.joint_device(i), inf);
            end
        end
    end
end


% 本项目是基于宇树开源项目https://github.com/unitreerobotics/unitree_guide进行的，只实现了皮毛，还有很多很多的不足；
% 
% 感谢宇树公司出版的《四足机器人控制算法——建模、控制与实践》这本书，让我可以学会四足机器人的控制，并使我能够理解工程代码。
% 
% 目前只开源运动学验证部分，待之后进行整理再发布其他。虽然说是“开源”，但更像是分享，因为自己只做了点皮毛，只是希望能帮助刚入门的同学。
% 
% 最后还是建议大家，理解后直接C+ROS入手，也看一下宇树的这本书，让人受益匪浅。

%来源：B站——起名字不是问题