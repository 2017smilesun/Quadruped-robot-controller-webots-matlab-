classdef LowLevelState < handle
    %STATE 本类主要是通过各种传感器获取测量值，并且利用计算所得的值对电机等进行控制

    
    properties
        joint_device;joint_sensor_device;IMU;Gyro;trunk_body;Robot;foot_DEF;
    end
    
    methods
        function obj = LowLevelState(joint_device,joint_sensor_device,IMU,Gyro,trunk_body,foot_DEF)
           obj.joint_device = joint_device;
           obj.joint_sensor_device = joint_sensor_device;
           obj.IMU = IMU;
           obj.Gyro = Gyro;
           obj.trunk_body = trunk_body;
           obj.Robot = QuadrupedRobot(obj);
           %obj.joint_Vel_DEF = joint_Vel_DEF;
           obj.foot_DEF = foot_DEF;
        end

        function  feetpos_cur = getQ(obj)%获取关节角度
            feetpos_cur = zeros(3,4);
            for i = 1:4
                feetpos_cur(1,i) = wb_position_sensor_get_value(obj.joint_sensor_device(1+3*(i-1)));
                feetpos_cur(2,i) = wb_position_sensor_get_value(obj.joint_sensor_device(2+3*(i-1)));
                feetpos_cur(3,i) = wb_position_sensor_get_value(obj.joint_sensor_device(3+3*(i-1)));
            end
        end
        function  feetvel_cur = getQd(obj)%获取关节角速度
            feetvel_cur = zeros(3,4);
            for i = 1:4
                feetvel_cur(1,i) = wb_motor_get_velocity(obj.joint_device(1+3*(i-1)));
                feetvel_cur(2,i) = wb_motor_get_velocity(obj.joint_device(2+3*(i-1)));
                feetvel_cur(3,i) = wb_motor_get_velocity(obj.joint_device(3+3*(i-1)));
            end
        end
        
        function RotMat = getRotMat(obj)  %%通过inertial_unit获得绕x/y/z轴的角度
            R_angle = wb_inertial_unit_get_roll_pitch_yaw(obj.IMU);
            yaw = R_angle(3);
            pitch = R_angle(2);
            roll = R_angle(1);
            RotMat = [cos(yaw)*cos(pitch) cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll) cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll);
                      sin(yaw)*cos(pitch) sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll) sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);
                      -sin(pitch)         cos(pitch)*sin(roll)                             cos(pitch)*cos(roll)                            ];
        end
        
        function Gyro_vel = getGyro(obj)  %通过陀螺仪获得绕x/y/z轴旋转的角速度
            Gyro_vel = wb_gyro_get_values(obj.Gyro);
            Gyro_vel = Gyro_vel';
            %fprintf('现在的陀螺仪: %f  %f  %f\n',Gyro_vel(1),Gyro_vel(2),Gyro_vel(3));
        end
        
        function setQ(obj,q)   %%利用计算所得的角度控制电机
            for i = 1:12
                wb_motor_set_position(obj.joint_device(i), q(i));
                wb_motor_set_velocity(obj.joint_device(i), inf);
            end
        end
        
        function BodyPos = getBodyPos(obj) %获取机身几何中心在世界坐标系下的坐标
            BodyPos = wb_supervisor_node_get_position(obj.trunk_body)';
        end
        
        function BodyVel = getBodyVel(obj)  %获取机身中心的速度
            BodyVel6 = wb_supervisor_node_get_velocity(obj.trunk_body);
            BodyVel = BodyVel6(1:3);   %wb_supervisor_node_get_velocity获取六个值（沿x/y/z轴的线速度，绕x/y/z轴的角速度）
            BodyVel = BodyVel';  %只取前三个值，这里是为了之后的矩阵乘法的维度统一
        end
        
        function GyroGlobal = getGyroGlobal(obj)   %计算世界坐标系下的机身绕x/y/z轴旋转的角速度
            GyroGlobal = obj.getRotMat() * obj.getGyro();
        end
        
        function Yaw = getYaw(obj)%获取机身在世界坐标系下的目标偏航角
            R_angle = wb_inertial_unit_get_roll_pitch_yaw(obj.IMU);
            Yaw = R_angle(3);
        end
        
        function DYaw = getDYaw(obj)  %获取机器人绕y轴的角速度
            DYaw_3 = obj.getGyroGlobal();
            DYaw = DYaw_3(3);
        end
        
        function feet2BPos = getPosFeet2BGlobal(obj) %获取足端在世界坐标系下相对于机身中心的位置坐标
            feet2BPos = zeros(3,4);
            for i = 1:4
                feet2BPos(:,i) = obj.getFootPos(i) - obj.getBodyPos();
            end
        end
        
        function FootPos = getFootPos(obj,i) 
            FootPos = obj.getBodyPos() + obj.getRotMat() * obj.Robot.getFootPosition(i, 'BODY');
        end
        
        function FeetPos = getFeetPos(obj) %获取足端在世界坐标系下的位置坐标
            FeetPos = zeros(3,4);
            for i = 1:4
                FeetPos(:,i) = obj.getFootPos(i);
            end
            %fprintf('FeetPos的大小: %f %f %f\n',FeetPos(1,1),FeetPos(2,1),FeetPos(3,1));
        end
        
        function FeetVel = getFeetVel(obj)
            feetVel = zeros(3,4);
            FR_foot_vel = wb_supervisor_node_get_velocity(obj.foot_DEF(1));
            FL_foot_vel = wb_supervisor_node_get_velocity(obj.foot_DEF(2));
            RR_foot_vel = wb_supervisor_node_get_velocity(obj.foot_DEF(3));
            RL_foot_vel = wb_supervisor_node_get_velocity(obj.foot_DEF(4));
            
            FeetVel(:,1) = FR_foot_vel(1:3);
            FeetVel(:,2) = FL_foot_vel(1:3);
            FeetVel(:,3) = RR_foot_vel(1:3);
            FeetVel(:,4) = RL_foot_vel(1:3);
            %fprintf('FeetVel的大小: %f %f %f\n',FeetVel(1,1),FeetVel(2,1),FeetVel(3,1)); 
        end
        
        function setTau(obj,Tau)    %利用计算所得的各个关节力矩控制电机
           %fprintf('Tau的维度: %f  %f\n',size(Tau,1),size(Tau,2));
           wb_motor_set_torque(obj.joint_device(1), Tau(1));
           wb_motor_set_torque(obj.joint_device(2), Tau(2));
           wb_motor_set_torque(obj.joint_device(3), Tau(3));
           wb_motor_set_torque(obj.joint_device(4), Tau(4));
           wb_motor_set_torque(obj.joint_device(5), Tau(5));
           wb_motor_set_torque(obj.joint_device(6), Tau(6));
           wb_motor_set_torque(obj.joint_device(7), Tau(7));
           wb_motor_set_torque(obj.joint_device(8), Tau(8));
           wb_motor_set_torque(obj.joint_device(9), Tau(9));
           wb_motor_set_torque(obj.joint_device(10), Tau(10));
           wb_motor_set_torque(obj.joint_device(11), Tau(11));
           wb_motor_set_torque(obj.joint_device(12), Tau(12));
        end
        
        function setTauID(obj, Tau, legID)
            wb_motor_set_torque(obj.joint_device((legID-1)*3+1), Tau((legID-1)*3+1));
            wb_motor_set_torque(obj.joint_device((legID-1)*3+2), Tau((legID-1)*3+2));
            wb_motor_set_torque(obj.joint_device((legID-1)*3+3), Tau((legID-1)*3+3));
        end
        
        function setQID(obj, q, legID)
           for i = (legID-1)*3+1:(legID-1)*3+3
               wb_motor_set_position(obj.joint_device(i), q(i));
               wb_motor_set_velocity(obj.joint_device(i), inf);
           end
        end
    end
end

