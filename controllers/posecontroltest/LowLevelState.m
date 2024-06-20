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

        function  feetpos_cur = getQ(obj)  %��ȡ���ؽ�λ��
            feetpos_cur = zeros(3,4);
            for i = 1:4
                feetpos_cur(1,i) = wb_position_sensor_get_value(obj.joint_sensor_device(1+3*(i-1)));
                feetpos_cur(2,i) = wb_position_sensor_get_value(obj.joint_sensor_device(2+3*(i-1)));
                feetpos_cur(3,i) = wb_position_sensor_get_value(obj.joint_sensor_device(3+3*(i-1)));
            end
        end
        
        function  feetvel_cur = getQd(obj)  %%��ȡ���ؽ��ٶ�
            feetvel_cur = zeros(3,4);
            for i = 1:4
                feetvel_cur(1,i) = wb_motor_get_velocity(obj.joint_sensor_device(1+3*(i-1)));
                feetvel_cur(2,i) = wb_motor_get_velocity(obj.joint_sensor_device(2+3*(i-1)));
                feetvel_cur(3,i) = wb_motor_get_velocity(obj.joint_sensor_device(3+3*(i-1)));
            end
        end
        
        function RotMat = getRotMat(obj)%��ȡ��ת����
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
        
        function setQ(obj,q)%�Ը��ؽڽ��п���
            for i = 1:12
                wb_motor_set_position(obj.joint_device(i), q(i));
                wb_motor_set_velocity(obj.joint_device(i), inf);
            end
        end
    end
end


% ����Ŀ�ǻ���������Դ��Ŀhttps://github.com/unitreerobotics/unitree_guide���еģ�ֻʵ����Ƥë�����кܶ�ܶ�Ĳ��㣻
% 
% ��л������˾����ġ���������˿����㷨������ģ��������ʵ�����Ȿ�飬���ҿ���ѧ����������˵Ŀ��ƣ���ʹ���ܹ���⹤�̴��롣
% 
% Ŀǰֻ��Դ�˶�ѧ��֤���֣���֮����������ٷ�����������Ȼ˵�ǡ���Դ�����������Ƿ�����Ϊ�Լ�ֻ���˵�Ƥë��ֻ��ϣ���ܰ��������ŵ�ͬѧ��
% 
% ����ǽ����ң�����ֱ��C+ROS���֣�Ҳ��һ���������Ȿ�飬���������ǳ��

%��Դ��Bվ���������ֲ�������