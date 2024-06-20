classdef QuadrupedRobot < handle
    %   宇树机器人四足的运动学和动力学
    % 1——右前腿
    % 2——左前腿
    % 3——右后腿
    % 4——左后腿
    
    properties
        Legs;state;
%         Legs1;Legs2;Legs3;Legs4
    end
    

    methods
        function obj = QuadrupedRobot(state_1)
            Legs1 = QuadrupedLeg(1, 0.0838, 0.2, 0.2, [0.1805; -0.047; 0]);
            Legs2 = QuadrupedLeg(2, 0.0838, 0.2, 0.2, [0.1805; 0.047; 0]);
            Legs3 = QuadrupedLeg(3, 0.0838, 0.2, 0.2, [-0.1805; -0.047; 0]);
            Legs4 = QuadrupedLeg(4, 0.0838, 0.2, 0.2, [-0.1805; 0.047; 0]);
            obj.Legs = [Legs1,Legs2,Legs3,Legs4];
            obj.state = state_1;
        end
        
        
        function vecXP = getVecXP(obj)
            vecXP = zeros(3,4);
            x = obj.getFootPosition(1, 'BODY'); %获取机器人第id条腿在机身坐标系下的位置坐标
            qLegs = obj.state.getQ();
            for i = 1:4
                vecXP(:,i) = obj.Legs(i).calcPEe2B(qLegs(:,i)) - x; %calcPEe2B 已知关节角度计算足端到机身中心的向量坐标
            end
        end
        
        
        %逆向运动学,已知四个足端的位置坐标，获取机器人全部12个关节的角度
        function q = getQ(obj, vecp, frame)
            q = zeros(1,12);
            for i=1:4
                q(1+3*(i-1):3*i) = obj.Legs(i).calcQ(vecp(:,i),frame);
            end
        end
        
       %逆向微分运动学计算已知四个足端的位置，四个足端的速度，获取当前机器人全部12个关节角速度
        function qd = getQd(obj, pos, vel, frame)
            qd = zeros(12,1);
            for i=1:4
                qd(1+3*(i-1):3*i) = obj.Legs(i).calcQdd(pos(:,i),vel(:,i),frame);
            end
        end
        
        %机器人静力学计算，已知12个关节角度，四个足端对外作用力，获取当前机器人全部12个关节的力矩
        function tau = getTau(obj,q, feetForce)
            %fprintf('q的维度: %f  %f\nfeetForce的维度: %f  %f',size(q,1),size(q,2),size(feetForce,1),size(feetForce,2));
            for i=1:4
                tau(1+3*(i-1):3*i) = obj.Legs(i).calcTau(q(3*(i-1)+1:3*i),feetForce(:,i));
            end
        end
        
        %正向运动学, 获取机器人第id条腿在frame坐标系下的位置坐标
        function X = getFootPosition(obj, id, frame)  
            qLegs = obj.state.getQ(); 
            if strcmp(frame,'BODY')
                X = obj.Legs(id).calcPEe2B(qLegs(:,id));
            elseif strcmp(frame,'HIP')
                X = obj.Legs(id).calcPEe2H(qLegs(:,id));
            else
                disp('[ERROR]The frame of function:getFootPosition can only be BODY or HIP.')
                return
            end
        end
        
        %正向微分运动学计算,获取机器人第id条腿的速度向量
        function qd = getFootVelocity(obj,id)
            qLegs = obj.state.getQ();
            qdLegs = obj.state.getQd();
            %fprintf('qdLegs的大小: %f %f %f\n',qdLegs(1,1),qdLegs(2,1),qdLegs(3,1)); 
            qd = obj.Legs(id).calcVEe(qLegs(:,id),qdLegs(:,id));%calcVEe 已知关节角度q和角速度qd时，求足端速度向量，书公式5.42
        end
        
         %正向运动学,获取所有足端相对于机身中心的位置坐标，frame除了HIP和BODY外，还可以是GLOBAL
        function feetPos = getFeet2BPositions(obj,frame)
            feetPos = zeros(3,4);
            if strcmp(frame,'GLOBAL')
                for i=1:4
                    feetPos(:,i) = obj.getFootPosition(i, 'BODY');   %获得机身坐标系下足端坐标
                end
                feetPos = obj.state.getRotMat() * feetPos;      %getRotMat函数为获取当前姿态Rsb，该行代码将足端坐标转到世界坐标系下
            elseif strcmp(frame,'BODY') || strcmp(frame,'HIP')
                for i=1:4
                    feetPos(:,i) = obj.getFootPosition(i, frame);
                end
            else
                disp('[ERROR] Frame error of function getFeet2BPositions');
            end
        end
        
        %获取所有足端相对于机身中心的速度向量，若是GLOBAL坐标系下，参考式6.13
        function feetVel2B = getFeet2BVelocities(obj,frame)
            feetVel = zeros(3,4);
            for i = 1:4
                feetVel(:,i) = obj.getFootVelocity(i);
            end
            fprintf('feetVel的大小: %f %f %f\n',feetVel(1,1),feetVel(2,1),feetVel(3,1)); 
            if strcmp(frame,'GLOBAL')
                feetPos = obj.getFeet2BPositions('BODY');
                feetVel = feetVel + skew(obj.state.getGyro()) *feetPos;   %skew函数将向量转为叉乘矩阵,getGyro函数获得绕x/y/z轴的角速度
                feetVel2B = obj.state.getRotMat() * feetVel;
            elseif strcmp(frame,'BODY') || strcmp(frame,'HIP')
                feetVel2B = obj.getFeet2BPositions('BODY');
            else
                disp('[ERROR] Frame error of function getFeet2BVelocities');
            end
        end
        
        %%���õ�LegID���ȵ��ſɱȾ���
        function jacoAll = getJaco(obj,legID)
            getQID = obj.state.getQ();
            jacoAll = obj.Legs(legID).calcJaco(getQID(:,legID));
        end
        
        function feetPosNormalStand = getFeetPosIdeal(static) %代表各个足端中性落脚点在机身坐标系{b}下的坐标
            feetPosNormalStand = [ 0.1805,  0.1805, -0.1805, -0.1805;
                                  -0.1308,  0.1308, -0.1308,  0.1308;
                                  -0.3180, -0.3180, -0.3180, -0.3180];
        end
        

        % 本项目是基于宇树开源项目https://github.com/unitreerobotics/unitree_guide进行的，只实现了皮毛，还有很多很多的不足；
        % 
        % 感谢宇树公司出版的《四足机器人控制算法——建模、控制与实践》这本书，让我可以学会四足机器人的控制，并使我能够理解工程代码。
        % 
        %虽然说是“开源”，但更像是分享，因为自己只做了点皮毛，只是希望能帮助刚入门的同学。
        % 
        % 最后还是建议大家，理解后直接C+ROS入手，也看一下宇树的这本书，让人受益匪浅。

        %来源：B站——起名字不是问题
    end
end

