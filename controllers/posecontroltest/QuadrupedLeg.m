classdef QuadrupedLeg < handle
    %UNTITLED 此处显示有关此类的摘要
    %   宇树四足机器人腿部运动学计算
    
    properties
        sideSign;abadLinkLength;hipLinkLength;kneeLinkLength;pHip2B;
    end
    
    methods
%         function obj = untitled(inputArg1,inputArg2)
%             %UNTITLED 构造此类的实例
%             %   此处显示详细说明
%             obj.Property1 = inputArg1 + inputArg2;
%         end
        
        %%QuadrupedLeg类的构造函数，判断是左腿还是右腿
        function obj = QuadrupedLeg(LegID,abadLinkLength,hipLinkLength,kneeLinkLength,pHip2B)
            if (LegID == 1 || LegID == 3)
                obj.sideSign = -1;
            elseif (LegID == 2 || LegID == 4)
                obj.sideSign = 1;
            else
                disp('Leg ID incorrect!');
                return
            end
            obj.abadLinkLength = abadLinkLength;
            obj.hipLinkLength = hipLinkLength;
            obj.kneeLinkLength = kneeLinkLength;
            obj.pHip2B = pHip2B;
            
        end
        
        
        %%正向运动学
        function pEe2H = calcPEe2H(obj,q)   %%已知关节角度q计算足端到腿基座坐标系原点的向量坐标(公式5.11）
            pEe2H = zeros(3,1);
            
            l1 = obj.sideSign * obj.abadLinkLength;
            l2 = -obj.hipLinkLength;
            l3 = -obj.kneeLinkLength;
            
            s1 = sin(q(1));
            s2 = sin(q(2));
            s3 = sin(q(3));
            
            c1 = cos(q(1));
            c2 = cos(q(2));
            c3 = cos(q(3));
            
            c23 = c2 * c3 - s2 * s3;
            s23 = s2 * c3 + c2 * s3;
            
            pEe2H(1) = l3 * s23 + l2 * s2;
            pEe2H(2) = -l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1;
            pEe2H(3) =  l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2;
        end
        
        %%Forward kinematics
        function pEe2H2B = calcPEe2B(obj,q)    %%已知关节角度计算足端到机身中心的向量坐标，足端到腿基座坐标系原点的向量坐标+腿基座坐标系到机身中心的向量
            pEe2H2B = obj.pHip2B + obj.calcPEe2H(q);
        end
        
        %%导数正向运动学
        function vEe = calcVEe(q,qd)  %%已知关节角度q和角速度qd时，求足端速度向量，书公式5.42
            vEe = calcJaco(q) * qd;
        end
        

        %%逆运动学
        function qResult = calcQ(obj,pEe,frame)  %%计算当足端坐标为pEe时腿上三个关节角度；
%             pEe2H; %足端到腿基座坐标系原点的向量坐标
            if strcmp(frame,'HIP')
                pEe2H = pEe;
            elseif strcmp(frame,'BODY')
                pEe2H = pEe - obj.pHip2B;
            else
                fprintf("[ERROR] The frame of QuadrupedLeg::calcQ can only be HIP or BODY!");
            end
%             q1,q2,q3;%三个关节角度
%             px,py,pz;%足端坐标
%             b2y,b3z,b4z,a,b,c;%
            
            px = pEe2H(1); %将基座坐标系下足端坐标分成xyz
            py = pEe2H(2);
            pz = pEe2H(3);
            
            b2y = obj.abadLinkLength * obj.sideSign;   %labcd的长度，判断是左腿还是右腿
            b3z = -obj.hipLinkLength;              %大腿长度，z坐标
            b4z = -obj.kneeLinkLength;             %小腿长度，z坐标
            a = obj.abadLinkLength;
            c = sqrt(px^2 + py^2 + pz^2);     % whole length     %pow函数计算x的y次幂
            b = sqrt(c^2 - a^2);              % distance between shoulder and footpoint
            
            q1 = q1_ik(py,pz,b2y);                         %计算theta1
            q3 = q3_ik(b3z, b4z, b);                  %计算theta3
            q2 = q2_ik(q1, q3, px, py, pz, b3z, b4z); %计算theta2

                    %%求解逆运动学，计算theta1,公式5.17
           function q1 = q1_ik(py1,pz1,l1)
                L = sqrt(py1^2+pz1^2-l1^2);
                q1 = atan2(pz1*l1+py1*L,py1*l1-pz1*L);
           end
        
        %%计算theta3,公式5.23
            function q3 = q3_ik(b3z,b4z,b)
                temp = (b3z^2+b4z^2-b^2)/(2*abs(b3z*b4z));
                if temp > 1
                    temp = 1;
                elseif temp < -1
                    temp = -1;
                end
                    q3 = acos(temp);
                    q3 = -(pi - q3);%%0~180
            end
        
        %%计算theta2,公式5.37
            function q2 = q2_ik(q1, q3, px, py, pz, b3z, b4z)
                a1 = py*sin(q1) - pz*cos(q1); 
                a2 = px;
                m1 = b4z*sin(q3);
                m2 = b3z + b4z*cos(q3);
                q2 = atan2(m1*a1+m2*a2, m1*a2-m2*a1);
            end
            
            qResult(1) = q1;
            qResult(2) = q2;
            qResult(3) = q3;
        end
        
           
        %%导数逆运动学
        function inVEe = calcQd(q,vEe)
            inJaco = inv(calcJaco(q));
            inVEe = inJaco * vEe;
        end
        
        %%导数逆运动学,根据足端在frame坐标系下的位置坐标pEe和速度vEe计算三个关节的角速度，可以认为该函数是上两个函数的融合
        function inVEe = calcQdd(pEe,vEe,frame)
            q = calcQ(pEe,frame);
            inJaco = inv(calcJaco(q));
            inVEe = inJaco * vEe;
        end
        
        %%逆动力学
        function Tau = calcTau(q,force)
            Tau = obj.calcJaco(q)' * force;
        end
        
        %%雅可比矩阵
        function jaco = calcJaco(obj,q)
            l1 = obj.sideSign * obj.abadLinkLength;
            l2 = -obj.hipLinkLength;
            l3 = -obj.kneeLinkLength;
            
            s1 = sin(q(1));
            s2 = sin(q(2));
            s3 = sin(q(3));
            
            c1 = cos(q(1));
            c2 = cos(q(2));
            c3 = cos(q(3));
            
            c23 = c2 * c3 - s2 * s3;
            s23 = s2 * c3 + c2 * s3;
            jaco(1, 1) = 0;
            jaco(2, 1) = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1;
            jaco(3, 1) = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1;
            jaco(1, 2) = l3 * c23 + l2 * c2;
            jaco(2, 2) = l3 * s1 * s23 + l2 * s1 * s2;
            jaco(3, 2) = -l3 * c1 * s23 - l2 * c1 * s2;
            jaco(1, 3) = l3 * c23;
            jaco(2, 3) = l3 * s1 * s23;
            jaco(3, 3) = -l3 * c1 * s23;
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