classdef QuadrupedRobot < handle
    %UNTITLED �˴���ʾ�йش�����ժҪ
    %   �����������������˶�ѧ�Ͷ���ѧ
    % 1������ǰ��
    % 2������ǰ��
    % 3�����Һ���
    % 4����������
    
    properties
        Legs;state;
%         Legs1;Legs2;Legs3;Legs4
    end
    

    methods
        function obj = QuadrupedRobot(state_1)
            %UNTITLED ����������ʵ��
            %   �˴���ʾ��ϸ˵��
%             obj.Legs = struct();
            Legs1 = QuadrupedLeg(1, 0.0838, 0.2, 0.2, [0.1805; -0.047; 0]);
            Legs2 = QuadrupedLeg(2, 0.0838, 0.2, 0.2, [0.1805; 0.047; 0]);
            Legs3 = QuadrupedLeg(3, 0.0838, 0.2, 0.2, [-0.1805; -0.047; 0]);
            Legs4 = QuadrupedLeg(4, 0.0838, 0.2, 0.2, [-0.1805; 0.047; 0]);
            obj.Legs = [Legs1,Legs2,Legs3,Legs4];
            obj.state = state_1;
        end
        
        %%��������λ��                %%%%****%%%%state
%         function X1 = getX() %%��ȡ�����˵�id�����ڻ�������ϵ�µ�λ������
%             X1 = getFootPosition(1, BODY);
%         end
        
        %%getVecXP����ûд                         %%%%****%%%%
        function vecXP = getVecXP(obj)
            vecXP = zeros(3,4);
            x = obj.getFootPosition(1, 'BODY'); %%��ȡ�����˵�id�����ڻ�������ϵ�µ�λ������
            qLegs = obj.state.getQ();
            for i = 1:4
                vecXP(:,i) = obj.Legs(i).calcPEe2B(qLegs(:,i)) - x; %%calcPEe2B ��֪�ؽڽǶȼ������˵��������ĵ���������
            end
        end
        
        
        %%�����˶�ѧ,��֪�ĸ����˵�λ�����꣬��ȡ������ȫ��12���ؽڵĽǶ�
        function q = getQ(obj, vecp, frame)
            q = zeros(1,12);
            for i=1:4
                q(1+3*(i-1):3*i) = obj.Legs(i).calcQ(vecp(:,i),frame);
            end
        end
        
        %%����΢���˶�ѧ������֪�ĸ����˵�λ�ã��ĸ����˵��ٶȣ���ȡ��ǰ������ȫ��12���ؽڽ��ٶ�
        function qd = getQd(pos, vel, frame)
            qd = zeros(1.12);
            for i=1:4
                qd(1+3*(i-1):3*i) = obj.Legs(i).calcQd(pos(:,i),vel(:,i),frame);
            end
        end
        
        %%�����˾���ѧ���㣬��֪12���ؽڽǶȣ��ĸ����˶�������������ȡ��ǰ������ȫ��12���ؽڵ�����
        function tau = getTau(obj,q, feetForce)
            %fprintf('q的维度: %f  %f\nfeetForce的维度: %f  %f',size(q,1),size(q,2),size(feetForce,1),size(feetForce,2));
            for i=1:4
                tau(1+3*(i-1):3*i) = obj.Legs(i).calcTau(q(3*(i-1)+1:3*i),feetForce(:,i));
            end
        end
        
        %%�����˶�ѧ, ��ȡ�����˵�id������frame����ϵ�µ�λ������
        function X = getFootPosition(obj, id, frame)   %%%%****%%%%
            qLegs = obj.state.getQ();
            %fprintf('first all angle: %f  %f  %f  %f  %f  %f  %f  %f  %f  %f  %f  %f\n',qLegs(1),qLegs(2),qLegs(3),qLegs(4),qLegs(5),qLegs(6),qLegs(7),qLegs(8),qLegs(9),qLegs(10),qLegs(11),qLegs(12));
%               qLegs = [1, 2, 3, 4;
%                        5, 6, 7, 8;
%                        9, 10, 11, 12];   
            if strcmp(frame,'BODY')
                X = obj.Legs(id).calcPEe2B(qLegs(:,id));
            elseif strcmp(frame,'HIP')
                X = obj.Legs(id).calcPEe2H(qLegs(:,id));
            else
                disp('[ERROR]The frame of function:getFootPosition can only be BODY or HIP.')
                return
            end
        end
        
        %%����΢���˶�ѧ����,��ȡ�����˵�id���ȵ��ٶ�����
        function qd = getFootVelocity(obj,id)
            qLegs = obj.state.getQ();
            qdLegs = obj.state.getQd();
            qd = obj.Legs(id).calcVEe(qLegs(:,id),qdLegs(:,id)); %%calcVEe ��֪�ؽڽǶ�q�ͽ��ٶ�qdʱ���������ٶ��������鹫ʽ5.42
        end
        
        %%�����˶�ѧ,��ȡ�������������ڻ������ĵ�λ�����꣬frame����HIP��BODY�⣬��������GLOBAL
        function feetPos = getFeet2BPositions(obj,frame)
            feetPos = zeros(3,4);
            if strcmp(frame,'GLOBAL')
                for i=1:4
                    feetPos(:,i) = obj.getFootPosition(i, BODY);   %%���û�������ϵ����������
                end
                feetPos = obj.state.getRotMat() * feetPos;     %%getRotMat����Ϊ��ȡ��ǰ��̬Rsb�����д��뽫��������ת����������ϵ��
            elseif strcmp(frame,'BODY') || strcmp(frame,'HIP')
                for i=1:4
                    feetPos(:,i) = obj.getFootPosition(i, frame);
                end
            else
                disp('[ERROR] Frame error of function getFeet2BPositions');
            end
        end
        
        %%��ȡ�������������ڻ������ĵ��ٶ�����������GLOBAL����ϵ�£��ο�ʽ6.13������
        function feetVel2B = getFeet2BVelocities(obj,frame)
            feetVel = zeros(3,4);
            for i = 1:4
                feetVel(:,i) = obj.getFootVelocity(i);
            end
            if strcmp(frame,'GLOBAL')
                feetPos = obj.getFeet2BPositions('BODY');
                feetVel = feetVel + skew(obj.state.getGyro()) *feetPos;   %skew����������תΪ���˾���,getGyro����������x/y/z���Ľ��ٶ�
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
        

         
        
%         function outputArg = method1(obj,inputArg)
%             %METHOD1 �˴���ʾ�йش˷�����ժҪ
%             %   �˴���ʾ��ϸ˵��
%             outputArg = obj.Property1 + inputArg;
%         end
    end
end

