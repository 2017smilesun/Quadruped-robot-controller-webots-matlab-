classdef BalanceCtrl < handle
    
    %BALANCECTRL �˴���ʾ�йش�����ժҪ
    %   �˴���ʾ��ϸ˵��
    %%%%%******дһ��skew���󣬽�����תΪ���˾���
    
    properties
        pcb;
        S;W;U;Fprev;fricMat;
        
    end
    
    properties(Constant) %���õ��ĳ�������
        I3 = [1, 0, 0;
              0, 1, 0;
              0, 0, 1];
        g = [0;0;-9.81];
        Ib = [0.132, 0,  0;
              0, 0.3475, 0;
              0, 0, 0.3775];%��ȡ�����˵Ĺ�����������
        w = [10;10;4;10;10;4;10;10;4;10;10;4];
        u = [3;3;3;3;3;3;3;3;3;3;3;3];
        alpha = 0.001;
        beta = 0.1;
        fricRatio = 0.3;
        mass = 13.4;%��ȡ����������
        s = [20;20;50;450;450;450];
    end
    
    methods
        function obj = BalanceCtrl()

            obj.pcb = [0; 0; 0];%�����ǻ�������ϵ�»��������ĵ�����

            obj.S = zeros(6,6);
            for i = 1:6
                obj.S(i,i) = obj.s(i);
            end
            
            obj.W = zeros(12,12);
            for i = 1:12
                obj.W(i,i) = obj.w(i);
            end
            
            obj.U = zeros(12,12);
            for i = 1:12
                obj.U(i,i) = obj.u(i);
            end
            
            obj.Fprev = zeros(12,1);  %��һ�ּ����õ�������������
            obj.fricMat = [1, 0, obj.fricRatio;
                          -1, 0, obj.fricRatio;
                           0, 1, obj.fricRatio;
                           0,-1, obj.fricRatio;
                           0, 0, 1];
        end
        
        function F12to34 = calF(obj,ddPcd, dwbd, rotM, feetPos2B, contact) %ddPcd������Ŀ�����ٶȣ�dwbd������Ŀ���Ǽ��ٶȣ�rotM�����ĵ�ǰ��̬��feetPos2B��3,4�������������ĵ��������˵���������������ϵ�µ�����
            A = obj.calMatrixA(feetPos2B, rotM, contact);%����ʽ8.35�е�A����
            Bd = obj.calVectorBd(ddPcd, dwbd, rotM);%����ʽ8.35�е�bd����
            [CI,CE,ci0,ce0] = obj.calConstraints(contact); %���㲻��ʽԼ���͵�ʽԼ����ϵ��
            
            G = A'*obj.S* A +obj.alpha* obj.W+ obj.beta*obj.U;
            g0T = -Bd'* obj.S* A - obj.beta*obj.Fprev'*obj.U;
            
            F = obj.solveQP(G,g0T,CI,CE,ci0,ce0);
            
            obj.Fprev = F;
            F12to34 = zeros(3,4);
            F12to34(1:3,1) = F(1:3);
            F12to34(1:3,2) = F(4:6);
            F12to34(1:3,3) = F(7:9);
            F12to34(1:3,4) = F(10:12);
        end%%����ddPcd����������Ŀ�����ٶȣ�dwbd����������Ŀ���Ǽ��ٶȣ�rotM���������ĵ�ǰ��̬��contact������ǰ�ĸ������������ĽӴ�����
           %%feetPos2B������ǰ�����������ĵ��������˵���������������ϵ{s}�µ����꣬���зֱ�����P0��P1��P2��P3
        
        function A = calMatrixA(obj,feetPos2B, rotM, contact) %contact�а������ĸ������������ĽӴ�������1�����������Ӵ���0��ʾ�ڿ�
            for i = 1:4
                A(1:3,(3*(i-1)+1):3*i) = obj.I3;
                A(4:6,(3*(i-1)+1):3*i) = skew(feetPos2B(:,i) - rotM*obj.pcb);
            end
        end
        
        function Bd = calVectorBd(obj,ddPcd, dwbd, rotM)   %ʽ8.34
            Bd = zeros(6,1);
            %fprintf('ddpcd的维度: %f  %f\n',size(ddPcd,1),size(ddPcd,2));
            Bd(1:3) = obj.mass * (ddPcd - obj.g);
            Bd(4:6) = (rotM * obj.Ib * rotM' *dwbd);
        end
        
        function [CI,CE,ci0,ce0] = calConstraints(obj,contact)   %contact�������洢�������ĸ��ȵĴ�������
            contactLegNum = 0;
            for i = 1:4
                if contact(i) == 1
                    contactLegNum = contactLegNum + 1;  %�����м����ȴ���
                end
            end
            
            CI = zeros(5*contactLegNum, 12);
            ci0 = zeros(5*contactLegNum,1);
            CE = zeros(3*(4 - contactLegNum), 12);
            ce0 = zeros(3*(4 - contactLegNum),1);
            
%             CI = zeros(20, 12);
%             ci0 = zeros(20,1);
%             CE = zeros(12, 12);
%             ce0 = zeros(12,1);
                        
            ceID = 0;
            ciID = 0;
            for i = 1:4
                if contact(i) == 1
                    CI((5*ciID+1):(5*(ciID+1)),(3*(i-1)+1):3*i) = obj.fricMat;  %����ʽԼ��
                    ciID = ciID + 1;
                else
                    CE((3*ceID+1):(3*(ceID+1)),(3*(i-1)+1):3*i) = obj.I3;  %��ʽԼ��
                    ceID = ceID + 1;
                end
%              fprintf('CE的维度: %f  %f\nCI的维度: %f  %f',size(CE,1),size(CE,2),size(CI,1),size(CI,2));   
            end
        end
        
        function value = solveQP(obj,G,g0T,CI,CE,ci0,ce0)
            n = size(obj.Fprev,1);%��ȡ�Ժ��÷���ֵ�����������У��У���F��������
%             n = 12;
            m = size(ce0,1);
%             m = 12;
            p = size(ci0,1);
%             p = 20;
            
            G_ = zeros(n);
            CE_ = zeros(n,m);
            CI_ = zeros(n,p);
            g0 = zeros(n,1);
            ce0_ = zeros(m,1);
            ci0_ = zeros(p,1);
            x_ = zeros(n);
            
            for i = 1:n
                for j = 1:n
                    G_(i,j) = G(i,j);
                end
            end
            
            CE_tran = CE';
            %fprintf('CE_inv的维度: %f  %f\n',size(CE_inv,1),size(CE_inv,2));
            for i = 1:n
                for j = 1:m
                    CE_(i,j) = CE_tran(i,j);
                end
            end
            
            CI_tran = CI';
            fprintf('CI_inv的维度: %f  %f\n',size(CI,1),size(CI,2));
            for  i = 1:n
                for j = 1:p
                    CI_(i,j) = CI_tran(i,j);
                end
            end
            
            for i = 1:n
               g0(i) = g0T(i);
            
            end
                   
            for i = 1:m
               ce0_(i) = ce0(i);
            end
            
            for i = 1:p
              ci0_(i) = ci0(i);
            end
            value = quadprog((G_+G_')/2,g0',-CI_',ci0_,CE_',ce0_);  %%%%%******%%%%%%��������һ��  CE�ǵ�ʽԼ����CI�ǲ���ʽԼ��
        end
    end
end

