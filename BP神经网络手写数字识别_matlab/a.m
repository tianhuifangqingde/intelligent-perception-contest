function [w1,w2,b1,b2]=a(input,output)
%�����ȡ4500������Ϊѵ��������500������ΪԤ������
input_train=input;
output_train=output;
 
 
%% ����ṹ��ʼ��
innum=25;
midnum=25;
outnum=10;
 
%Ȩֵ��ʼ��
w1=rands(midnum,innum);%���뵽����
b1=rands(midnum,1);
w2=rands(midnum,outnum);%���ص����
b2=rands(outnum,1);
 
%ѧϰ��
xite=0.1;
alfa=0.01;
 
%% ����ѵ��
   for h=1:100
    for i=1:1:length(input)
       %% ����Ԥ�����
        x=input_train(:,i);
        % ���������
        for j=1:1:midnum
            I(j)=input_train(:,i)'*w1(j,:)'+b1(j);
            Iout(j)=1/(1+exp(double(-I(j))));
        end
        % ��������
        yn=w2'*Iout'+b2;
       
       %% Ȩֵ��ֵ����
        %�������
        e=output_train(:,i)-yn;    
       
        %����Ȩֵ�仯��
        dw2=e*Iout;
        db2=e';
        %=======���ڲ��õ���sigmoid��Ԫ������Ҫ��ÿ�������Ԫ�Լ����ص�Ԫ���������======%
        for j=1:1:midnum
            S=1/(1+exp(double(-I(j))));
            FI(j)=S*(1-S);
        end     
        for k=1:1:innum
            for j=1:1:midnum
               dw1(k,j)=FI(j)*x(k)*(w2(j,:)*e);%
                db1(j)=FI(j)*(w2(j,:)*e);
            end
        end
          
        w1=w1+xite*dw1';
        b1=b1+xite*db1';
        w2=w2+xite*dw2';
        b2=b2+xite*db2';
       
    end
    h=h+1
    end
end