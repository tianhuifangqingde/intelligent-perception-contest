function [w1,w2,b1,b2]=a(input,output)
%随机提取4500个样本为训练样本，500个样本为预测样本
input_train=input;
output_train=output;
 
 
%% 网络结构初始化
innum=25;
midnum=25;
outnum=10;
 
%权值初始化
w1=rands(midnum,innum);%输入到隐藏
b1=rands(midnum,1);
w2=rands(midnum,outnum);%隐藏到输出
b2=rands(outnum,1);
 
%学习率
xite=0.1;
alfa=0.01;
 
%% 网络训练
   for h=1:100
    for i=1:1:length(input)
       %% 网络预测输出
        x=input_train(:,i);
        % 隐含层输出
        for j=1:1:midnum
            I(j)=input_train(:,i)'*w1(j,:)'+b1(j);
            Iout(j)=1/(1+exp(double(-I(j))));
        end
        % 输出层输出
        yn=w2'*Iout'+b2;
       
       %% 权值阀值修正
        %计算误差
        e=output_train(:,i)-yn;    
       
        %计算权值变化率
        dw2=e*Iout;
        db2=e';
        %=======由于采用的是sigmoid单元，所以要对每个输出单元以及隐藏单元计算误差项======%
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