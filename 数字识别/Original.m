clc;
clear;
%% 读取data文件夹下的所有图片
image_number=0;%图片个数
image=cell(0);%使用cell存储图片
root1='./data';
First_order_list=dir(root1);%读取data文件夹内的列表
First_order_list_number=length(First_order_list);%计算列表1内的文件个数
for i=1:First_order_list_number
    if strcmp(First_order_list(i).name,'.')||strcmp(First_order_list(i).name,'..')
    else
        root2=strcat(root1,'/',First_order_list(i).name);%列表2的路径
        First_second_list=dir(root2);%列表2
        First_second_list_number=length(First_second_list);
        for j=1:First_second_list_number
            if strcmp(First_second_list(j).name,'.')||strcmp(First_second_list(j).name,'..')|| strcmp(First_second_list(j).name,'Desktop_1.ini')|| strcmp(First_second_list(j).name,'Desktop_2.ini')
            else
                image_number=image_number+1;%数据集内的图像个数
                image{image_number}=imread(strcat(root2,'/',First_second_list(j).name));
            end
        end
    end
end   
%% 对数据集内的图像进行特征提取
for h=1:11
    for i=1:100
        path='F:/图像处理/xunlian/';
        path1=[path,int2str(h), '/1 (', int2str(i) ,').png'];
        image=imread(path1);
        image_resize=imresize(image ,[50 50]);%将图片大小调整为50*50
        image_two_value=im2bw(image_resize ,200/255); %自动调整阈值并二值化
        
        %并对每个10*10的区域进行灰度合并
        for j=1:5
            for k=1:5
                image_feature1(1,(j-1)*5+k)=sum(sum(image_two_value(((j*10-9):(j*10)),((k*10-9):(k*10)))));
            end
        end
        image_feature(:,(h-1)*100+i)=(100-image_feature1)/100;
    end
end
% 输入 输出 
k=rand(1,1100);  
[m,n]=sort(k);  
trian_image=image_feature(:,n(1:1100)); %输入
trian_out1=zeros(11,1100);
for i=1:1100
    trian_out1(ceil(i/100),i)=1;%输出
end
train_out=trian_out1(:,n(1:1100));

%% BP神经网络创建，训练
[w1,w2,b1,b2]=a(trian_image,train_out);
createxml('w1',w1);
w2=w2';
createxml('w2',w2);
createxml('b1',b1);
createxml('b2',b2);
save('w1.mat','w1');
save('w2.mat','w2');
save('b1.mat','b1');
save('b2.mat','b2');

% w1c=load('w1.mat');
% w1=w1c.w1;
% w2c=load('w2.mat');
% w2=w2c.w2;
% b1c=load('b1.mat');
% b1=b1c.b1;
% b2c=load('b2.mat');
% b2=b2c.b2;
% image_featurec=load('image_feature.mat');
% image_feature=image_featurec.image_feature;
% %% 测试
% test_data(:,1)= image_feature(:,66);
% b=0;
% for i=1:size(test_data,2)
%     for m=1:1:25
%     hidden(m)=w1(m,:)*test_data(:,i)+b1(m);
%     hiddenout(m)=1/(1+exp(-hidden(m)));
%     end
%     % 输出层输出
%     out=(hiddenout*w2)'+b2;
%     out1(i)=find(out==max(out));
%     if out1(i)==floor(i/60+1)
%         b=b+1;
%     end
% end 
% accuary=b/660



 
% test_data(:,1)= image_feature(:,181);
% b=0;
% 
%      hidden =w1(:,:)*test_data(:,1)+b1 ;
%     for m=1:1:25
%         hiddenout(m)=1/(1+exp(-hidden(m)));
%     end
%     % 输出层输出
%     out=(hiddenout*w2')'+b2;
%     out1=find(out==max(out))


test_data(:,1:1100)= image_feature(:,1:1100);
b=0;
for i=1:size(test_data,2)
    for m=1:1:25
    hidden(m)=w1(m,:)*test_data(:,i)+b1(m);
    hiddenout(m)=1/(1+exp(-hidden(m)));
    end
    % 输出层输出
    out=(hiddenout*w2')'+b2;
    out1(i)=find(out==max(out));
    if out1(i)==floor(i/100+1)
        b=b+1;
    end
end 
accuary=b/1100








