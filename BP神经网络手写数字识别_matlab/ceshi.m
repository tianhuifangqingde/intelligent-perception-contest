clc;
clear;

image=imread('F:/图像处理/BP神经网络手写数字识别_matlab1/BP神经网络手写数字识别_matlab/data3/1/1 (8).jpg');
image_resize=imresize(image ,[50 50]);%将图片大小调整为50*50
image_two_value=im2bw(image_resize ,200/255); %自动调整阈值并二值化

%并对每个10*10的区域进行灰度合并
for j=1:5
    for k=1:5
        image_feature1(1,(j-1)*5+k)=sum(sum(image_two_value(((j*10-9):(j*10)),((k*10-9):(k*10)))));
    end
end
image_feature(:,1)=(500.0-image_feature1)/500.0;
 

w1c=load('w1.mat');
w1=w1c.w1;
w2c=load('w2.mat');
w2=w2c.w2;
b1c=load('b1.mat');
b1=b1c.b1;
b2c=load('b2.mat');
b2=b2c.b2;

test_data(:,1)= image_feature;
hidden =w1(:,:)*test_data(:,1)+b1 ;
for m=1:1:25
    hiddenout(m)=1/(1+exp(-hidden(m)));
end
% 输出层输出
out=(hiddenout*w2')'+b2;
out1=find(out==max(out))

%