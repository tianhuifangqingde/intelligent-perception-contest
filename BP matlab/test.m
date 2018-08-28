clc;
clear;

w1c=load('w1.mat');
w1=w1c.w1;
w2c=load('w2.mat');
w2=w2c.w2;
b1c=load('b1.mat');
b1=b1c.b1;
b2c=load('b2.mat');
b2=b2c.b2;
image_featurec=load('image_feature.mat');
image_feature=image_featurec.image_feature;
%% ²âÊÔ
test_data(:,1)= image_feature(:,990);
for m=1:1:25
hidden(m)=w1(m,:)*test_data(:,:)+b1(m);
hiddenout(m)=1/(1+exp(-hidden(m)));
end
% Êä³ö²ãÊä³ö
out=(hiddenout*w2')'+b2;
out1=find(out==max(out));


% %%È«²¿²âÊÔ 
% test_data(:,1:1100)= image_feature(:,1:1100);
% b=0;
% for i=1:size(test_data,2)
%     for m=1:1:25
%     hidden(m)=w1(m,:)*test_data(:,i)+b1(m);
%     hiddenout(m)=1/(1+exp(-hidden(m)));
%     end
%     % Êä³ö²ãÊä³ö
%     out=(hiddenout*w2')'+b2;
%     out1(i)=find(out==max(out));
%     if out1(i)==floor(i/100+1)
%         b=b+1;
%     end
% end 
% accuary=b/1100








