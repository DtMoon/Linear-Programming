% 本程序使用对偶单纯形法求解线性规划问题
% min c'*x
% s.t. A*x<=b
% x>=0
% 对偶单纯形法求解步骤：
% 假设已有一个初始对偶可行基指标IB和非基指标IN={1,...,n}\IB
% 1.求出单纯形表T(B)所有元素
% 2.定主元行指标k：bt=min{bj|1<=j<=m}
% 3.如果bk>=0，则关于B的基本解就是一个最优解，z0为最优值，算法中止
% 4.如果ykt>=0成立，则对偶规划无上界，原始（LP）无可行解，算法中止
% 5.定列指标t:a=rt/(-ykt)=min{ri/(-yki)|yki<0,i∈IN}
% 6.以ykt为主元转轴，，得到新的单纯形表和新的指标IB和IN，转至步骤2

% c = [1;2;1]
% A = [-3,-1,-1;1,-4,-1]
% b = [-1;-2]

m = round(rand(1,1)*4)+1; %随机生成行数
n = round(rand(1,1)*4)+1; %随机生成列数
c = round(rand(n,1)*20); %随机生成c
A = round(rand(m,n)*40)-20; %随机生成A
b = round(rand(m,1)*30)-15; %随机生成b
disp('c=');
disp(c);
disp('A=');
disp(A);
disp('b=');
disp(b);

%以下向量均为列向量
%function[x,z0]=Simplex(c,A,b)
[m,n]=size(A); %求得A的行列数
x=zeros(n,1);
B = eye(m); %确定初始可行基B
N = A; 
IB = [n+1:n+m]; %基指标
cB = zeros(m,1); %基变量在目标函数中的系数
cN = c; %非基变量在目标函数中的系数
xB = B\b;
xN = zeros(m,1);
flag = 0;

while 1
    T = B\N; %计算T(B)元素    
    rN = (cN-T'*cB)';
    [xBmin, k] = min(xB); %确定行指标k，离基指标
    if xBmin >= 0 %如果bk>=0
       disp('得到最优解'); %此时有最优解
       break; %跳出循环
    end
    yk=T(k,:); %取出第k行元素
    if min(yk)>=0 %若yk>=0成立
        disp('对偶规划无下界，原始（LP）无可行解');
        x=zeros(n,1);
        z0=0;
        flag=1;
        break; %跳出循环
    end;
    
    temp = find(yk<0); %找出yk中所有<0的分量的指标
    [a,u] = min(rN(temp)./(-yk(temp))); %确定列指标t，进基指标，a为步长
    t = temp(u); %确定列指标t
    temp_col = B(:,k); %将基B的第k列与N的t列相交换，得到新的基B
    B(:,k) = N(:,t);
    N(:,t) = temp_col;
    temp_value = cB(k); %将目标函数中的系数也进行相应的交换 
    cB(k) = cN(t);   
    cN(t) = temp_value;
    IB(k) = t; %改变基指标
    xB = B\b; %计算新的基本可行解xB,xN
    xN(t) = a;
end

%计算最优解，最优值
z0 = cB'*xB;
if flag~=1 %若存在可行解
    temp = find(IB<=n); %根据基指标确定最优解
    x(IB(temp))= xB(temp)';
end
%返回最优值z0和最优解x
