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
[x,z0] = Simplex(c,A,b); %最优解，最优值
disp('最优解为：');
disp(x');
disp('最优值为：');
disp(z0);