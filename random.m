m = round(rand(1,1)*4)+1; %�����������
n = round(rand(1,1)*4)+1; %�����������
c = round(rand(n,1)*20); %�������c
A = round(rand(m,n)*40)-20; %�������A
b = round(rand(m,1)*30)-15; %�������b
disp('c=');
disp(c);
disp('A=');
disp(A);
disp('b=');
disp(b);
[x,z0] = Simplex(c,A,b); %���Ž⣬����ֵ
disp('���Ž�Ϊ��');
disp(x');
disp('����ֵΪ��');
disp(z0);