% ������ʹ�ö�ż�����η�������Թ滮����
% min c'*x
% s.t. A*x<=b
% x>=0
% ��ż�����η���ⲽ�裺
% ��������һ����ʼ��ż���л�ָ��IB�ͷǻ�ָ��IN={1,...,n}\IB
% 1.��������α�T(B)����Ԫ��
% 2.����Ԫ��ָ��k��bt=min{bj|1<=j<=m}
% 3.���bk>=0�������B�Ļ��������һ�����Ž⣬z0Ϊ����ֵ���㷨��ֹ
% 4.���ykt>=0���������ż�滮���Ͻ磬ԭʼ��LP���޿��н⣬�㷨��ֹ
% 5.����ָ��t:a=rt/(-ykt)=min{ri/(-yki)|yki<0,i��IN}
% 6.��yktΪ��Ԫת�ᣬ���õ��µĵ����α���µ�ָ��IB��IN��ת������2

% c = [1;2;1]
% A = [-3,-1,-1;1,-4,-1]
% b = [-1;-2]

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

%����������Ϊ������
%function[x,z0]=Simplex(c,A,b)
[m,n]=size(A); %���A��������
x=zeros(n,1);
B = eye(m); %ȷ����ʼ���л�B
N = A; 
IB = [n+1:n+m]; %��ָ��
cB = zeros(m,1); %��������Ŀ�꺯���е�ϵ��
cN = c; %�ǻ�������Ŀ�꺯���е�ϵ��
xB = B\b;
xN = zeros(m,1);
flag = 0;

while 1
    T = B\N; %����T(B)Ԫ��    
    rN = (cN-T'*cB)';
    [xBmin, k] = min(xB); %ȷ����ָ��k�����ָ��
    if xBmin >= 0 %���bk>=0
       disp('�õ����Ž�'); %��ʱ�����Ž�
       break; %����ѭ��
    end
    yk=T(k,:); %ȡ����k��Ԫ��
    if min(yk)>=0 %��yk>=0����
        disp('��ż�滮���½磬ԭʼ��LP���޿��н�');
        x=zeros(n,1);
        z0=0;
        flag=1;
        break; %����ѭ��
    end;
    
    temp = find(yk<0); %�ҳ�yk������<0�ķ�����ָ��
    [a,u] = min(rN(temp)./(-yk(temp))); %ȷ����ָ��t������ָ�꣬aΪ����
    t = temp(u); %ȷ����ָ��t
    temp_col = B(:,k); %����B�ĵ�k����N��t���ཻ�����õ��µĻ�B
    B(:,k) = N(:,t);
    N(:,t) = temp_col;
    temp_value = cB(k); %��Ŀ�꺯���е�ϵ��Ҳ������Ӧ�Ľ��� 
    cB(k) = cN(t);   
    cN(t) = temp_value;
    IB(k) = t; %�ı��ָ��
    xB = B\b; %�����µĻ������н�xB,xN
    xN(t) = a;
end

%�������Ž⣬����ֵ
z0 = cB'*xB;
if flag~=1 %�����ڿ��н�
    temp = find(IB<=n); %���ݻ�ָ��ȷ�����Ž�
    x(IB(temp))= xB(temp)';
end
%��������ֵz0�����Ž�x
