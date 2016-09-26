t=[4 5;16 25.8;10 45;20 55;30 65;35 55;29 31;37 26;47 27;50 51.3;51 17;14 7;40 10;26.7 22.5;21 39;38 42;5 26;28 53;20 13;10 60;26 41;54 38;7 58;12 36;30 2] %24个点,第25个点事origin
save t.mat t
load t.mat
%%计算城市间相互距离
%n=size(t,1);
n=6;
D=zeros(25,25);
for i=1:25
    for j=1:25
        if i~=j
            D(i,j)=sqrt(sum((t(i,:)-t(j,:)).^2));
        else
            D(i,j)=1e-4;
        end
    end
end
%%初始化参数
m=4;         %蚂蚁个数 【原来是1只蚂蚁要走过24个点，现在是4只蚂蚁总共走过24个点】【5组蚂蚁，每组里面4只，每只走过6个点】
alpha=1;      %信息素重要程度因子
beta=5;       %启发函数重要程度因子
rho=0.1;      %信息素挥发因子
Q=50;          %总量
eta=1./D;     %启发函数
tau=ones(25,25);%信息素矩阵
table=zeros(m,n);%路径记录表
iter=1;       %迭代次数初始值
iter_max=400;  %迭代次数最大值
route_best=zeros((4*iter_max),n); %每次迭代最佳路径
length_best=zeros(iter_max,1);%每次迭代最佳路径长度(应该是一次比一次小)
length_ave=zeros(iter_max,1); %每次迭代路径平均长度

%%迭代寻找最佳路径
while iter<=iter_max    
    city_index=1:24;       %城市来标号
    wholetable=[];
    s=1;
    while s<=10
          start=zeros(4,1);
          temp=randperm(24);
            for i=1:4
             start(i)=temp(i);
            end
          table(:,1)=start;
    for j=2:n
        for i=1:m
            tabu=table(1:((j-1)*4+i-1));  %把第i只蚂蚁之前所走过的所有城市放入禁忌表中 【把table里面从第一个数到j-1全放到禁忌表中】
            allow_index=~ismember(city_index,tabu);  %【走过的变成0，能走的为1】【若tabu=(1 4)则allow_index=(0 1 1 0 1 1 1...)】【注意：allow_index与city_index同维】
            allow=city_index(allow_index);  %把还能走的序号摘出来了（待访问的城市集合）
            P=allow;
            %计算城市的转移概率
            for k=1:max(size(allow))
                P(k)=(tau(tabu(end-3),allow(k))^alpha)*(eta(tabu(end-3),allow(k))^beta);
            end
            P=P/sum(P);
            %轮盘赌法选择下一个城市
            pc=cumsum(P);  % （p1 p1+p2 p1+p2+p3 p1+p2+p3+p4 ....）【p1<->allow(1)  p2<->allow(2) ...】
            target_index=find(pc>=rand);  %选中那个概率较大的选中的点，返回的是allow数组中的序号
            target=allow(target_index(1));  %这次返回的是allow数组中城市的真正序号
            table(i,j)=target;  %把选好这个点放到路径表里面
        end
    end
    wholetable=[wholetable;table]; 
    table=zeros(m,n); %下面要做的是把table表清零然后重新走
    s=s+1;                %第5次完了以后table里第一列有数，所以下次iter循环的时候应该清零，其实不清零也行，后面有再给table第一列覆盖数
    end
    %吧wholetable总共40行，先每行进行2-opt之后，再更新信息素
     s1=wholetable;
     wholetable2=wholetable;
    for tt=1:40
       % s1(tt,:)=wholetable(tt,:);
           l1(tt)=0;
                    for e=1:5
                        l1(tt)=l1(tt)+D(wholetable(tt,e),wholetable(tt,(e+1)));
                    end
                    l1(tt)=l1(tt)+D(wholetable(tt,1),25)+D(wholetable(tt,6),25); %l1是2opt之前每行的长度
                   % wholetable2(tt,:)=wholetable(tt,:);
        for i=1:5
            for j=1:6
                if j>i
                    for k=0:fix((j-i)/2)
                        u=s1(tt,(i+k));
                        s1(tt,(i+k))=s1(tt,(j-k));
                        s1(tt,(j-k))=u;
                    end
                    l0=0;
                    for e=1:5
                        l0=l0+D(s1(tt,e),s1(tt,(e+1)));
                    end
                    l0=l0+D(s1(tt,1),25)+D(s1(tt,6),25); %l0是每次之后每行的长度
                    if l0<=l1(tt)
                        l1(tt)=l0;
                        wholetable2(tt,:)=s1(tt,:);
                    else
                        l1(tt)=l1(tt);
                        wholetable2(tt,:)=wholetable2(tt,:);
                    end
                    s1(tt,:)=wholetable(tt,:);
                end
            end
        end
    end  %wholetable2（tt,:）里面放的都是改过的最短的路径，共40行；l1（tt）放的是改过后最短的长度，共40行
    %%开始计算路径啦
    Length_=[];
    h=0;
    while h<=37
    Length1=zeros(4,1);
    for i=1:4
        route1=wholetable2((i+h),:);  %【把第i只蚂蚁走过的路径给route】【route里就是m只蚂蚁每只走过的序列了】
        for j=1:5
            Length1(i)=Length1(i)+D(route1(j),route1(j+1));
        end
        Length1(i)=Length1(i)+D(route1(6),25)+D(route1(1),25);
    end
    length1=0;
    for r=1:4
        length1=length1+Length1(r);
    end
     Length_=[Length_;length1];  %Length_是一个10行的纵矩阵
     h=h+4;
    end
    %%计算最短路径及平均距离
    if iter==1
        [min_length,min_index]=min(Length_);
        length_best(iter)=min_length;
        length_ave(iter)=mean(Length_);
        route_best(1:4,:)=wholetable2((4*min_index-3):(4*min_index),:); %route_best(1:4,:)是本次10只蚂蚁中总和最短的4条路径
    else
        [min_length,min_index]=min(Length_);
        length_best(iter)=min(min_length,length_best(iter-1));
        length_ave(iter)=mean(Length_);
        if length_best(iter)==min_length
          route_best((4*iter-3):(4*iter),:)=wholetable2((4*min_index-3):(4*min_index),:);
        else
            route_best((4*iter-3):(4*iter),:)=route_best((4*iter-7):(4*iter-4),:);
        end
    end  
  %%更新信息素
delta_tau=zeros(25,25);
for i=1:40
    for j=1:5
        delta_tau(wholetable2(i,j),wholetable2(i,j+1))=delta_tau(wholetable2(i,j),wholetable2(i,j+1))+Q/l1(i);
    end
    delta_tau(wholetable2(i,6),25)=delta_tau(wholetable2(i,6),25)+Q/l1(i);
    delta_tau(25,wholetable2(i,6))=delta_tau(25,wholetable2(i,1))+Q/l1(i);
end
tau=(1-rho)*tau+delta_tau;
iter=iter+1;
table=zeros(m,n);
end
    %%结果显示
[shortest_length,index]=min(length_best);
shortest_route=route_best((4*index-3):(4*index),:);  %shortest_route 是一个4行的矩阵
shortest_route_1=shortest_route(1,:);
shortest_route_2=shortest_route(2,:);
shortest_route_3=shortest_route(3,:);
shortest_route_4=shortest_route(4,:);
disp(['最短距离：', num2str(shortest_length)])
disp(['最短路径：', num2str(shortest_route_1),'      ',num2str(shortest_route_2),'     ',num2str(shortest_route_3),'      ',num2str(shortest_route_4)])
%%下面计算4个UAV分别走的长度
len1=0;
for s=1:5
len1=len1+D(shortest_route_1(s),shortest_route_1(s+1));
end
len1=len1+D(shortest_route_1(1),25)+D(shortest_route_1(6),25);
disp(['UAV 1航程长度', num2str(len1)])
len2=0;
for s=1:5
len2=len2+D(shortest_route_2(s),shortest_route_2(s+1));
end
len2=len2+D(shortest_route_2(1),25)+D(shortest_route_2(6),25);
disp(['UAV 2航程长度', num2str(len2)])
len3=0;
for s=1:5
len3=len3+D(shortest_route_3(s),shortest_route_3(s+1));
end
len3=len3+D(shortest_route_3(1),25)+D(shortest_route_3(6),25);
disp(['UAV 3航程长度', num2str(len3)])
len4=0;
for s=1:5
len4=len4+D(shortest_route_4(s),shortest_route_4(s+1));
end
len4=len4+D(shortest_route_4(1),25)+D(shortest_route_4(6),25);
disp(['UAV 4航程长度', num2str(len4)])
    
%%绘图
figure(1)
%plot([t(25,1);t(shortest_route_1,1);t(25,1);t(shortest_route_2,1);t(25,1);t(shortest_route_3,1);t(25,1);t(shortest_route_4,1);t(25,1)],...
%    [t(25,2);t(shortest_route_1,2);t(25,2);t(shortest_route_2,2);t(25,2);t(shortest_route_3,2);t(25,2);t(shortest_route_4,2);t(25,2)],'o-');
plot([t(25,1);t(shortest_route_1(1),1);t(shortest_route_1(2),1);t(shortest_route_1(3),1);t(shortest_route_1(4),1);t(shortest_route_1(5),1);t(shortest_route_1(6),1);t(25,1)],...
    [t(25,2);t(shortest_route_1(1),2);t(shortest_route_1(2),2);t(shortest_route_1(3),2);t(shortest_route_1(4),2);t(shortest_route_1(5),2);t(shortest_route_1(6),2);t(25,2)],'ro:','LineWidth',1.8);
hold on;
plot([t(25,1);t(shortest_route_2(1),1);t(shortest_route_2(2),1);t(shortest_route_2(3),1);t(shortest_route_2(4),1);t(shortest_route_2(5),1);t(shortest_route_2(6),1);t(25,1)],...
    [t(25,2);t(shortest_route_2(1),2);t(shortest_route_2(2),2);t(shortest_route_2(3),2);t(shortest_route_2(4),2);t(shortest_route_2(5),2);t(shortest_route_2(6),2);t(25,2)],'gp-.','LineWidth',1.8);
hold on;
plot([t(25,1);t(shortest_route_3(1),1);t(shortest_route_3(2),1);t(shortest_route_3(3),1);t(shortest_route_3(4),1);t(shortest_route_3(5),1);t(shortest_route_3(6),1);t(25,1)],...
    [t(25,2);t(shortest_route_3(1),2);t(shortest_route_3(2),2);t(shortest_route_3(3),2);t(shortest_route_3(4),2);t(shortest_route_3(5),2);t(shortest_route_3(6),2);t(25,2)],'cd-','LineWidth',1.8);
hold on;
plot([t(25,1);t(shortest_route_4(1),1);t(shortest_route_4(2),1);t(shortest_route_4(3),1);t(shortest_route_4(4),1);t(shortest_route_4(5),1);t(shortest_route_4(6),1);t(25,1)],...
    [t(25,2);t(shortest_route_4(1),2);t(shortest_route_4(2),2);t(shortest_route_4(3),2);t(shortest_route_4(4),2);t(shortest_route_4(5),2);t(shortest_route_4(6),2);t(25,2)],'m*--','LineWidth',1.8);
hold off;
for i=1:24
    text(t(i,1),t(i,2),['  T' num2str(i)]);
end
text(5,15,['\leftarrowUAV 1'])
text(1,55,['UAV 2\rightarrow'])
text(35,18,['\leftarrowUAV 3'])
text(36,6,['\leftarrowUAV 4'])
grid on
%text(t(shortest_route(1),1),t(shortest_route(2),2),'   起点');
%text(t(shortest_route(end),1),t(shortest_route(end),2),'    终点');
xlabel('城市位置横坐标 X(km)')
ylabel('城市位置纵坐标 Y(km)')
title(['多组群蚁群算法优化2-opt改进路径(最短距离：',num2str(shortest_length),')'])
figure(2)
plot(1:iter_max,length_best,'b',1:iter_max,length_ave,'r')
axis([0,400,460,650])
legend('最短距离','平均距离')
xlabel('迭代次数')
ylabel('距离')
title('各代最短距离与平均距离对比')
for i=1:24
    text(t(i,1),t(i,2),['  T' num2str(i)]);
end
hold off;