t=[4 5;16 25.8;10 45;20 55;30 65;35 55;29 31;37 26;47 27;30 31.3;31 17;14 7;35.6 13.8;26.7 22.5;21 39;38 42;5 26;28 53;20 13;10 60;26 31;54 38;7 58;12 36;30 2] %24个点,第25个点事origin
save t.mat t
load t.mat
value=[1 1 1 2 3 2 1 3 3 2 2 2 2 2 1 2 3 3 1 1 2 1 1 1]; %24个目标的价值
value=value/100;
time=zeros(1,25); %侦察UAV时间数组，里面放的是飞机走的航程，除以速度便是时间，设速度为‘1’
attacktime=zeros(1,25); %打击UAV时间数组
%把侦察过的任务对应无人机走过的航程存到该任务的一个矩阵里，当做时间，然后打击任务如果选定某任务，check一下时间是否合格，合格的话，可以打击，并存入禁忌表，不合格的话，选次概率的

%注：目的是把所有目标执行完所有任务，所以每次迭代最后所有无人机收获的总价值都一样，都是所有目标的价值之和；所以本程序考虑优先执行价值大的目标，防止无人机飞很久、打很久后攻打效率变低
%的情况出现

%%计算城市间相互距离
n=size(t,1);
D=zeros(n,n);
for i=1:n
    for j=1:n
        if i~=j
            D(i,j)=sqrt(sum((t(i,:)-t(j,:)).^2));
        else
            D(i,j)=1e-2;
        end
    end
end
%%初始化参数
m=10;         %蚂蚁个数
alpha=1;      %信息素重要程度因子
beta=1;       %启发函数重要程度因子
gama=2;
rho=0.3;      %信息素挥发因子
Q=1.0;          %总量
eta=1./D;     %启发函数
tau=ones(n,n)+7.1192e-005;%信息素矩阵
iter=1;       %迭代次数初始值
iter_max=80;  %迭代次数最大值
length_best=zeros(iter_max,1);%每次迭代最佳路径长度(应该是一次比一次小)
length_ave=zeros(iter_max,1); %每次迭代路径平均长度 
%%迭代寻找最佳路径
while iter<=iter_max
    whta=cell(8,1);
    lieend=zeros(8,1);
    for zu=1:8
    city_index=1:25;       %城市来标号 
    table=[];
    start=zeros(4,1);
        temp=randperm(24);
        for i=1:4
        start(i)=temp(i);
        end
    table(:,1)=start;
    j=2;
 while (j<=30)
        for i=1:4
            if i==1 %UAV1只负责“侦察”任务
                if table(1,(j-1))~=25
                    table1=table(1,:);
                    table1=[table1;table(3:4,:)];
                    tabu1=table1(:); %UAV1的禁忌表出来了 %25如果也在tabu1里的话，那么
            allow_index1=~ismember(city_index,tabu1);  %【走过的变成0，能走的为1】【若tabu=(1 4)则allow_index=(0 1 1 0 1 1 1...)】【注意：allow_index与city_index同维】
            allow1=city_index(allow_index1);  %把还能走的序号摘出来了（待访问的城市集合）
            P1=allow1;
            %计算城市的转移概率
            if numel(allow1)~=0
              for k=1:max(size(allow1))-1
               P1(k)=(tau(table(1,(j-1)),allow1(k))^alpha)*(eta(table(1,(j-1)),allow1(k))^beta)*10000+7.1192e-004;
              end
            P1(max(size(allow1)))=7.1192e-005;
            P1=P1/sum(P1);
            [d1,ind1]=sort(P1,2,'descend');%从大到小排序是d1,对应的原序号是ind1
            target1=allow1(ind1(1));
            %轮盘赌法选择下一个城市
            %pc1=cumsum(P1);  % （p1 p1+p2 p1+p2+p3 p1+p2+p3+p4 ....）【p1<->allow(1)  p2<->allow(2) ...】
            %target_index1=find(pc1>=rand); 
            %target1=allow1(target_index1(1));  %这次返回的是allow数组中城市的真正序号
            table(1,j)=target1;  %把选好这个点放到路径表里面
            rr=D(25,table(1,1));
            time(table(1,1))=rr;
            if j>2
            for c=2:(j-1)
                rr=rr+D(table(1,c-1),table(1,c));
            end 
            end
            rrr=rr+D(table(1,j-1),target1);%rrr就是UAV1到该点时走过的航程
            time(target1)=rrr;
            else
                table(1,j)=25;
            end
                end
                 if table(1,(j-1))==25
                    table(1,j)=25;
                 end
            end          
            if i==2 %UAV2只负责“打击”任务
                if (table(2,(j-1))~=25)
                table(2,1)=table(1,1); %设定它第一次打击的是UAV1侦察过的目标
                ta2=table(1:(4*(j-1)+1)); %当前元素之前所有的元素
                tabu21=[];
                tabu22=[];
                tabu2=[];
                for y=1:24
                    if sum(ta2==y)==2
                        tabu21=[tabu21;y];
                    end
                end                       %出现过两次的放在tabu21里
                tabu22=setdiff(1:24,ta2); %一次都没出现的放在tabu22里
                tabu2=[tabu21',tabu22];   %tabu2出来了
            allow_index2=~ismember(city_index,tabu2);  %【走过的变成0，能走的为1】【若tabu=(1 4)则allow_index=(0 1 1 0 1 1 1...)】【注意：allow_index与city_index同维】
            allow2=city_index(allow_index2);  %把还能走的序号摘出来了（待访问的城市集合）
            P2=allow2;
            %计算城市的转移概率
           for k=1:(length(allow2)-1)
               P2(k)=tau(table(2,(j-1)),allow2(k))*eta(table(2,(j-1)),allow2(k))*value(allow2(k))*10000;
           end
           P2(max(size(allow2)))=7.1192e-005;
            P2=P2/sum(P2);
            [d2,ind2]=sort(P2,2,'descend');%从大到小排序是d1,对应的原序号是ind1
            target2=allow2(ind2(1)); %target2=d1(1);
            %轮盘赌法选择下一个城市
            %pc2=cumsum(P2);  % （p1 p1+p2 p1+p2+p3 p1+p2+p3+p4 ....）【p1<->allow(1)  p2<->allow(2) ...】
            %target_index2=find(pc2>=rand);  %选中那个概率较大的选中的点，返回的是allow数组中的序号
            %target2=allow2(target_index2(1));  %这次返回的是allow数组中城市的真正序号
            %table(2,j)=target2;  %把选好这个点放到路径表里面
            oo=D(25,table(2,1));
            attacktime(table(2,1))=oo;
            if j>2
            for c=2:(j-1)
                oo=oo+D(table(2,c-1),table(2,c));
            end 
            end
            ooo=oo+D(table(2,j-1),target2);%ooo就是UAV2到该点时走过的航程
            if numel(d2)>5
            u=2;
            while (ooo>time(target2)+20 & u<6)
                 target2=allow2(ind2(u));
                 ooo=oo+D(table(2,(j-1)),target2);
                 u=u+1;
            end
            end
            table(2,j)=target2;
            attacktime(target2)=ooo;
                end
                if table(2,(j-1))==25
                    table(2,j)=25;
                end
            end
            if i==3 %UAV3是“察打”任务
                if table(3,(j-1))~=25
                    ta3=table(1:(4*(j-1)+2));
                    tabu3=[];
                    tabu3c=[];
                for y=1:24
                    if sum(ta3==y)==2
                        tabu3=[tabu3;y];
                    end
                end    %出现两次的放在tabu3里   
                  for y=1:24
                    if sum(ta3==y)==1
                        tabu3c=[tabu3c;y];
                    end
                end %tabu3c是待打的任务，已侦查完的任务
            allow_index3=~ismember(city_index,tabu3);  %【走过的变成0，能走的为1】【若tabu=(1 4)则allow_index=(0 1 1 0 1 1 1...)】【注意：allow_index与city_index同维】
            allow3=city_index(allow_index3);  %把还能走的序号摘出来了（待访问的城市集合）
            P3=allow3;
            %计算城市的转移概率
           for k=1:(length(allow3)-1)
               %if ismember(allow3(k),tabu3c)==1     
               h=table(3,(j-1))
               P3(k)=(tau(table(3,j-1),allow3(k))^alpha)*(eta(table(3,(j-1)),allow3(k))^beta)*value(allow3(k))*10000+7.1192e-005;%这是要打的，需要价值
               %else
               %P3(k)=(tau(table(3,(j-1)),allow3(k))^alpha)*(eta(table(3,(j-1)),allow3(k))^beta)*100+7.1192e-005;%这些是待侦察的，没有价值
               %end
           end
            P3(max(size(allow3)))=7.1192e-009;
            P3=P3/sum(P3);
            [d3,ind3]=sort(P3,2,'descend');%从大到小排序是d1,对应的原序号是ind1
            target3=allow3(ind3(1));
            %轮盘赌法选择下一个城市
            %pc3=cumsum(P3);  % （p1 p1+p2 p1+p2+p3 p1+p2+p3+p4 ....）【p1<->allow(1)  p2<->allow(2) ...】
            %target_index3=find(pc3>=rand);  %选中那个概率较大的选中的点，返回的是allow数组中的序号
            %target3=allow3(target_index3(1));  %这次返回的是allow数组中城市的真正序号
            %table(3,j)=target3;  %把选好这个点放到路径表里面
            ww=D(25,table(3,1));
            time(table(3,1))=ww;
            if j>2
            for c=2:(j-1)
                ww=ww+D(table(3,c-1),table(3,c));
            end 
            end
            www=ww+D(table(3,j-1),target3);%www就是UAV3到该点时走过的航程
            if ismember(target3,tabu3c)==0 %侦察任务
                time(target3)=www;
                table(3,j)=target3;
            else %打击任务
                attacktime(target3)=www;
                if numel(d3)>5
                u=2;
            while (www>time(target3)+20 & u<6)
                 target3=allow3(ind3(u));
                 www=ww+D(table(3,(j-1)),target3);
                 u=u+1;
            end
                end
            attacktime(target3)=www;
            table(3,j)=target3;%www<time(target3)+10 说明此打击任务合理
            end 
                end
                if table(3,(j-1))==25
                    table(3,j)=25;
                end
            end
            if i==4 %UAV4是“察打”任务
                if table(4,(j-1))~=25
                    ta4=table(1:(4*(j-1)+3));
                    tabu4=[];
                    tabu4c=[];
                for y=1:24
                    if sum(ta4==y)==2
                        tabu4=[tabu4;y];
                    end
                end    %出现两次的放在tabu4里、、可以把已经侦察过的放在tabu4c中，（即出现过一次的），如果选到的是在tabu4'中的说明是要打击的然后算一下它的航程，再和侦察路径比较 
                for y=1:24
                    if sum(ta4==y)==1
                        tabu4c=[tabu4c;y];
                    end
                end 
            allow_index4=~ismember(city_index,tabu4);  %【走过的变成0，能走的为1】【若tabu=(1 4)则allow_index=(0 1 1 0 1 1 1...)】【注意：allow_index与city_index同维】
            allow4=city_index(allow_index4);  %把还能走的序号摘出来了（待访问的城市集合）
            P4=allow4;
            %计算城市的转移概率
           for k=1:(max(size(allow4))-1)
               %if ismember(allow4(k),tabu4c)==1
               sxx=table(4,(j-1))
               P4(k)=(tau(table(4,(j-1)),allow4(k))^alpha)*(eta(table(4,(j-1)),allow4(k))^beta)*value(allow4(k))*10000+7.1192e-005;
               %else
               %P4(k)=(tau(table(4,(j-1)),allow4(k))^alpha)*(eta(table(4,(j-1)),allow4(k))^beta)*100+7.1192e-005;
               %end
           end
           P4(max(size(allow4)))=7.1192e-009;
            P4=P4/sum(P4);
            [d4,ind4]=sort(P4,2,'descend');%从大到小排序是d1,对应的原序号是ind1
            target4=allow4(ind4(1));
            %轮盘赌法选择下一个城市
            %pc4=cumsum(P4);  % （p1 p1+p2 p1+p2+p3 p1+p2+p3+p4 ....）【p1<->allow(1)  p2<->allow(2) ...】
            %target_index4=find(pc4>=rand);  %选中那个概率较大的选中的点，返回的是allow数组中的序号
            %target4=allow4(target_index4(1));  %这次返回的是allow数组中城市的真正序号
            %table(4,j)=target4;  %把选好这个点放到路径表里面
            qq=D(25,table(4,1));
            time(table(4,1))=qq;
            if j>2
            for c=2:(j-1)
                qq=qq+D(table(4,c-1),table(4,c));
            end 
            end
            qqq=qq+D(table(4,j-1),target4);%www就是UAV3到该点时走过的航程
            if ismember(target4,tabu4c)==0 %侦察任务
                time(target4)=qqq;
                table(4,j)=target4;
            else %打击任务
                attacktime(target4)=qqq;
                if numel(d4)>5
                u=2;
            while (qqq>time(target4)+20 & u<6)
                 target4=allow4(ind4(u));
                 qqq=qq+D(table(4,j-1),target4);
                 u=u+1;
            end
                end
            attacktime(target4)=qqq;
            table(4,j)=target4;%www<time(target4)+10 说明此打击任务合理
            end                
                end
                if table(4,(j-1))==25
                    table(4,j)=25;
                end
              end
        end %一列结束
             if table(:,j)==[25;25;25;25]
                 jishu=0;
                 for i=1:24
                 if sum(table(:)'==i)==2
                     jishu=jishu+1;
                 end
                 end
                 if jishu==24
                     lieend1=j;
                     break;
                 else
                     j=j+1;
                     if j==31
                         break;
                     end
                 end
             else
                 j=j+1;
                 if j==31
                     break;
                 end
             end  
 end %table是4行lieend1列的矩阵
 lieend1=j;
 if lieend1==31
     lieend1=lieend1-1;
 end
 lieend(zu)=lieend1;
 disp(['lieend1=' num2str(lieend1)])
 wholetable1=table;
 s1=table;
 disp([wholetable1])
 %吧wholetable1总共4行，先每行进行2-opt之后，再更新信息素
 if lieend1<20
    for tt=1:4
           l1(tt)=0;
                    for e=1:lieend1-1
                        l1(tt)=l1(tt)+D(table(tt,e),table(tt,(e+1)));
                    end
                   % wholetable1(tt,:)=table(tt,:);
        for i=1:lieend1-1
            for j=1:lieend1-1
                if j>i
                    for k=0:fix((j-i)/2)
                        u=s1(tt,(i+k));
                        s1(tt,(i+k))=s1(tt,(j-k));
                        s1(tt,(j-k))=u;
                    end
                    l0=0;
                    for e=1:lieend1-1
                        l0=l0+D(s1(tt,e),s1(tt,(e+1)));
                    end %l0是每次之后每行的长度
                        if (s1(1,1)==25)||(s1(2,1)==25)||(s1(3,1)==25)||(s1(4,1)==25)
                            s1(tt,:)=table(tt,:);
                            break;
                        end
                    if l0<=l1(tt) %%？？这里好像有问题，应该是包括用价值的总指标比，而不只是用航程代价比
                        l1(tt)=l0;
                        wholetable1(tt,:)=s1(tt,:);
                    else
                        l1(tt)=l1(tt);
                        wholetable1(tt,:)=wholetable1(tt,:);
                    end
                    s1(tt,:)=table(tt,:);
                end
            end
        end
    end  %wholetable1（tt,:）里面放的都是改过的最短的路径，共4行；l1（tt）放的是改过后最短的长度，共4行
 end
     whta{zu}=wholetable1;
    end %whta里面放了8个wholetable，分别是whta{1} whta{2}。。。第一个wholetable有lieend（1）列，第二个有lieend（2）列。。。
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%总共8个小分队
    %%开始计算路径啦
    
     Length=zeros(8,1);
     for i=1:8
     Length(i)=D(25,whta{i}(1,1))+D(25,whta{i}(2,1))+D(25,whta{i}(3,1))+D(25,whta{i}(4,1));
     end
     for i=1:8
         for j=1:4
             for k=1:(lieend(i)-1)
                 Length(i)=Length(i)+D(whta{i}(j,k),whta{i}(j,k+1));
             end
         end
     end %Length是一个8行的纵矩阵 放着每个小wholetable总路径长度
 J=zeros(8,3);
 for i=1:8
     for j=2:4 %因为1是侦察无人机所以只考虑2:4
         for k=1:12 %只考虑前12个，因为分配方案每个蚂蚁一般也不过12个任务左右
             if whta{i}(j,k+1)~=whta{i}(j,k);
                 J(i,j-1)=J(i,j-1)+(13-k)*value(whta{i}(j,k))*100; %价值大的如果它优先执行，则对应的执行序号k小，则13-k就大，所乘以的权重就大，得到的总指标J就大
             end
         end
     end
 end %J每一行的3列分别装的是每个wholetable打击任务的那三行，每一行的价值和
 zb=Length'*0.7-sum(J')*0.3; %zhibiao这个矩阵是价值+航迹的总指标； 是8个数的行矩阵； %%由于length这个指标是越小越好，而J'价值这个指标是越大越好，且J'都小于length，所以这里用减号
 
    %%计算最小指标
    if iter==1
        route_best=[];
        [up,upindex]=sort(zb); %up矩阵是从小到大排列好的矩阵，upindex矩阵是他们在原矩阵中的标号；upindex(1)是最短的wholetable(x)对应的x
        i=1;
        while(i<=8)
        route_best=whta{upindex(i)};
        if size(route_best,2)==30
            i=i+1;
            route_best=[];
        else
            break;
        end
        end
        length_best(iter)=up(i);
    else
        route_best_last=route_best; %把上一次迭代的route_best存起来，以免这一次用
        route_best1=[]; %每次迭代最佳路径  % n:25
        route_best=[];
        [up1,upindex1]=sort(zb);
        i=1;
        while(i<=8)
        route_best1=whta{upindex1(i)};
        if size(route_best1,2)==30
            i=i+1;
            route_best1=[];
        else
            break;
        end
        end
        if up1(i)<=length_best(iter-1)
            route_best=route_best1;
            length_best(iter)=up1(i);
        else
            route_best=route_best_last;
            length_best(iter)=length_best(iter-1);
        end
    end
%%更新信息素
%%更新信息素
delta_tau=zeros(25,25)+7.1192e-004;
for i=1:8
    for j=1:4
        delta_tau(25,whta{i}(j,1))=Q/zb(i)';%更新信息素是性能指标最小呢还是航迹最小？
    end
end
for i=1:8
    for j=1:4
        for k=1:(lieend(i)-1)
            delta_tau(whta{i}(j,k),whta{i}(j,k+1))=delta_tau(whta{i}(j,k),whta{i}(j,k+1))+Q/zb(i)';
        end
    end
end
            
delta_tau=10*delta_tau
tau=(1-rho)*tau+delta_tau
iter=iter+1;
end
%%结果显示
[shortest_length,index]=min(length_best);
shortest_route=route_best; %shortest_route 是一个4行的矩阵

disp([ num2str(shortest_route)])
disp(['最短距离：', num2str(shortest_length)])
shortest_route_1=shortest_route(1,:);
shortest_route_2=shortest_route(2,:);
shortest_route_3=shortest_route(3,:);
shortest_route_4=shortest_route(4,:);

plot([t(25,1);t(shortest_route_1,1);],...
    [t(25,2);t(shortest_route_1,2);],'ro:','LineWidth',1.8);
hold on;
plot([t(25,1);t(shortest_route_2,1);],...
    [t(25,2);t(shortest_route_2,2);],'gp-.','LineWidth',1.8);
hold on;
plot([t(25,1);t(shortest_route_3,1);],...
    [t(25,2);t(shortest_route_3,2);],'cd-','LineWidth',1.8);
hold on;
plot([t(25,1);t(shortest_route_4,1);],...
    [t(25,2);t(shortest_route_4,2);],'m*--','LineWidth',1.8);
hold off;
figure(2)
plot(1:iter_max,length_best,'k')
axis([0,200,100,700])
%legend('最短距离')
xlabel('迭代次数')
ylabel('代价指标')
%title('各代最短航程与平均航程比较')
%%下面计算4个UAV分别走的长度
len1=D(shortest_route_1(1),25);
for s=1:length(shortest_route_1)-1
disp([num2str(len1)])
len1=len1+D(shortest_route_1(s),shortest_route_1(s+1));
end
disp(['UAV 1航程长度', num2str(len1)])
len2=D(shortest_route_2(1),25);
for s=1:length(shortest_route_2)-1
disp([num2str(len2)])
len2=len2+D(shortest_route_2(s),shortest_route_2(s+1));
end
disp(['UAV 2航程长度', num2str(len2)])
len3=D(shortest_route_3(1),25);
for s=1:length(shortest_route_3)-1
disp([num2str(len3)])
len3=len3+D(shortest_route_3(s),shortest_route_3(s+1));
end
disp(['UAV 3航程长度', num2str(len3)])
len4=D(shortest_route_4(1),25);
for s=1:length(shortest_route_4)-1
disp([num2str(len4)])
len4=len4+D(shortest_route_4(s),shortest_route_4(s+1));
end
disp(['UAV 4航程长度', num2str(len4)])
s2=0;
for k=1:12
    s2=s2-value(shortest_route_2(k))*(13-k);
end
s3=0;
for k=1:2:12
    s3=s3-value(shortest_route_3(k))*(13-k);
end
s4=0;
for k=1:2:12
    s4=s4-value(shortest_route_4(k))*(13-k);
end
s=shortest_length/100*0.7+(s2+s3+s4)*0.3