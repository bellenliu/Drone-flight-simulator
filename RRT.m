function [path, num_expanded] = RRT(map, start, goal, fang)
%compare                 参数舍去
%start_node_ind         起始点
%goal_node_ind          终点
%node                   点集合
%nodes                  路径点信息
%prand                  随机点 判断是否在碰撞
%fangxinag              初始化参数
%idx                    离随机点最近的编号
%q                      表示访问过的点
%fang                   %方向特征向量  flight direction characteristic parameters
                        %用num_expanded将最后时刻的速度方向保存下来，用于前往下一个目标点的规划


 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    初始化信息  点集个数，xyz个数，对应坐标，生成node存储点
 xmin=map(1,1);  ymin=map(1,2);  zmin=map(1,3);%利用地图数组加载地图信息 
xmax=map(1,4);  ymax=map(1,5);  zmax=map(1,6);
xy_res=map(end,1);  z_res=map(end,2);
start_node_ind=[];  start_node_dist=inf;
goal_node_ind=[];   goal_node_dist=inf;
path=[];
num_expanded=0;
%accounting for large resolution numbers
if(xy_res>min(xmax,ymax))
    xy_res=min(xmax,ymax);
end

if(z_res>zmax)
    z_res=zmax;
end

X=xmin:xy_res:xmax; %地图基本信息
if (round(X(end),2)~=round(xmax,2))     %generating X, Y and Z. 
   X=[X,xmax];                          %Also if xmax is not part of X, then include xmax as well.    
end

Y=ymin:xy_res:ymax;
if (round(Y(end),2)~=round(ymax,2))
    Y=[Y,ymax]; 
end

Z=zmin:z_res:zmax;%将xyz按步长分成一维数组
if (round(Z(end),2)~=round(zmax,2))
    Z=[Z,zmax];
end

nx=numel(X); ny=numel(Y); nz=numel(Z);%元素个数
totnodes=nx*ny*nz;%总个数

node=zeros(totnodes,4);%n行 4列生成所有点的矩阵
nodeind=0;
for yloop=1:numel(Y)%循环搜索所有的点
     for zloop=1:numel(Z)
         for xloop=1:numel(X)
             
             nodeind=nodeind+1;
             
             node(nodeind,1:3)=[X(xloop),Y(yloop),Z(zloop)];%第nodeind行前三位数为xyz
             
             
             %checking if node is in block
             
             if (collide(map,[X(xloop),Y(yloop),Z(zloop)]))%collide函数使用map
                 node(nodeind,4)=0;%第四位判断是否在障碍物里
             else
                 node(nodeind,4)=1;
             end
             
             %search for closest start and goal node indices
             if (sqrt((X(xloop)-start(1))^2+(Y(yloop)-start(2))^2+(Z(zloop)-start(3))^2)<start_node_dist)
                 start_node_dist=sqrt((X(xloop)-start(1))^2+(Y(yloop)-start(2))^2+(Z(zloop)-start(3))^2);
                 start_node_ind=nodeind;%保存起始点的编号
             end
             
             if (sqrt((X(xloop)-goal(1))^2+(Y(yloop)-goal(2))^2+(Z(zloop)-goal(3))^2)<goal_node_dist)
                 goal_node_dist=sqrt((X(xloop)-goal(1))^2+(Y(yloop)-goal(2))^2+(Z(zloop)-goal(3))^2);
                 goal_node_ind=nodeind;%保存终点编号
             end
             
         end
     end
end
% Boundary for the map or reach space of robot 
x_max = xmax;
y_max = ymax;
z_max = zmax;
Max_num_Nodes = totnodes; %总个数
q_start.config = [node(start_node_ind ,1) node(start_node_ind ,2) node(start_node_ind ,3)]; % initial position of the robot
q_start.parent = 0; % parent is node through which it passed 父节点
q_goal.config =[node(goal_node_ind ,1) node(goal_node_ind ,2) node(goal_node_ind ,3)]; % Desired position
nodes(1) = q_start; % Defining the first node as starting position
fangxiang=4;
Q=1:totnodes;  
i = Max_num_Nodes;
nf=[];
nf(1)=fang;
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %设置xyz通道  h为最低安全距离 hc为最大测量角度时高度 ujiang为判断条件  w为水平方向距离 wn为位置u时水平距离 sx sy起始点坐标 gx gy终点坐标
  %sita 最大测量角度  l3为起始点与终点水平距离 l2 l1为位置u距离起始点和终点的水平距离
  %l1=(sx-node(,1))^2+(sy-node(,2))^2;
  %          l2=(gx-node(,1))^2+(gy-node(,2))^2;
  %          wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
  %          if wn<w
  %            neighbor_nodes(end+1)=;
  %          end
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   h=28;
   ns=0;
  w=0;
  sita=30/180*3.14;
  ujiang=round(h/z_res*nx);
  hc=ujiang+ns*nx;
  sx=node(start_node_ind,1);
  sy=node(start_node_ind,2);
  gx=node(goal_node_ind,1);
  gy=node(goal_node_ind,2);
  l3=(sx-gx)^2+(sy-gy)^2;
  la3=sqrt(l3);
  l1=0;
  l2=0;
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%循环rrt
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

while i
    i
    i=i-1;
    rx=round(rand*x_max);
    ry=round(rand*y_max);
    rz=round(rand*z_max);
    q_rand = [rx ry rz];
    jieguo=0;
    for j = 1:1:length(nodes)                   %nodes路径系列刚开始只存起始点 先检查是否已经遍历终点
        if nodes(j).config == q_goal.config     %到达目标点
            jieguo=1;
        end
    end
    if jieguo==1
        fangfang=666
        break;
    end
    % {Comparing distance between current nodes and new nodes to find
    % nearest node out of a group of random nodes}比较当前节点和新节点之间的距离 从一组随机节点中最近的节点%
    n_dist = [];
    for j = 1:length(nodes)
        n = nodes(j);
        dist_tmp = euc_dist_3d(n.config, q_rand);%与随机点距离
        n_dist = [n_dist dist_tmp];
    end
    [val, idx] = min(n_dist);
    idx;
    q_near = nodes(idx);
    %求出离随机点最近的点，然后开始计算拓展新点   父节点编号idx
    q_near.config;
    u=round((q_near.config(1)+xy_res)/xy_res+q_near.config(2)/xy_res*nx*nz+q_near.config(3)/z_res*nx);
    Q(u)=-1; 
    idx;
    fangxiang=nf(idx);
    neighbor_nodes=[];
    for j=1:round(node(u,3))
        if node(u-j*nx,3)==0
            break;
        end
    end
    w=(j*z_res-h)*tan(sita);
    while(1)
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  if(fangxiang==1||fangxiang==2||fangxiang==3)
     %front-up-left1
     if u<(ny-1)*nx*nz+1 && rem(u-1,nx*nz)<nx*(nz-1) &&  rem(u-1,nx)~=0 && round(node(u,1),2)==round(node(u+nx+nx*nz-1,1)+xy_res,2)...
         && round((node(u,2)+xy_res),2)==round(node(u+nx+nx*nz-1,2),2) && round((node(u,3)+z_res),2)==round(node(u+nx+nx*nz-1,3),2)...
         && node(u+nx+nx*nz-1,4)==1 && Q(u+nx+nx*nz-1)~=-1 &&  u+nx+nx*nz-1-hc>0  && node(u+nx+nx*nz-1-ujiang,4)==1 % && node(u+nx+nx*nz-1-hc,4)==0 
            l1=(sx-node(u+nx+nx*nz-1,1))^2+(sy-node(u+nx+nx*nz-1,2))^2;
            l2=(gx-node(u+nx+nx*nz-1,1))^2+(gy-node(u+nx+nx*nz-1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
                neighbor_nodes(end+1)=u+nx+nx*nz-1;
                neighbor_nodes(end+1)=1;   
            end
            
     end
    %xy-left-front2
     if rem(u-1,nx)~=0 &&  u<(ny-1)*nx*nz+1 && (round((node(u,1)-xy_res),2)==round(node(u+nx*nz-1,1),2))...
        && (round((node(u,2)+xy_res),2)==round(node(u+nx*nz-1,2),2)) && node(u+nx*nz-1,4)==1 && Q(u+nx*nz-1)~=-1 && u+nx*nz-1-hc>0 && node(u+nx*nz-1-ujiang,4)==1 %&& node(u+nx*nz-1-hc,4)==0
            l1=(sx-node(u+nx*nz-1,1))^2+(sy-node(u+nx*nz-1,2))^2;
            l2=(gx-node(u+nx*nz-1,1))^2+(gy-node(u+nx*nz-1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
                neighbor_nodes(end+1)=u+nx*nz-1;
                neighbor_nodes(end+1)=2;
            end 
    
     end 
    %front-down-left3
      if  u<(ny-1)*nx*nz+1 && rem(u,nx*nz)>nx && rem(u-1,nx)~=0 && round((node(u,1)-xy_res),2)==round(node(u-nx+nx*nz-1,1),2)...
          && round((node(u,2)+xy_res),2)==round(node(u-nx+nx*nz-1,2),2) && round((node(u,3)-z_res),2)==round(node(u-nx+nx*nz-1,3),2)...
          && node(u-nx+nx*nz-1,4)==1 && Q(u-nx+nx*nz-1)~=-1 && u-nx+nx*nz-1-hc>0  && node(u-nx+nx*nz-1-ujiang,4)==1  %&& node(u-nx+nx*nz-1-hc,4)==0
            l1=(sx-node(u-nx+nx*nz-1,1))^2+(sy-node(u-nx+nx*nz-1,2))^2;
            l2=(gx-node(u-nx+nx*nz-1,1))^2+(gy-node(u-nx+nx*nz-1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
                neighbor_nodes(end+1)=u-nx+nx*nz-1;
                neighbor_nodes(end+1)=3;
            end
       
      end
      %zy-up-front4
      if rem(u-1,nx*nz)<nx*(nz-1) && u<(ny-1)*nx*nz+1  && (round((node(u,2)+xy_res),2)==round(node(u+nx+nx*nz,2),2))...
        && (round((node(u,3)+z_res),2)==round(node(u+nx+nx*nz,3),2)) && node(u+nx+nx*nz,4)==1 && Q(u+nx+nx*nz)~=-1 && u+nx+nx*nz-hc>0  && node(u+nx+nx*nz-ujiang,4)==1 %&& node(u+nx+nx*nz-hc,4)==0
            l1=(sx-node(u+nx+nx*nz,1))^2+(sy-node(u+nx+nx*nz,2))^2;
            l2=(gx-node(u+nx+nx*nz,1))^2+(gy-node(u+nx+nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2)
            w
            if wn<w
                neighbor_nodes(end+1)=u+nx+nx*nz;
                neighbor_nodes(end+1)=4;
            end
     end
    %yfront5
     if u<(ny-1)*nx*nz+1 && (round((node(u,2)+xy_res),2)==round(node(u+nx*nz,2),2))...
             && node(u+nx*nz,4)==1 && Q(u+nx*nz)~=-1 &&  u+nx*nz-hc>0 && node(u+nx*nz-ujiang,4)==1 %&& node(u+nx*nz-hc,4)==0
        l1=(sx-node(u+nx*nz,1))^2+(sy-node(u+nx*nz,2))^2;
        l2=(gx-node(u+nx*nz,1))^2+(gy-node(u+nx*nz,2))^2;
        wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
        if wn<w
            neighbor_nodes(end+1)=u+nx*nz;
            neighbor_nodes(end+1)=5;
        end
     end
     %zy-down-front6
     if u<(ny-1)*nx*nz+1 && rem(u,nx*nz)>nx && (round((node(u,2)+xy_res),2)==round(node(u-nx+nx*nz,2),2))... 
         && (round(node(u,3),2)==round((node(u-nx+nx*nz,3)+z_res),2)) && node(u-nx+nx*nz,4)==1 && Q(u-nx+nx*nz)~=-1 &&  u-nx+nx*nz-hc>0 && node(u-nx+nx*nz-ujiang,4)==1 %&& node(u-nx+nx*nz-hc,4)==0
            l1=(sx-node(u-nx+nx*nz,1))^2+(sy-node(u-nx+nx*nz,2))^2;
            l2=(gx-node(u-nx+nx*nz,1))^2+(gy-node(u-nx+nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
                neighbor_nodes(end+1)=u-nx+nx*nz;
                neighbor_nodes(end+1)=6;
            end
     end
    %xz-left-up10
    if  rem(u-1,nx)~=0 && rem(u-1,nx*nz)<nx*(nz-1) && (round((node(u,3)+z_res),2)==round(node(u+nx-1,3),2))...
            &&(round((node(u,1)-xy_res),2)==round(node(u+nx-1,1),2)) && node(u+nx-1,4)==1 && Q(u+nx-1)~=-1 && u+nx-1-hc>0  && node(u+nx-1-ujiang,4)==1  %&& node(u+nx-1-hc,4)==0  
            l1=(sx-node(u+nx-1,1))^2+(sy-node(u+nx-1,2))^2;
            l2=(gx-node(u+nx-1,1))^2+(gy-node(u+nx-1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u+nx-1;
        neighbor_nodes(end+1)=10;
            end  
     end
    %xleft11
    if rem(u-1,nx)~=0 && (round(node(u,1),2)==round((node(u-1,1)+xy_res),2))...
             && node(u-1,4)==1 && Q(u-1)~=-1 && u-1-hc>0 && node(u-1-ujiang,4)==1 %&& node(u-1-hc,4)==0
        l1=(sx-node(u-1,1))^2+(sy-node(u-1,2))^2;
        l2=(gx-node(u-1,1))^2+(gy-node(u-1,2))^2;
        wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
        if wn<w
        neighbor_nodes(end+1)=u-1;
        neighbor_nodes(end+1)=11;
        end
     end
    %xz-left-down12
     if rem(u-1,nx)~=0 && rem(u,nx*nz)>nx && (round((node(u,3)-z_res),2)==round(node(u-nx-1,3),2))...
            &&(round((node(u,1)-xy_res),2)==round(node(u-nx-1,1),2)) && node(u-nx-1,4)==1 && Q(u-nx-1)~=-1 && u-nx-1-hc>0 && node(u-nx-1-ujiang,4)==1% && node(u-nx-1-hc,4)==0   
            l1=(sx-node(u-nx-1,1))^2+(sy-node(u-nx-1,2))^2;
            l2=(gx-node(u-nx-1,1))^2+(gy-node(u-nx-1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u-nx-1;
        neighbor_nodes(end+1)=12;
            end   
     end
    break;
  end
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
 
 if(fangxiang==4||fangxiang==5||fangxiang==6)
   %front-up-left1
      if u<(ny-1)*nx*nz+1 && rem(u-1,nx*nz)<nx*(nz-1) &&  rem(u-1,nx)~=0 && round(node(u,1),2)==round(node(u+nx+nx*nz-1,1)+xy_res,2)...
         && round((node(u,2)+xy_res),2)==round(node(u+nx+nx*nz-1,2),2) && round((node(u,3)+z_res),2)==round(node(u+nx+nx*nz-1,3),2)...
         && node(u+nx+nx*nz-1,4)==1 && Q(u+nx+nx*nz-1)~=-1  &&  u+nx+nx*nz-1-hc>0 && node(u+nx+nx*nz-1-ujiang,4)==1  %&& node(u+nx+nx*nz-1-hc,4)==0 
            l1=(sx-node(u+nx+nx*nz-1,1))^2+(sy-node(u+nx+nx*nz-1,2))^2;
            l2=(gx-node(u+nx+nx*nz-1,1))^2+(gy-node(u+nx+nx*nz-1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2)
            w
            if wn<w
                neighbor_nodes(end+1)=u+nx+nx*nz-1;
                neighbor_nodes(end+1)=1;   
            end
            
     end
    %xy-left-front2
    if rem(u-1,nx)~=0 &&  u<(ny-1)*nx*nz+1 && (round((node(u,1)-xy_res),2)==round(node(u+nx*nz-1,1),2))...
        && (round((node(u,2)+xy_res),2)==round(node(u+nx*nz-1,2),2)) && node(u+nx*nz-1,4)==1 && Q(u+nx*nz-1)~=-1 && u+nx*nz-1-hc>0  && node(u+nx*nz-1-ujiang,4)==1 %&& node(u+nx*nz-1-hc,4)==0
            l1=(sx-node(u+nx*nz-1,1))^2+(sy-node(u+nx*nz-1,2))^2;
            l2=(gx-node(u+nx*nz-1,1))^2+(gy-node(u+nx*nz-1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2)
            w
            if wn<w
                neighbor_nodes(end+1)=u+nx*nz-1;
                neighbor_nodes(end+1)=2;
            end 
    
     end 
    %front-down-left3
       if  u<(ny-1)*nx*nz+1 && rem(u,nx*nz)>nx && rem(u-1,nx)~=0 && round((node(u,1)-xy_res),2)==round(node(u-nx+nx*nz-1,1),2)...
          && round((node(u,2)+xy_res),2)==round(node(u-nx+nx*nz-1,2),2) && round((node(u,3)-z_res),2)==round(node(u-nx+nx*nz-1,3),2)...
          && node(u-nx+nx*nz-1,4)==1 && Q(u-nx+nx*nz-1)~=-1 && u-nx+nx*nz-1-hc>0 && node(u-nx+nx*nz-1-ujiang,4)==1  %&& node(u-nx+nx*nz-1-hc,4)==0
            l1=(sx-node(u-nx+nx*nz-1,1))^2+(sy-node(u-nx+nx*nz-1,2))^2;
            l2=(gx-node(u-nx+nx*nz-1,1))^2+(gy-node(u-nx+nx*nz-1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2)
            w
            if wn<w
                neighbor_nodes(end+1)=u-nx+nx*nz-1;
                neighbor_nodes(end+1)=3;
            end
       
      end
    %zy-up-front4
      if rem(u-1,nx*nz)<nx*(nz-1) && u<(ny-1)*nx*nz+1  && (round((node(u,2)+xy_res),2)==round(node(u+nx+nx*nz,2),2))...
        && (round((node(u,3)+z_res),2)==round(node(u+nx+nx*nz,3),2)) && node(u+nx+nx*nz,4)==1 && Q(u+nx+nx*nz)~=-1  && u+nx+nx*nz-hc>0  && node(u+nx+nx*nz-ujiang,4)==1 %&& node(u+nx+nx*nz-hc,4)==0
            l1=(sx-node(u+nx+nx*nz,1))^2+(sy-node(u+nx+nx*nz,2))^2;
            l2=(gx-node(u+nx+nx*nz,1))^2+(gy-node(u+nx+nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2)
            w
            if wn<w
                neighbor_nodes(end+1)=u+nx+nx*nz;
                neighbor_nodes(end+1)=4;
            end
     end
    %yfront5
     if u<(ny-1)*nx*nz+1 && (round((node(u,2)+xy_res),2)==round(node(u+nx*nz,2),2))...
             && node(u+nx*nz,4)==1 && Q(u+nx*nz)~=-1 &&  u+nx*nz-hc>0 && node(u+nx*nz-ujiang,4)==1% && node(u+nx*nz-hc,4)==0
        l1=(sx-node(u+nx*nz,1))^2+(sy-node(u+nx*nz,2))^2;
        l2=(gx-node(u+nx*nz,1))^2+(gy-node(u+nx*nz,2))^2;
        wn=sqrt(l2-((l2+l3-l1)/2/la3)^2)
        w
        if wn<w
            neighbor_nodes(end+1)=u+nx*nz;
            neighbor_nodes(end+1)=5;
        end
     end
     %zy-down-front6
     if u<(ny-1)*nx*nz+1 && rem(u,nx*nz)>nx && (round((node(u,2)+xy_res),2)==round(node(u-nx+nx*nz,2),2))... 
         && (round(node(u,3),2)==round((node(u-nx+nx*nz,3)+z_res),2)) && node(u-nx+nx*nz,4)==1 && Q(u-nx+nx*nz)~=-1 &&  u-nx+nx*nz-hc>0 && node(u-nx+nx*nz-ujiang,4)==1 %&& node(u-nx+nx*nz-hc,4)==0
            l1=(sx-node(u-nx+nx*nz,1))^2+(sy-node(u-nx+nx*nz,2))^2;
            l2=(gx-node(u-nx+nx*nz,1))^2+(gy-node(u-nx+nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2)
            w
            if wn<w
                neighbor_nodes(end+1)=u-nx+nx*nz;
                neighbor_nodes(end+1)=6;
            end
     end
    %front-up-right7
     if u<(ny-1)*nx*nz+1 &&  rem(u-1,nx*nz)<nx*(nz-1) && rem(u,nx)~=0 && round((node(u,1)+xy_res),2)==round(node(u+nx+nx*nz+1,1),2)...
         && round((node(u,2)+xy_res),2)==round(node(u+nx+nx*nz+1,2),2) && round((node(u,3)+z_res),2)==round(node(u+nx+nx*nz+1,3),2)...
         && node(u+nx+nx*nz+1,4)==1 && Q(u+nx+nx*nz+1)~=-1 && u+nx+nx*nz+1-hc>0 && node(u+nx+nx*nz+1-ujiang,4)==1  %&& node(u+nx+nx*nz+1-hc,4)==0 
            l1=(sx-node(u+nx+nx*nz+1,1))^2+(sy-node(u+nx+nx*nz+1,2))^2;
            l2=(gx-node(u+nx+nx*nz+1,1))^2+(gy-node(u+nx+nx*nz+1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2)
            w
            if wn<w
                neighbor_nodes(end+1)=u+nx+nx*nz+1;
                neighbor_nodes(end+1)=7;
            end
     end
    %xy-right-front8
     if rem(u,nx)~=0 && u<(ny-1)*nx*nz+1 && (round((node(u,1)+xy_res),2)==round(node(u+nx*nz+1,1),2))... 
        && (round((node(u,2)+xy_res),2)==round(node(u+nx*nz+1,2),2)) && node(u+nx*nz+1,4)==1 && Q(u+nx*nz+1)~=-1 &&  u+nx*nz+1-hc>0 && node(u+nx*nz+1-ujiang,4)==1 %&& node(u+nx*nz+1-hc,4)==0
            l1=(sx-node(u+nx*nz+1,1))^2+(sy-node(u+nx*nz+1,2))^2;
            l2=(gx-node(u+nx*nz+1,1))^2+(gy-node(u+nx*nz+1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2)
            w
            if wn<w
         neighbor_nodes(end+1)=u+nx*nz+1;
         neighbor_nodes(end+1)=8;
            end
     end 
    %front-down-right9
     if u<(ny-1)*nx*nz+1 && rem(u,nx*nz)>nx && rem(u,nx)~=0 && round((node(u,1)+xy_res),2)==round(node(u-nx+nx*nz+1,1),2)...
         && round((node(u,2)+xy_res),2)==round(node(u-nx+nx*nz+1,2),2) && round((node(u,3)-z_res),2)==round(node(u-nx+nx*nz+1,3),2)...
         && node(u-nx+nx*nz+1,4)==1 && Q(u-nx+nx*nz+1)~=-1 && u-nx+nx*nz+1-hc>0 && node(u-nx+nx*nz+1-ujiang,4)==1 % && node(u-nx+nx*nz+1-hc,4)==0 
            l1=(sx-node(u-nx+nx*nz+1,1))^2+(sy-node(u-nx+nx*nz+1,2))^2;
            l2=(gx-node(u-nx+nx*nz+1,1))^2+(gy-node(u-nx+nx*nz+1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2)
            w
            if wn<w
        neighbor_nodes(end+1)=u-nx+nx*nz+1;
        neighbor_nodes(end+1)=9;
            end
     end 
  break;
 end

 
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
  
  if(fangxiang==7||fangxiang==8||fangxiang==9)
  
      %zy-up-front4
     if rem(u-1,nx*nz)<nx*(nz-1) && u<(ny-1)*nx*nz+1  && (round((node(u,2)+xy_res),2)==round(node(u+nx+nx*nz,2),2))...
        && (round((node(u,3)+z_res),2)==round(node(u+nx+nx*nz,3),2)) && node(u+nx+nx*nz,4)==1 && Q(u+nx+nx*nz)~=-1  && u+nx+nx*nz-hc>0  && node(u+nx+nx*nz-ujiang,4)==1 %&& node(u+nx+nx*nz-hc,4)==0
            l1=(sx-node(u+nx+nx*nz,1))^2+(sy-node(u+nx+nx*nz,2))^2;
            l2=(gx-node(u+nx+nx*nz,1))^2+(gy-node(u+nx+nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
                neighbor_nodes(end+1)=u+nx+nx*nz;
                neighbor_nodes(end+1)=4;
            end
     end
    %yfront5
     if u<(ny-1)*nx*nz+1 && (round((node(u,2)+xy_res),2)==round(node(u+nx*nz,2),2))...
             && node(u+nx*nz,4)==1 && Q(u+nx*nz)~=-1  &&  u+nx*nz-hc>0 && node(u+nx*nz-ujiang,4)==1 %&& node(u+nx*nz-hc,4)==0
        l1=(sx-node(u+nx*nz,1))^2+(sy-node(u+nx*nz,2))^2;
        l2=(gx-node(u+nx*nz,1))^2+(gy-node(u+nx*nz,2))^2;
        wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
        if wn<w
            neighbor_nodes(end+1)=u+nx*nz;
            neighbor_nodes(end+1)=5;
        end
     end
     %zy-down-front6
     if u<(ny-1)*nx*nz+1 && rem(u,nx*nz)>nx && (round((node(u,2)+xy_res),2)==round(node(u-nx+nx*nz,2),2))... 
         && (round(node(u,3),2)==round((node(u-nx+nx*nz,3)+z_res),2)) && node(u-nx+nx*nz,4)==1 && Q(u-nx+nx*nz)~=-1 &&  u-nx+nx*nz-hc>0 && node(u-nx+nx*nz-ujiang,4)==1 %&& node(u-nx+nx*nz-hc,4)==0
            l1=(sx-node(u-nx+nx*nz,1))^2+(sy-node(u-nx+nx*nz,2))^2;
            l2=(gx-node(u-nx+nx*nz,1))^2+(gy-node(u-nx+nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
                neighbor_nodes(end+1)=u-nx+nx*nz;
                neighbor_nodes(end+1)=6;
            end
     end
    %front-up-right7
     if u<(ny-1)*nx*nz+1 &&  rem(u-1,nx*nz)<nx*(nz-1) && rem(u,nx)~=0 && round((node(u,1)+xy_res),2)==round(node(u+nx+nx*nz+1,1),2)...
         && round((node(u,2)+xy_res),2)==round(node(u+nx+nx*nz+1,2),2) && round((node(u,3)+z_res),2)==round(node(u+nx+nx*nz+1,3),2)...
         && node(u+nx+nx*nz+1,4)==1 && Q(u+nx+nx*nz+1)~=-1 && u+nx+nx*nz+1-hc>0 && node(u+nx+nx*nz+1-ujiang,4)==1  %&& node(u+nx+nx*nz+1-hc,4)==0 
            l1=(sx-node(u+nx+nx*nz+1,1))^2+(sy-node(u+nx+nx*nz+1,2))^2;
            l2=(gx-node(u+nx+nx*nz+1,1))^2+(gy-node(u+nx+nx*nz+1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
                neighbor_nodes(end+1)=u+nx+nx*nz+1;
                neighbor_nodes(end+1)=7;
            end
     end
    %xy-right-front8
     if rem(u,nx)~=0 && u<(ny-1)*nx*nz+1 && (round((node(u,1)+xy_res),2)==round(node(u+nx*nz+1,1),2))... 
        && (round((node(u,2)+xy_res),2)==round(node(u+nx*nz+1,2),2)) && node(u+nx*nz+1,4)==1 && Q(u+nx*nz+1)~=-1  &&  u+nx*nz+1-hc>0 && node(u+nx*nz+1-ujiang,4)==1 %&& node(u+nx*nz+1-hc,4)==0
            l1=(sx-node(u+nx*nz+1,1))^2+(sy-node(u+nx*nz+1,2))^2;
            l2=(gx-node(u+nx*nz+1,1))^2+(gy-node(u+nx*nz+1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
         neighbor_nodes(end+1)=u+nx*nz+1;
         neighbor_nodes(end+1)=8;
            end
     end 
    %front-down-right9
     if u<(ny-1)*nx*nz+1 && rem(u,nx*nz)>nx && rem(u,nx)~=0 && round((node(u,1)+xy_res),2)==round(node(u-nx+nx*nz+1,1),2)...
         && round((node(u,2)+xy_res),2)==round(node(u-nx+nx*nz+1,2),2) && round((node(u,3)-z_res),2)==round(node(u-nx+nx*nz+1,3),2)...
         && node(u-nx+nx*nz+1,4)==1 && Q(u-nx+nx*nz+1)~=-1 && u-nx+nx*nz+1-hc>0 && node(u-nx+nx*nz+1-ujiang,4)==1  %&& node(u-nx+nx*nz+1-hc,4)==0 
            l1=(sx-node(u-nx+nx*nz+1,1))^2+(sy-node(u-nx+nx*nz+1,2))^2;
            l2=(gx-node(u-nx+nx*nz+1,1))^2+(gy-node(u-nx+nx*nz+1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u-nx+nx*nz+1;
        neighbor_nodes(end+1)=9;
            end
     end 
     %xz-right-up15
     if rem(u,nx)~=0 && rem(u-1,nx*nz)<nx*(nz-1) && (round((node(u,3)+z_res),2)==round(node(u+nx+1,3),2))...
            &&(round((node(u,1)+xy_res),2)==round(node(u+nx+1,1),2)) && node(u+nx+1,4)==1 && Q(u+nx+1)~=-1 && u+nx+1-hc>0    && node(u+nx+1-ujiang,4)==1   %&& node(u+nx+1-hc,4)==0 
            l1=(sx-node(u+nx+1,1))^2+(sy-node(u+nx+1,2))^2;
            l2=(gx-node(u+nx+1,1))^2+(gy-node(u+nx+1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u+nx+1;
        neighbor_nodes(end+1)=15;
            end
      end
    %xright 16
      if  rem(u,nx)~=0 &&(round((node(u,1)+xy_res),2)==round(node(u+1,1),2))...
             && node(u+1,4)==1 && Q(u+1)~=-1  && u+1-hc>0    && node(u+1-ujiang,4)==1 %&& node(u+1-hc,4)==0  
            l1=(sx-node(u+1,1))^2+(sy-node(u+1,2))^2;
            l2=(gx-node(u+1,1))^2+(gy-node(u+1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u+1;
        neighbor_nodes(end+1)=16;
            end
     end
    %xz-right-down17
     if  rem(u,nx)~=0 &&  rem(u,nx*nz)>nx && (round((node(u,3)-z_res),2)==round(node(u-nx+1,3),2))...
            &&(round((node(u,1)+xy_res),2)==round(node(u-nx+1,1),2)) && node(u-nx+1,4)==1 && Q(u-nx+1)~=-1 && u+1-hc>0   && node(u-nx+1-ujiang,4)==1 %&& node(u-nx+1-hc,4)==0
            l1=(sx-node(u-nx+1,1))^2+(sy-node(u-nx+1,2))^2;
            l2=(gx-node(u-nx+1,1))^2+(gy-node(u-nx+1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u-nx+1;
        neighbor_nodes(end+1)=17;
            end 
     end
   break;
  end
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

  if(fangxiang==10||fangxiang==11||fangxiang==12)
      
       %front-up-left1
      if u<(ny-1)*nx*nz+1 && rem(u-1,nx*nz)<nx*(nz-1) &&  rem(u-1,nx)~=0 && round(node(u,1),2)==round(node(u+nx+nx*nz-1,1)+xy_res,2)...
         && round((node(u,2)+xy_res),2)==round(node(u+nx+nx*nz-1,2),2) && round((node(u,3)+z_res),2)==round(node(u+nx+nx*nz-1,3),2)...
         && node(u+nx+nx*nz-1,4)==1 && Q(u+nx+nx*nz-1)~=-1  &&  u+nx+nx*nz-1-hc>0 && node(u+nx+nx*nz-1-ujiang,4)==1  %&& node(u+nx+nx*nz-1-hc,4)==0 
            l1=(sx-node(u+nx+nx*nz-1,1))^2+(sy-node(u+nx+nx*nz-1,2))^2;
            l2=(gx-node(u+nx+nx*nz-1,1))^2+(gy-node(u+nx+nx*nz-1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
                neighbor_nodes(end+1)=u+nx+nx*nz-1;
                neighbor_nodes(end+1)=1;   
            end
            
     end
    %xy-left-front2
     if rem(u-1,nx)~=0 &&  u<(ny-1)*nx*nz+1 && (round((node(u,1)-xy_res),2)==round(node(u+nx*nz-1,1),2))...
        && (round((node(u,2)+xy_res),2)==round(node(u+nx*nz-1,2),2)) && node(u+nx*nz-1,4)==1 && Q(u+nx*nz-1)~=-1 && u+nx*nz-1-hc>0  && node(u+nx*nz-1-ujiang,4)==1% && node(u+nx*nz-1-hc,4)==0
            l1=(sx-node(u+nx*nz-1,1))^2+(sy-node(u+nx*nz-1,2))^2;
            l2=(gx-node(u+nx*nz-1,1))^2+(gy-node(u+nx*nz-1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
                neighbor_nodes(end+1)=u+nx*nz-1;
                neighbor_nodes(end+1)=2;
            end 
    
     end 
    %front-down-left3
      if  u<(ny-1)*nx*nz+1 && rem(u,nx*nz)>nx && rem(u-1,nx)~=0 && round((node(u,1)-xy_res),2)==round(node(u-nx+nx*nz-1,1),2)...
          && round((node(u,2)+xy_res),2)==round(node(u-nx+nx*nz-1,2),2) && round((node(u,3)-z_res),2)==round(node(u-nx+nx*nz-1,3),2)...
          && node(u-nx+nx*nz-1,4)==1 && Q(u-nx+nx*nz-1)~=-1 && u-nx+nx*nz-1-hc>0 && node(u-nx+nx*nz-1-ujiang,4)==1 % && node(u-nx+nx*nz-1-hc,4)==0
            l1=(sx-node(u-nx+nx*nz-1,1))^2+(sy-node(u-nx+nx*nz-1,2))^2;
            l2=(gx-node(u-nx+nx*nz-1,1))^2+(gy-node(u-nx+nx*nz-1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
                neighbor_nodes(end+1)=u-nx+nx*nz-1;
                neighbor_nodes(end+1)=3;
            end
       
      end
      %xz-left-up10
    if  rem(u-1,nx)~=0 && rem(u-1,nx*nz)<nx*(nz-1) && (round((node(u,3)+z_res),2)==round(node(u+nx-1,3),2))...
            &&(round((node(u,1)-xy_res),2)==round(node(u+nx-1,1),2)) && node(u+nx-1,4)==1 && Q(u+nx-1)~=-1 && u+nx-1-hc>0 && node(u+nx-1-ujiang,4)==1 % && node(u+nx-1-hc,4)==0  
            l1=(sx-node(u+nx-1,1))^2+(sy-node(u+nx-1,2))^2;
            l2=(gx-node(u+nx-1,1))^2+(gy-node(u+nx-1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u+nx-1;
        neighbor_nodes(end+1)=10;
            end  
     end
    %xleft11
     if rem(u-1,nx)~=0 && (round(node(u,1),2)==round((node(u-1,1)+xy_res),2))...
             && node(u-1,4)==1 && Q(u-1)~=-1 && u-1-hc>0 && node(u-1-ujiang,4)==1 %&& node(u-1-hc,4)==0
        l1=(sx-node(u-1,1))^2+(sy-node(u-1,2))^2;
        l2=(gx-node(u-1,1))^2+(gy-node(u-1,2))^2;
        wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
        if wn<w
        neighbor_nodes(end+1)=u-1;
        neighbor_nodes(end+1)=11;
        end
     end
     %xz-left-down12
      if rem(u-1,nx)~=0 && rem(u,nx*nz)>nx && (round((node(u,3)-z_res),2)==round(node(u-nx-1,3),2))...
            &&(round((node(u,1)-xy_res),2)==round(node(u-nx-1,1),2)) && node(u-nx-1,4)==1 && Q(u-nx-1)~=-1 && u-nx-1-hc>0 && node(u-nx-1-ujiang,4)==1 %&& node(u-nx-1-hc,4)==0   
            l1=(sx-node(u-nx-1,1))^2+(sy-node(u-nx-1,2))^2;
            l2=(gx-node(u-nx-1,1))^2+(gy-node(u-nx-1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u-nx-1;
        neighbor_nodes(end+1)=12;
            end   
     end
     %back-left-up18
      if u>nx*nz &&  rem(u-1,nx)~=0 && rem(u-1,nx*nz)<nx*(nz-1) && (round((node(u,1)-xy_res),2)==round(node(u-1+nx-nx*nz,1),2))...
            &&(round((node(u,2)-xy_res),2)==round(node(u-1+nx-nx*nz,2),2)) &&(round((node(u,3)+z_res),2)==round(node(u-1+nx-nx*nz,3),2)) && node(u-1+nx-nx*nz,4)==1 && Q(u-1+nx-nx*nz)~=-1  &&  u-1+nx-nx*nz-hc>0 && node(u-1+nx-nx*nz-ujiang,4)==1 %&& node(u-1+nx-nx*nz-hc,4)==0   
            l1=(sx-node(u-1+nx-nx*nz,1))^2+(sy-node(u-1+nx-nx*nz,2))^2;
            l2=(gx-node(u-1+nx-nx*nz,1))^2+(gy-node(u-1+nx-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u-1+nx-nx*nz;
        neighbor_nodes(end+1)=18;
            end     
    end
    %xy-back-left19
      if u>nx*nz &&  rem(u-1,nx)~=0 && (round(node(u,1),2)==round((node(u-1-nx*nz,1)+xy_res),2))...
         && (round(node(u,2),2)==round((node(u-1-nx*nz,2)+xy_res),2)) && node(u-1-nx*nz,4)==1 && Q(u-1-nx*nz)~=-1 && u-nx*nz-1-hc>0 && node(u-nx*nz-1-ujiang,4)==1 %&& node(u-nx*nz-1-hc,4)==0
            l1=(sx-node(u-1-nx*nz,1))^2+(sy-node(u-1-nx*nz,2))^2;
            l2=(gx-node(u-1-nx*nz,1))^2+(gy-node(u-1-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
            neighbor_nodes(end+1)=u-nx*nz-1;
            neighbor_nodes(end+1)=19;
            end
     end   
    %back-left-down20
    if u>nx*nz &&  rem(u-1,nx)~=0 && rem(u,nx*nz)>nx && (round((node(u,1)-xy_res),2)==round(node(u-1-nx-nx*nz,1),2))...
            &&(round((node(u,2)-xy_res),2)==round(node(u-1-nx-nx*nz,2),2)) &&(round((node(u,3)-z_res),2)==round(node(u-1-nx-nx*nz,3),2)) && node(u-1-nx-nx*nz,4)==1 && Q(u-1-nx-nx*nz)~=-1 && u-1-nx-nx*nz-hc>0 && node(u-1-nx-nx*nz-ujiang,4)==1 %&& node(u-1-nx-nx*nz-hc,4)==0
            l1=(sx-node(u-1-nx-nx*nz,1))^2+(sy-node(u-1-nx-nx*nz,2))^2;
            l2=(gx-node(u-1-nx-nx*nz,1))^2+(gy-node(u-1-nx-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
            neighbor_nodes(end+1)=u-1-nx-nx*nz;
            neighbor_nodes(end+1)=20;
            end 
    end

   break;   
  end
  
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

  if(fangxiang==15||fangxiang==16||fangxiang==17)
      
      %front-up-right7
     if u<(ny-1)*nx*nz+1 &&  rem(u-1,nx*nz)<nx*(nz-1) && rem(u,nx)~=0 && round((node(u,1)+xy_res),2)==round(node(u+nx+nx*nz+1,1),2)...
         && round((node(u,2)+xy_res),2)==round(node(u+nx+nx*nz+1,2),2) && round((node(u,3)+z_res),2)==round(node(u+nx+nx*nz+1,3),2)...
         && node(u+nx+nx*nz+1,4)==1 && Q(u+nx+nx*nz+1)~=-1 && u+nx+nx*nz+1-hc>0 && node(u+nx+nx*nz+1-ujiang,4)==1  %&& node(u+nx+nx*nz+1-hc,4)==0 
            l1=(sx-node(u+nx+nx*nz+1,1))^2+(sy-node(u+nx+nx*nz+1,2))^2;
            l2=(gx-node(u+nx+nx*nz+1,1))^2+(gy-node(u+nx+nx*nz+1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
                neighbor_nodes(end+1)=u+nx+nx*nz+1;
                neighbor_nodes(end+1)=7;
            end
     end
    %xy-right-front8
     if rem(u,nx)~=0 && u<(ny-1)*nx*nz+1 && (round((node(u,1)+xy_res),2)==round(node(u+nx*nz+1,1),2))... 
        && (round((node(u,2)+xy_res),2)==round(node(u+nx*nz+1,2),2)) && node(u+nx*nz+1,4)==1 && Q(u+nx*nz+1)~=-1  &&  u+nx*nz+1-hc>0 && node(u+nx*nz+1-ujiang,4)==1 %&& node(u+nx*nz+1-hc,4)==0
            l1=(sx-node(u+nx*nz+1,1))^2+(sy-node(u+nx*nz+1,2))^2;
            l2=(gx-node(u+nx*nz+1,1))^2+(gy-node(u+nx*nz+1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
         neighbor_nodes(end+1)=u+nx*nz+1;
         neighbor_nodes(end+1)=8;
            end
     end 
    %front-down-right9
     if u<(ny-1)*nx*nz+1 && rem(u,nx*nz)>nx && rem(u,nx)~=0 && round((node(u,1)+xy_res),2)==round(node(u-nx+nx*nz+1,1),2)...
         && round((node(u,2)+xy_res),2)==round(node(u-nx+nx*nz+1,2),2) && round((node(u,3)-z_res),2)==round(node(u-nx+nx*nz+1,3),2)...
         && node(u-nx+nx*nz+1,4)==1 && Q(u-nx+nx*nz+1)~=-1 && u-nx+nx*nz+1-hc>0 && node(u-nx+nx*nz+1-ujiang,4)==1  %&& node(u-nx+nx*nz+1-hc,4)==0 
            l1=(sx-node(u-nx+nx*nz+1,1))^2+(sy-node(u-nx+nx*nz+1,2))^2;
            l2=(gx-node(u-nx+nx*nz+1,1))^2+(gy-node(u-nx+nx*nz+1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u-nx+nx*nz+1;
        neighbor_nodes(end+1)=9;
            end
     end 
     
     %xz-right-up15
      if rem(u,nx)~=0 && rem(u-1,nx*nz)<nx*(nz-1) && (round((node(u,3)+z_res),2)==round(node(u+nx+1,3),2))...
            &&(round((node(u,1)+xy_res),2)==round(node(u+nx+1,1),2)) && node(u+nx+1,4)==1 && Q(u+nx+1)~=-1 && u+nx+1-hc>0  && node(u+nx+1-ujiang,4)==1  % && node(u+nx+1-hc,4)==0 
            l1=(sx-node(u+nx+1,1))^2+(sy-node(u+nx+1,2))^2;
            l2=(gx-node(u+nx+1,1))^2+(gy-node(u+nx+1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u+nx+1;
        neighbor_nodes(end+1)=15;
            end
      end
    %xright 16
     if  rem(u,nx)~=0 &&(round((node(u,1)+xy_res),2)==round(node(u+1,1),2))...
             && node(u+1,4)==1 && Q(u+1)~=-1 && u+1-hc>0  && node(u+1-ujiang,4)==1 %&& node(u+1-hc,4)==0  
            l1=(sx-node(u+1,1))^2+(sy-node(u+1,2))^2;
            l2=(gx-node(u+1,1))^2+(gy-node(u+1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u+1;
        neighbor_nodes(end+1)=16;
            end
     end
    %xz-right-down17
     if  rem(u,nx)~=0 &&  rem(u,nx*nz)>nx && (round((node(u,3)-z_res),2)==round(node(u-nx+1,3),2))...
            &&(round((node(u,1)+xy_res),2)==round(node(u-nx+1,1),2)) && node(u-nx+1,4)==1 && Q(u-nx+1)~=-1 && u-nx+1-hc>0   && node(u-nx+1-ujiang,4)==1% && node(u-nx+1-hc,4)==0
            l1=(sx-node(u-nx+1,1))^2+(sy-node(u-nx+1,2))^2;
            l2=(gx-node(u-nx+1,1))^2+(gy-node(u-nx+1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u-nx+1;
        neighbor_nodes(end+1)=17;
            end 
     end
     
      %back-right-up24
     if  u>nx*nz && rem(u,nx)~=0 &&  rem(u-1,nx*nz)<nx*(nz-1) && (round((node(u,1)+xy_res),2)==round(node(u+1+nx-nx*nz,1),2))...
            &&(round((node(u,2)-xy_res),2)==round(node(u+1+nx-nx*nz,2),2)) &&(round((node(u,3)+z_res),2)==round(node(u+1+nx-nx*nz,3),2)) && node(u+1+nx-nx*nz,4)==1 && Q(u+1+nx-nx*nz)~=-1 && u+1+nx-nx*nz-hc>0  && node(u+1+nx-nx*nz-ujiang,4)==1% && node(u+1+nx-nx*nz-hc,4)==0
            l1=(sx-node(u+1+nx-nx*nz,1))^2+(sy-node(u+1+nx-nx*nz,2))^2;
            l2=(gx-node(u+1+nx-nx*nz,1))^2+(gy-node(u+1+nx-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
       neighbor_nodes(end+1)=u+1+nx-nx*nz;
       neighbor_nodes(end+1)=24;
            end       
     end
        %xy-back-right25
      if u>nx*nz && rem(u,nx)~=0 && (round((node(u,1)+xy_res),2)==round(node(u+1-nx*nz,1),2))...
                && (round((node(u,2)+xy_res),2)==round(node(u+1-nx*nz,2),2)) && node(u+1-nx*nz,4)==1 && Q(u+1-nx*nz)~=-1  && u+1-nx*nz-hc>0  && node(u+1-nx*nz-ujiang,4)==1 %&& node(u+1-nx*nz-hc,4)==0
            l1=(sx-node(u+1-nx*nz,1))^2+(sy-node(u+1-nx*nz,2))^2;
            l2=(gx-node(u+1-nx*nz,1))^2+(gy-node(u+1-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
            neighbor_nodes(end+1)=u+1-nx*nz;
            neighbor_nodes(end+1)=25;
            end
         end
   %back-right-down26
   if u>nx*nz && rem(u,nx)~=0 && rem(u,nx*nz)>nx && (round((node(u,1)-xy_res),2)==round(node(u+1-nx-nx*nz,1),2))...
            &&(round((node(u,2)-xy_res),2)==round(node(u+1-nx-nx*nz,2),2)) &&(round((node(u,3)+z_res),2)==round(node(u+1-nx-nx*nz,3),2)) && node(u+1-nx-nx*nz,4)==1 && Q(u+1-nx-nx*nz)~=-1  && u+1-nx-nx*nz-hc>0 && node(u+1-nx-nx*nz-ujiang,4)==1% && node(u+1-nx-nx*nz-hc,4)==0
            l1=(sx-node(u+1-nx-nx*nz,1))^2+(sy-node(u+1-nx-nx*nz,2))^2;
            l2=(gx-node(u+1-nx-nx*nz,1))^2+(gy-node(u+1-nx-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
       neighbor_nodes(end+1)=u+1-nx-nx*nz;
       neighbor_nodes(end+1)=26; 
            end  
        end
  break;    
  end
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

  if(fangxiang==18||fangxiang==19||fangxiang==20)
     
      %xz-left-up10
     if  rem(u-1,nx)~=0 && rem(u-1,nx*nz)<nx*(nz-1) && (round((node(u,3)+z_res),2)==round(node(u+nx-1,3),2))...
            &&(round((node(u,1)-xy_res),2)==round(node(u+nx-1,1),2)) && node(u+nx-1,4)==1 && Q(u+nx-1)~=-1 && u+nx-1-hc>0 && node(u+nx-1-ujiang,4)==1 % && node(u+nx-1-hc,4)==0  
            l1=(sx-node(u+nx-1,1))^2+(sy-node(u+nx-1,2))^2;
            l2=(gx-node(u+nx-1,1))^2+(gy-node(u+nx-1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u+nx-1;
        neighbor_nodes(end+1)=10;
            end  
     end
    %xleft11
     if rem(u-1,nx)~=0 && (round(node(u,1),2)==round((node(u-1,1)+xy_res),2))...
             && node(u-1,4)==1 && Q(u-1)~=-1 && u-1-hc>0 && node(u-1-ujiang,4)==1 %&& node(u-1-hc,4)==0
        l1=(sx-node(u-1,1))^2+(sy-node(u-1,2))^2;
        l2=(gx-node(u-1,1))^2+(gy-node(u-1,2))^2;
        wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
        if wn<w
        neighbor_nodes(end+1)=u-1;
        neighbor_nodes(end+1)=11;
        end
     end
    %xz-left-down12
     if rem(u-1,nx)~=0 && rem(u,nx*nz)>nx && (round((node(u,3)-z_res),2)==round(node(u-nx-1,3),2))...
            &&(round((node(u,1)-xy_res),2)==round(node(u-nx-1,1),2)) && node(u-nx-1,4)==1 && Q(u-nx-1)~=-1  && u-nx-1-hc>0 && node(u-nx-1-ujiang,4)==1 %&& node(u-nx-1-hc,4)==0   
            l1=(sx-node(u-nx-1,1))^2+(sy-node(u-nx-1,2))^2;
            l2=(gx-node(u-nx-1,1))^2+(gy-node(u-nx-1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u-nx-1;
        neighbor_nodes(end+1)=12;
            end   
     end
      
     %back-left-up18
    if u>nx*nz &&  rem(u-1,nx)~=0 && rem(u-1,nx*nz)<nx*(nz-1) && (round((node(u,1)-xy_res),2)==round(node(u-1+nx-nx*nz,1),2))...
            &&(round((node(u,2)-xy_res),2)==round(node(u-1+nx-nx*nz,2),2)) &&(round((node(u,3)+z_res),2)==round(node(u-1+nx-nx*nz,3),2)) && node(u-1+nx-nx*nz,4)==1 && Q(u-1+nx-nx*nz)~=-1 &&  u-1+nx-nx*nz-hc>0  && node(u-1+nx-nx*nz-ujiang,4)==1% && node(u-1+nx-nx*nz-hc,4)==0   
            l1=(sx-node(u-1+nx-nx*nz,1))^2+(sy-node(u-1+nx-nx*nz,2))^2;
            l2=(gx-node(u-1+nx-nx*nz,1))^2+(gy-node(u-1+nx-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u-1+nx-nx*nz;
        neighbor_nodes(end+1)=18;
            end     
    end
    %xy-back-left19
     if u>nx*nz &&  rem(u-1,nx)~=0 && (round(node(u,1),2)==round((node(u-1-nx*nz,1)+xy_res),2))...
         && (round(node(u,2),2)==round((node(u-1-nx*nz,2)+xy_res),2)) && node(u-1-nx*nz,4)==1 && Q(u-1-nx*nz)~=-1 && u-nx*nz-1-hc>0 && node(u-nx*nz-1-ujiang,4)==1 %&& node(u-nx*nz-1-hc,4)==0
            l1=(sx-node(u-1-nx*nz,1))^2+(sy-node(u-1-nx*nz,2))^2;
            l2=(gx-node(u-1-nx*nz,1))^2+(gy-node(u-1-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
            neighbor_nodes(end+1)=u-nx*nz-1;
            neighbor_nodes(end+1)=19;
            end
     end   
    %back-left-down20
    if u>nx*nz &&  rem(u-1,nx)~=0 && rem(u,nx*nz)>nx && (round((node(u,1)-xy_res),2)==round(node(u-1-nx-nx*nz,1),2))...
            &&(round((node(u,2)-xy_res),2)==round(node(u-1-nx-nx*nz,2),2)) &&(round((node(u,3)-z_res),2)==round(node(u-1-nx-nx*nz,3),2)) && node(u-1-nx-nx*nz,4)==1 && Q(u-1-nx-nx*nz)~=-1 && u-1-nx-nx*nz-hc>0 && node(u-1-nx-nx*nz-ujiang,4)==1 %&& node(u-1-nx-nx*nz-hc,4)==0
            l1=(sx-node(u-1-nx-nx*nz,1))^2+(sy-node(u-1-nx-nx*nz,2))^2;
            l2=(gx-node(u-1-nx-nx*nz,1))^2+(gy-node(u-1-nx-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
            neighbor_nodes(end+1)=u-1-nx-nx*nz;
            neighbor_nodes(end+1)=20;
            end 
    end
    
    %back-up21
    if u>nx*nz && rem(u-1,nx*nz)<nx*(nz-1) && (round((node(u,2)-xy_res),2)==round(node(u+nx-nx*nz,2),2))...
             &&(round((node(u,3)+z_res),2)==round(node(u+nx-nx*nz,3),2)) && node(u+nx-nx*nz,4)==1 && Q(u+nx-nx*nz)~=-1 &&  u+nx-nx*nz-hc>0 && node(u+nx-nx*nz-ujiang,4)==1 %&& node(u+nx-nx*nz-hc,4)==0
            l1=(sx-node(u+nx-nx*nz,1))^2+(sy-node(u+nx-nx*nz,2))^2;
            l2=(gx-node(u+nx-nx*nz,1))^2+(gy-node(u+nx-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
       neighbor_nodes(end+1)=u+nx-nx*nz;
       neighbor_nodes(end+1)=21; 
            end   
    end
    %yback22
     if u>nx*nz && (round(node(u,2),2)==round((node(u-nx*nz,2)+xy_res),2))...
             && node(u-nx*nz,4)==1 && Q(u-nx*nz)~=-1 &&  u-nx*nz-hc>0 && node(u-nx*nz-ujiang,4)==1 && node(u-nx*nz-hc,4)==0
            l1=(sx-node(u-nx*nz,1))^2+(sy-node(u-nx*nz,2))^2;
            l2=(gx-node(u-nx*nz,1))^2+(gy-node(u-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u-nx*nz;
        neighbor_nodes(end+1)=22;
            end   
     end
     %back-down23
    if u>nx*nz && rem(u,nx*nz)>nx && (round((node(u,2)-xy_res),2)==round(node(u-nx-nx*nz,2),2))...
             &&(round((node(u,3)-z_res),2)==round(node(u-nx-nx*nz,3),2)) && node(u-nx-nx*nz,4)==1 && Q(u-nx-nx*nz)~=-1 && u-nx-nx*nz-hc>0 && node(u-nx-nx*nz-ujiang,4)==1 %&& node(u-nx-nx*nz-hc,4)==0
            l1=(sx-node(u-nx-nx*nz,1))^2+(sy-node(u-nx-nx*nz,2))^2;
            l2=(gx-node(u-nx-nx*nz,1))^2+(gy-node(u-nx-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
       neighbor_nodes(end+1)=u-nx-nx*nz;
       neighbor_nodes(end+1)=23; 
            end     
     end
  break;   
  end
  
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

  if(fangxiang==21||fangxiang==22||fangxiang==23)
    
      %back-left-up18
     if u>nx*nz &&  rem(u-1,nx)~=0 && rem(u-1,nx*nz)<nx*(nz-1) && (round((node(u,1)-xy_res),2)==round(node(u-1+nx-nx*nz,1),2))...
            &&(round((node(u,2)-xy_res),2)==round(node(u-1+nx-nx*nz,2),2)) &&(round((node(u,3)+z_res),2)==round(node(u-1+nx-nx*nz,3),2)) && node(u-1+nx-nx*nz,4)==1 && Q(u-1+nx-nx*nz)~=-1  &&  u-1+nx-nx*nz-hc>0 && node(u-1+nx-nx*nz-ujiang,4)==1% && node(u-1+nx-nx*nz-hc,4)==0   
            l1=(sx-node(u-1+nx-nx*nz,1))^2+(sy-node(u-1+nx-nx*nz,2))^2;
            l2=(gx-node(u-1+nx-nx*nz,1))^2+(gy-node(u-1+nx-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u-1+nx-nx*nz;
        neighbor_nodes(end+1)=18;
            end     
    end
    %xy-back-left19
     if u>nx*nz &&  rem(u-1,nx)~=0 && (round(node(u,1),2)==round((node(u-1-nx*nz,1)+xy_res),2))...
         && (round(node(u,2),2)==round((node(u-1-nx*nz,2)+xy_res),2)) && node(u-1-nx*nz,4)==1 && Q(u-1-nx*nz)~=-1 && u-nx*nz-1-hc>0 && node(u-nx*nz-1-ujiang,4)==1 %&& node(u-nx*nz-1-hc,4)==0
            l1=(sx-node(u-1-nx*nz,1))^2+(sy-node(u-1-nx*nz,2))^2;
            l2=(gx-node(u-1-nx*nz,1))^2+(gy-node(u-1-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
            neighbor_nodes(end+1)=u-nx*nz-1;
            neighbor_nodes(end+1)=19;
            end
     end    
    %back-left-down20
    if u>nx*nz &&  rem(u-1,nx)~=0 && rem(u,nx*nz)>nx && (round((node(u,1)-xy_res),2)==round(node(u-1-nx-nx*nz,1),2))...
            &&(round((node(u,2)-xy_res),2)==round(node(u-1-nx-nx*nz,2),2)) &&(round((node(u,3)-z_res),2)==round(node(u-1-nx-nx*nz,3),2)) && node(u-1-nx-nx*nz,4)==1 && Q(u-1-nx-nx*nz)~=-1 && u-1-nx-nx*nz-hc>0 && u-1-nx-nx*nz-hc>0 && node(u-1-nx-nx*nz-ujiang,4)==1% && node(u-1-nx-nx*nz-hc,4)==0
            l1=(sx-node(u-1-nx-nx*nz,1))^2+(sy-node(u-1-nx-nx*nz,2))^2;
            l2=(gx-node(u-1-nx-nx*nz,1))^2+(gy-node(u-1-nx-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
            neighbor_nodes(end+1)=u-1-nx-nx*nz;
            neighbor_nodes(end+1)=20;
            end 
    end
    %back-up21
    if u>nx*nz && rem(u-1,nx*nz)<nx*(nz-1) && (round((node(u,2)-xy_res),2)==round(node(u+nx-nx*nz,2),2))...
             &&(round((node(u,3)+z_res),2)==round(node(u+nx-nx*nz,3),2)) && node(u+nx-nx*nz,4)==1 && Q(u+nx-nx*nz)~=-1 &&  u+nx-nx*nz-hc>0 && node(u+nx-nx*nz-ujiang,4)==1 %&& node(u+nx-nx*nz-hc,4)==0
            l1=(sx-node(u+nx-nx*nz,1))^2+(sy-node(u+nx-nx*nz,2))^2;
            l2=(gx-node(u+nx-nx*nz,1))^2+(gy-node(u+nx-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
       neighbor_nodes(end+1)=u+nx-nx*nz;
       neighbor_nodes(end+1)=21; 
            end   
    end
    %yback22
    if u>nx*nz && (round(node(u,2),2)==round((node(u-nx*nz,2)+xy_res),2))...
             && node(u-nx*nz,4)==1 && Q(u-nx*nz)~=-1 &&  u-nx*nz-hc>0 && node(u-nx*nz-ujiang,4)==1% && node(u-nx*nz-hc,4)==0
            l1=(sx-node(u-nx*nz,1))^2+(sy-node(u-nx*nz,2))^2;
            l2=(gx-node(u-nx*nz,1))^2+(gy-node(u-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u-nx*nz;
        neighbor_nodes(end+1)=22;
            end   
     end

    %back-down23
    if u>nx*nz && rem(u,nx*nz)>nx && (round((node(u,2)-xy_res),2)==round(node(u-nx-nx*nz,2),2))...
             &&(round((node(u,3)-z_res),2)==round(node(u-nx-nx*nz,3),2)) && node(u-nx-nx*nz,4)==1 && Q(u-nx-nx*nz)~=-1 && u-nx-nx*nz-hc>0 && node(u-nx-nx*nz-ujiang,4)==1 %&& node(u-nx-nx*nz-hc,4)==0
            l1=(sx-node(u-nx-nx*nz,1))^2+(sy-node(u-nx-nx*nz,2))^2;
            l2=(gx-node(u-nx-nx*nz,1))^2+(gy-node(u-nx-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
       neighbor_nodes(end+1)=u-nx-nx*nz;
       neighbor_nodes(end+1)=23; 
            end     
     end
     %back-right-up24
     if  u>nx*nz && rem(u,nx)~=0 &&  rem(u-1,nx*nz)<nx*(nz-1) && (round((node(u,1)+xy_res),2)==round(node(u+1+nx-nx*nz,1),2))...
            &&(round((node(u,2)-xy_res),2)==round(node(u+1+nx-nx*nz,2),2)) &&(round((node(u,3)+z_res),2)==round(node(u+1+nx-nx*nz,3),2)) && node(u+1+nx-nx*nz,4)==1 && Q(u+1+nx-nx*nz)~=-1 && u+1+nx-nx*nz-hc>0  && node(u+1+nx-nx*nz-ujiang,4)==1 %&& node(u+1+nx-nx*nz-hc,4)==0
            l1=(sx-node(u+1+nx-nx*nz,1))^2+(sy-node(u+1+nx-nx*nz,2))^2;
            l2=(gx-node(u+1+nx-nx*nz,1))^2+(gy-node(u+1+nx-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
       neighbor_nodes(end+1)=u+1+nx-nx*nz;
       neighbor_nodes(end+1)=24;
            end       
     end
    %xy-back-right25
     if u>nx*nz && rem(u,nx)~=0 && (round((node(u,1)+xy_res),2)==round(node(u+1-nx*nz,1),2))...
                && (round((node(u,2)+xy_res),2)==round(node(u+1-nx*nz,2),2)) && node(u+1-nx*nz,4)==1 && Q(u+1-nx*nz)~=-1 && u+1-nx*nz-hc>0 && node(u+1-nx*nz-ujiang,4)==1 %&& node(u+1-nx*nz-hc,4)==0
            l1=(sx-node(u+1-nx*nz,1))^2+(sy-node(u+1-nx*nz,2))^2;
            l2=(gx-node(u+1-nx*nz,1))^2+(gy-node(u+1-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
            neighbor_nodes(end+1)=u+1-nx*nz;
            neighbor_nodes(end+1)=25;
            end
         end
   %back-right-down26
   if u>nx*nz && rem(u,nx)~=0 && rem(u,nx*nz)>nx && (round((node(u,1)-xy_res),2)==round(node(u+1-nx-nx*nz,1),2))...
            &&(round((node(u,2)-xy_res),2)==round(node(u+1-nx-nx*nz,2),2)) &&(round((node(u,3)+z_res),2)==round(node(u+1-nx-nx*nz,3),2)) && node(u+1-nx-nx*nz,4)==1 && Q(u+1-nx-nx*nz)~=-1 && u+1-nx-nx*nz-hc>0 && node(u+1-nx-nx*nz-ujiang,4)==1 %&& node(u+1-nx-nx*nz-hc,4)==0
            l1=(sx-node(u+1-nx-nx*nz,1))^2+(sy-node(u+1-nx-nx*nz,2))^2;
            l2=(gx-node(u+1-nx-nx*nz,1))^2+(gy-node(u+1-nx-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
       neighbor_nodes(end+1)=u+1-nx-nx*nz;
       neighbor_nodes(end+1)=26; 
            end  
        end
  break;    
  end
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

    if(fangxiang==24||fangxiang==25||fangxiang==26)
    
      
        %xz-right-up15
        if rem(u,nx)~=0 && rem(u-1,nx*nz)<nx*(nz-1) && (round((node(u,3)+z_res),2)==round(node(u+nx+1,3),2))...
            &&(round((node(u,1)+xy_res),2)==round(node(u+nx+1,1),2)) && node(u+nx+1,4)==1 && Q(u+nx+1)~=-1 && u+nx+1-hc>0    && node(u+nx+1-ujiang,4)==1   %&& node(u+nx+1-hc,4)==0 
            l1=(sx-node(u+nx+1,1))^2+(sy-node(u+nx+1,2))^2;
            l2=(gx-node(u+nx+1,1))^2+(gy-node(u+nx+1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u+nx+1;
        neighbor_nodes(end+1)=15;
            end
      end
        %xright 16
         if  rem(u,nx)~=0 &&(round((node(u,1)+xy_res),2)==round(node(u+1,1),2))...
             && node(u+1,4)==1 && Q(u+1)~=-1 && u+1-hc>0     && node(u+1-ujiang,4)==1 %&& node(u+1-hc,4)==0  
            l1=(sx-node(u+1,1))^2+(sy-node(u+1,2))^2;
            l2=(gx-node(u+1,1))^2+(gy-node(u+1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u+1;
        neighbor_nodes(end+1)=16;
            end
     end
         %xz-right-down17
         if  rem(u,nx)~=0 &&  rem(u,nx*nz)>nx && (round((node(u,3)-z_res),2)==round(node(u-nx+1,3),2))...
            &&(round((node(u,1)+xy_res),2)==round(node(u-nx+1,1),2)) && node(u-nx+1,4)==1 && Q(u-nx+1)~=-1  && u+1-hc>0  && node(u-nx+1-ujiang,4)==1% && node(u-nx+1-hc,4)==0
            l1=(sx-node(u-nx+1,1))^2+(sy-node(u-nx+1,2))^2;
            l2=(gx-node(u-nx+1,1))^2+(gy-node(u-nx+1,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u-nx+1;
        neighbor_nodes(end+1)=17;
            end 
     end
        %back-up21
        if u>nx*nz && rem(u-1,nx*nz)<nx*(nz-1) && (round((node(u,2)-xy_res),2)==round(node(u+nx-nx*nz,2),2))...
             &&(round((node(u,3)+z_res),2)==round(node(u+nx-nx*nz,3),2)) && node(u+nx-nx*nz,4)==1 && Q(u+nx-nx*nz)~=-1 &&  u+nx-nx*nz-hc>0 && node(u+nx-nx*nz-ujiang,4)==1 %&& node(u+nx-nx*nz-hc,4)==0
            l1=(sx-node(u+nx-nx*nz,1))^2+(sy-node(u+nx-nx*nz,2))^2;
            l2=(gx-node(u+nx-nx*nz,1))^2+(gy-node(u+nx-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
       neighbor_nodes(end+1)=u+nx-nx*nz;
       neighbor_nodes(end+1)=21; 
            end   
    end
        %yback22
     if u>nx*nz && (round(node(u,2),2)==round((node(u-nx*nz,2)+xy_res),2))...
             && node(u-nx*nz,4)==1 && Q(u-nx*nz)~=-1 &&  u-nx*nz-hc>0 && node(u-nx*nz-ujiang,4)==1% && node(u-nx*nz-hc,4)==0
            l1=(sx-node(u-nx*nz,1))^2+(sy-node(u-nx*nz,2))^2;
            l2=(gx-node(u-nx*nz,1))^2+(gy-node(u-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
        neighbor_nodes(end+1)=u-nx*nz;
        neighbor_nodes(end+1)=22;
            end   
     end

        %back-down23
        if u>nx*nz && rem(u,nx*nz)>nx && (round((node(u,2)-xy_res),2)==round(node(u-nx-nx*nz,2),2))...
             &&(round((node(u,3)-z_res),2)==round(node(u-nx-nx*nz,3),2)) && node(u-nx-nx*nz,4)==1 && Q(u-nx-nx*nz)~=-1 && u-nx-nx*nz-hc>0 && node(u-nx-nx*nz-ujiang,4)==1 %&& node(u-nx-nx*nz-hc,4)==0
            l1=(sx-node(u-nx-nx*nz,1))^2+(sy-node(u-nx-nx*nz,2))^2;
            l2=(gx-node(u-nx-nx*nz,1))^2+(gy-node(u-nx-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
       neighbor_nodes(end+1)=u-nx-nx*nz;
       neighbor_nodes(end+1)=23; 
            end     
     end
         %back-right-up24
        if  u>nx*nz && rem(u,nx)~=0 &&  rem(u-1,nx*nz)<nx*(nz-1) && (round((node(u,1)+xy_res),2)==round(node(u+1+nx-nx*nz,1),2))...
            &&(round((node(u,2)-xy_res),2)==round(node(u+1+nx-nx*nz,2),2)) &&(round((node(u,3)+z_res),2)==round(node(u+1+nx-nx*nz,3),2)) && node(u+1+nx-nx*nz,4)==1 && Q(u+1+nx-nx*nz)~=-1 && u+1+nx-nx*nz-hc>0  && node(u+1+nx-nx*nz-ujiang,4)==1 %&& node(u+1+nx-nx*nz-hc,4)==0
            l1=(sx-node(u+1+nx-nx*nz,1))^2+(sy-node(u+1+nx-nx*nz,2))^2;
            l2=(gx-node(u+1+nx-nx*nz,1))^2+(gy-node(u+1+nx-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
       neighbor_nodes(end+1)=u+1+nx-nx*nz;
       neighbor_nodes(end+1)=24;
            end       
     end
            %xy-back-right25
         if u>nx*nz && rem(u,nx)~=0 && (round((node(u,1)+xy_res),2)==round(node(u+1-nx*nz,1),2))...
                && (round((node(u,2)+xy_res),2)==round(node(u+1-nx*nz,2),2)) && node(u+1-nx*nz,4)==1 && Q(u+1-nx*nz)~=-1   && u+1-nx*nz-hc>0 && node(u+1-nx*nz-ujiang,4)==1 %&& node(u+1-nx*nz-hc,4)==0
            l1=(sx-node(u+1-nx*nz,1))^2+(sy-node(u+1-nx*nz,2))^2;
            l2=(gx-node(u+1-nx*nz,1))^2+(gy-node(u+1-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
            neighbor_nodes(end+1)=u+1-nx*nz;
            neighbor_nodes(end+1)=25;
            end
         end
         %back-right-down26
        if u>nx*nz && rem(u,nx)~=0 && rem(u,nx*nz)>nx && (round((node(u,1)-xy_res),2)==round(node(u+1-nx-nx*nz,1),2))...
            &&(round((node(u,2)-xy_res),2)==round(node(u+1-nx-nx*nz,2),2)) &&(round((node(u,3)+z_res),2)==round(node(u+1-nx-nx*nz,3),2)) && node(u+1-nx-nx*nz,4)==1 && Q(u+1-nx-nx*nz)~=-1  && u+1-nx-nx*nz-hc>0 && node(u+1-nx-nx*nz-ujiang,4)==1 %&& node(u+1-nx-nx*nz-hc,4)==0
            l1=(sx-node(u+1-nx-nx*nz,1))^2+(sy-node(u+1-nx-nx*nz,2))^2;
            l2=(gx-node(u+1-nx-nx*nz,1))^2+(gy-node(u+1-nx-nx*nz,2))^2;
            wn=sqrt(l2-((l2+l3-l1)/2/la3)^2);
            if wn<w
       neighbor_nodes(end+1)=u+1-nx-nx*nz;
       neighbor_nodes(end+1)=26; 
            end  
        end
    break;
    end
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
 
     end
    consider_points=[];
    long=length(neighbor_nodes);
    if long==0
        wdemaya=88888888;
        i;
        a=i+1;
        i=a;
      
        continue;    
    end
    for i2=1:(long/2)
       consider_points(i2)=neighbor_nodes(2*i2-1); 
    end
    
    %得到领点与方向  求取离随机点最近点为下一个点
    dist=[];
    for i1=1:numel(consider_points)
        pp=[node(consider_points(i1),1) node(consider_points(i1),2) node(consider_points(i1),3)];
        dist(i1)=euc_dist_3d(pp, q_rand);
    end
    [v,weizhi]=min(dist);
    u=consider_points(weizhi);
    Q(u)=-1; 
    u;
    node(u,1);
    node(u,2);
    node(u,3);
    q_new.config=[node(u,1) node(u,2) node(u,3) ];
    q_new.parent = idx;
    nodes = [nodes q_new];
    nf(length(nodes))=neighbor_nodes(2*weizhi);
    
    
 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%寻找最短路径
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
D = [];
for j = 1:length(nodes)
    tmp_dist = euc_dist_3d(nodes(j).config, q_goal.config);%每个点到终点距离？
    D = [D tmp_dist];
end
[val1, idx1] = min(D);
q_goal.parent = idx1;%作用??添加目标点的父节点？
q_end = q_goal;%循环起始点 倒推路径
nodes = [nodes q_goal];
temp_path=[];
nf(idx1);
num_expanded=0;
while q_end.parent ~= 0%父节点不为0
    u=round((q_end.config(1)+xy_res)/xy_res+q_end.config(2)/xy_res*nx*nz+q_end.config(3)/z_res*nx);
    temp_path=[node(u,1:3);temp_path];
    num_expanded=num_expanded+1;
    start = q_end.parent;%父节点
    q_end = nodes(start);
end
path=[];
path=[node(start_node_ind,1:3);temp_path]
u=999999999
nf(idx1)
num_expanded=nf(idx1)


 end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%euc函数
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function d = euc_dist_3d(q1,q2)
d = sqrt((q1(1)-q2(1))^2 + (q1(2)-q2(2))^2 + (q1(3)-q2(3))^2);
end
