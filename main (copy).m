clc; clear all; close all;

% Object positions initialization

prompt =  'Enter the number of agents';
n = input(prompt);
t=1;
prompt2 = 'Enter the maximum x axis value for your configuration space';
x = input(prompt2);

prompt3 = 'Enter the maximum y axis value for your configuration space';
y = input(prompt3);
%set the range of velocities as per the axis decided

velx = x/100;
vely = y/100;
% randomly generate the positions for the n agents
agentX = (x).*rand(n,1);           % returns a vector of length n containing x coordiante of each agent
agentY = (y).*rand(n,1);           %returns a vector of length n containing y coordiante of each agent
radius = (0.5-0.3).*rand(n,1) + 0.3;         % returns a vector of length n containing radius of each agent, (0.2 is the max radius & 0.001 the minimum i.e radius ranges from 0.001 to 0.2)

%ploting the positions of agents ( if required while debugging)
figure(1)
for i =1:n
    plot(agentX(i),agentY(i),'*');
    hold on;
end    

% for each of the agent, we need a set of velocities which is possible for
% it, next from that set we need the ones which avoid collision
velocityX=[];
velocityY=[];
for i=1:n
    vx = (velx).*rand(500,1);  %20 random velocities, change this value for more optimised solutions of lp
    vy = (vely).*rand(500,1); 
    velocityX=[velocityX;vx];
    velocityY=[velocityY;vy];
end

for i=1:n
    agents(i,1) = agentX(i);
    agents(i,2)= agentY(i);
end



% now for all agents... calculate kd tree members.. perform orca with all..
% calculate the feasible region


flag=[];
for i=1:n
    for j=1:n
        if i~=j
            flag(i,j)=0;
        else
            flag(i,j)=1;
        end
    end
end
CA=[];
for i=1:n
    VOij=[];
    MdlKDT = KDTreeSearcher(agents);
    IdxKDT = rangesearch(MdlKDT,agents,(x+y)*0.05);    % x*0.05 is the radius in which it looks for the neighbours... change it suitably
    for j=1:length(IdxKDT{i})
        if flag(i,IdxKDT{i}(j))==0
            if IdxKDT{i}(j)~=i
                k=VelocityObstacle([velocityX(i)-velocityX(IdxKDT{i}(j));velocityY(i)-velocityY(IdxKDT{i}(j))],agents(IdxKDT{i}(j),:),agents(j,:),radius(i),radius(IdxKDT{i}(j),:),t)
                if k==0
                    VOij=[VOij;[velocityX(i)-velocityX(IdxKDT{i}(j)),velocityY(i)-velocityY(IdxKDT{i}(j))]];
                end
                %setORCA = ( VOab,vAopt,vBopt)
                %now I have a set of collision avoiding velocities
                % for each i, calclate ORCA with agents(IdxKDT)               
%                 if k==0
%                     ORCAij = ORCA(VOij,[velx/9;vely/10],[velx/10;vely/9],velx,vely);
%                     CA{i}{IdxKDT{i}(j)} = ORCAij;
%                 end
                
%                 figure(2)
%                 hold off;
                %axis([0,vx,0,vy]);
%                 if flag(IdxKDT{i}(j),i)==1
%                     if ~isnull(CA{i}) && ~isnull(CA{IdxKDT{i}(j)}) 
%                         if length(CA{i}) ~= length(CA{IdxKDT{i}(j)})  %??????????????? missing condition over D
%                             
%                         end
%                     end
%                 end
%                 flag(i,IdxKDT{i}(j))=1;
            end
        end
    end
end