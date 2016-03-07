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
    %plot(agentX(i),agentY(i),'*');
    circle(agentX(i),agentY(i),radius(i));
    hold on;
end    

% for each of the agent, we need a set of velocities which is possible for
% it, next from that set we need the ones which avoid collision
velocityX=[];
velocityY=[];
sample=5;
for i=1:n
    vx = (velx).*rand(sample,1);  %sample=5 random velocities, change this value for more optimised solutions of lp
    vy = (vely).*rand(sample,1); 
    velocityX=[velocityX,vx];
    velocityY=[velocityY,vy];
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

%CA = cell(n,n,500,2);
CA=[];
CA2=[];
for i=1:n
    VOij=[];
    MdlKDT = KDTreeSearcher(agents);
    IdxKDT = rangesearch(MdlKDT,agents,(x+y)*0.05);    % x*0.05 is the radius in which it looks for the neighbours... change it suitably
    l=1;
    for j=1:length(IdxKDT{i})
        if flag(i,IdxKDT{i}(j))==0
            if IdxKDT{i}(j)~=i
                for o1=1:sample
                    for o2=1:sample
                        k=VelocityObstacle([velocityX(o1,i)-velocityX(o2,(IdxKDT{i}(j)));velocityY(o1,i)-velocityY(o2,(IdxKDT{i}(j)))],agents(IdxKDT{i}(j),:),agents(j,:),radius(i),radius(IdxKDT{i}(j),:),t)
                        if k==0
                            CA=[CA;[velocityX(o1,i)-velocityX(o2,IdxKDT{i}(j)) , velocityY(o1,i)-velocityY(o2,IdxKDT{i}(j))]]; %collision avoiding velocities
                            %VO(i,j)=[VOij;[velocityX(o1,i)-velocityX(o2,IdxKDT{i}(j));velocityY(o1,i)-velocityY(o2,IdxKDT{i}(j))]];       
                            %VOij{i}{j} = [VOij,[velocityX(o1,i)-velocityX(o2,IdxKDT{i}(j));velocityY(o1,i)-velocityY(o2,IdxKDT{i}(j))]]; %???
                            l=l+1;
                        end
                    end
                    %CA2(i,j,o1) = CA;
                    %CA=[];h
                    CA2{o1} = CA;           % every o1 has set of 1Xsample array % this itself is done sample times
                    CA=[];
                end
                
                %ORCAij = ORCA(CA2{i},[velx/9;vely/10],[velx/10;vely/9],velx,vely);
                %similarly make an4d array of ORCAij
                %now you have lines
                %apply lp on lines
                %ORCAij(i,j,o1,:)=[];
                
            end
        end
    end
    CAi{i}=CA2;
    CA2=[];
end


for i=1:n
    l=CAi{i};
    for o1=1:sample
        if ~isempty(l)
            m=l{o1};
            if ~isempty(m)
                ORCAij = ORCA(m,i,velocityX,velocityY,velx,vely,velocityX,velocityY);     
                figure(2)
                hold off;
                %LP(ORCAij);
            end
            %         see if lengthORCAij is equal to ORCAji... if it is not then
            %             solve this as LP problem
            %         make line equations of ORCA sets : they would be constrains
            %         minimisation function would be v-vpref
            %         from the solution choose va_new, move the robot with va_new for
            %         time t.. then reiterate
            
        end
        
    end
    
end
