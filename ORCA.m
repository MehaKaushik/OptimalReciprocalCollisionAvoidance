function ORCAa = ORCA( VOab,i,VagentsX,VagentsY,vx,vy,VoptX,VoptY )
% sample test case ORCA([1,2;1,5;2,6;15,32;23,222;41,32;-12,-1;-2,3;3,-43],[2,4],[1,2])
% this is a code for ORCA for 2 agents
%VOab: set of velocity obstacles of a wrt b  / the ones in which collision
%occurs
%u = ( arg min v − (voptA − vB opt) ) − (vA opt − vB opt), v∈∂ VOτA|B
vAopt=[VoptX(i);VoptY(i)]; %???????????????????????????????????????
vBopt=[0;0]; %??????????????????????????????????????????????????????????
vA=[VagentsX(i);VagentsY(i)];
for i=1:size(VOab)
    vB=VOab(i);
    min=0;
    min_index=1;
    for i =1:size(VOab)
        k=abs((VOab(i)-vA)-(vAopt-vBopt));
        if min > k
            min = k;
            min_index=k;
        end
    end
end
u=VOab(min_index)-vAopt-vBopt; 
ORCAab=[];
j=1;
v = [-u(2);u(1)];                      % calculating the n vector: its direction cosines(normalized) wud be n1
n= v/norm(v);
for i = 1:size(VOab)
    if dot((VOab(i)-(vAopt+(1/2)*u)),n)>=0
        ORCAab(j,:)=VOab(i,:);
        j=j+1;
    end
end
ORCAa{i}=ORCAab;

if ~isempty(ORCAab)
    figure(2)
    plot(ORCAab(:,1),ORCAab(:,2));
    axis([0,vx,0,vy]);
    hold on;
    pause();
end
ORCAab=[];
hold off;
end

