function ORCAab = ORCA( VOab,vAopt,Vagents,vx,vy )
% sample test case ORCA([1,2;1,5;2,6;15,32;23,222;41,32;-12,-1;-2,3;3,-43],[2,4],[1,2])
% this is a code for ORCA for 2 agents
%VOab: set of velocity obstacles of a wrt b  / the ones in which collision
%occurs
%u = ( arg min v − (voptA − vB opt) ) − (vA opt − vB opt), v∈∂ VOτA|B
min=0;
min_index=1;


for i =1:size(VOab)
         k=abs(VOab(i)-(vAopt-vBopt));
             if min > k
                 min = k;
                 min_index=k;
             end             
end
% doing the same using linear programming ??????????????? 
hi = 'VOab is'
VOab()

u=VOab(min_index)-vAopt-vBopt; 
            
ORCAab=[];
j=1;
v = [-u(2);u(1)];                      % calculating the n vector: its direction cosines(normalized) wud be n1
n1= v/norm(v);
%l=(vAopt(1)+1/2*u(1))*n1(1);           % direction vector(unnormalized) just that n passes through vAopt+1/2u
%m=(vAopt(2)+1/2*u(2))*n1(2);
%n=[l;m];
n=n1;
for i = 1:size(VOab)
    if dot((VOab(i)-(vAopt+(1/2)*u)),n)>=0
        ORCAab(j,:)=VOab(i,:);
        j=j+1;
    end
end
if ~isempty(ORCAab)
    figure(2)
    plot(ORCAab(:,1),ORCAab(:,2));
    k = ORCAab(:,1)
    l = ORCAab(:,2)
    axis([0,vx,0,vy]);
    hold on;
    pause();
end
end

