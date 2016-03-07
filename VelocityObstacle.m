function [val] = VelocityObstacle( v,pB,pA,rA,rB,T )
% returns a 1 if a velocity belongs to the velocity obstacle cone, for a time
% period T
% i.e v would result in collision at any instant till time T

% τVOA|B = {v | ∃t ∈ [0, T ] :: tv ∈ D(pB − pA, rA + rB)}
val=0;
for t=1:0.01:T             
    val =  Disc(pB-pA,[t*v(1),t*v(2)],rA+rB);
    if val == 1
        break;           %if for any t in the range[0,T], collision occurs then val is set to 1, once val is 1, the for loop breaks,next the function ends.
    end
end
end
