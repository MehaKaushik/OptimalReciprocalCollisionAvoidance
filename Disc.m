function [ val ] = Disc( p,q,r )
% An open disc of radius r centered at p
% val would return 1 if q belongs inside the disc of radius r centered at p
distance = ((p(1)-q(1))^2+(p(2)-q(2))^2);
if r^2 > distance
    val = 1;
elseif r^2 <= distance 
    val = 0;
end
end

