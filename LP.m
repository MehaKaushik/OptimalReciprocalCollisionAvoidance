function [ output_args ] = LP( ORCA )
%take a set of ORCA velocities for n agents: cell of cells
for i=1:length(ORCA)
    orca_i=ORCA(i); % check whether {} 
    % here start the linear programming code... 
    %minimise abs(vpref-v);
    %constraints in this loop
    if ~isempty(orca_i)
        if length(orca_i) >1
            a=orca_i(1);
            b=orca_i(length(orca_i));
            m = (a-b){1}/(a-b){2};                 % check how to get the x and y value from a-b
            
        end
    end
    
end


end

