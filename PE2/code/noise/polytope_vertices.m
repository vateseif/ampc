function w = polytope_vertices(V)
%POLYTOPE_VERTICES Randomly chosen vertex as noise
%   Choses a random vertex of the vertix matrix input as noise

%%% Parse inputs %%%
switch nargin
    case 1
        
    otherwise
        error('Wrong number of inputs!')
end
%%%%%%%%%%%%%%%%%%%
    idx = randi(size(V,1));
    w = V(idx,:)';
end

