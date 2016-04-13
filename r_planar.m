function pts = r_planar(n, lengths, theta)
   % This function calculate end position of each link in an R-planar robot
   % with n-links.
   % 'length' is a 1xn vector of length of each
   % 'theta' is a 1xn vector of the current joint values
   % 'pts' store the cend-oordinate of each link 
   %       [x1 y1; x2 y2; ... ...; xn yn]
   
   d = zeros(1, n);
   a = lengths;
   alpha = zeros(1, n);
   offset = zeros(1, n);
   
   pts = zeros(n, 2);
        
    for i = 1 : n        
        T = fkine_dh(i, theta(1:i), d(1:i), a(1:i), alpha(1:i), offset(1:i)); 
        pts(i, :) = (T(1:2, 4))';
    end
end