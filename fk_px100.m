% Forward Kinematics Function that takes alpha, a, d, and theta as input
% and gives the transformation matrix using output. 
function T = fk_px100(a, alpha, d, theta)
    T = [cos(theta), -sin(theta), 0, a;
         sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);
         sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha);
         0, 0, 0, 1];
end

