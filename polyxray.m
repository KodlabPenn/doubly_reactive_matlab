% polyxray(x,y,b, v) computes the intersection of the boundary of a
% polygon, with vertex coordinates x and y, and a ray, defined by a base
% point b on the line and a direction vector v.  
%
% Input:
%   x, y : Vertex coordinates of a polygon
%   b, v : Ray parameters starting at the base point b and in the direction of v 
% Output:
%   cx, cy : Vertex coordinates of the intersecting points of the polygon
%            boundary and ray
% Usage:
%   x = [0 1 1 0];
%   y = [0 0 1 1];
%   b = [0.5 0.5];
%   v = [cos(pi/6), sin(pi/6)];
%   [cx, cy] = polyxray(x, y, b, v);
%   figure, hold on;
%   patch(x,y,'w');
%   scatter(cx, cy, 60, 'r');
%   axis equal;

function  [cx, cy] = polyxray(x, y, b, v)
% Author: Omur Arslan - omur@seas.upenn.edu
% Date: August 19th, 2015 Wednesday

% Find the intersection with the line containing the input ray
[cx, cy] = polyxline(x, y, b, [-v(2), v(1)]);
% Only return the intersection points on the ray
a = (cx - b(1))*v(1) + (cy - b(2))*v(2);
cx = cx(a>=0);
cy = cy(a>=0);
