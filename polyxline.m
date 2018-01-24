% polyxline(x,y,m,n) computes the intersection of the boundary of a
% polygon, with vertex coordinates x and y, and a line, defined by a point
% m on the line and its normal n.      
%
% Input:
%   x, y : Vertex coordinates of a polygon
%   m, n : Line parameters,  m is a point on the line and n is its normal 
% Output:
%   cx, cy : Vertex coordinates of the intersecting points of the polygon
%            boundary and line
% Usage:
%   x = [0 1 1 0];
%   y = [0 0 1 1];
%   m = [0.5 0.5];
%   n = [cos(pi/6), sin(pi/6)];
%   [cx, cy] = polyxline(x, y, m, n);
%   figure, hold on;
%   patch(x,y,'w');
%   scatter(cx, cy, 60, 'r');
%   axis equal;

function [cx, cy] = polyxline(x,y,m,n)
% Author: Omur Arslan - omur@seas.upenn.edu
% Date: August 09th, 2015 Sunday

if numel(x) ~= numel(y)
    error('Dimension Mismatch! Coordinate arrays should have the same number of elements.');
end

cx = [];
cy = [];
K = numel(x); % Number of vertices
for ck = 1:K
    sk = ((x(ck)- m(1))*n(1) + (y(ck) - m(2))*n(2));
    if (sk == 0)
        cx = [cx; x(ck)];
        cy = [cy; y(ck)];
    else
        cn = mod(ck,K) + 1;
        sn = ((x(cn)- m(1))*n(1) + (y(cn) - m(2))*n(2));
        if ((sk*sn) < 0)
           a = -sn/(sk - sn);
           if ((a >= 0)&&(a<= 1))
                cx = [cx; a*x(ck)+(1-a)*x(cn)];
                cy = [cy; a*y(ck)+(1-a)*y(cn)];
           end
        end
    end
end


