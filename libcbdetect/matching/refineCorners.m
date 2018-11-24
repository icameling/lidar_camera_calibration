% Copyright 2012. All rights reserved.
% Author: Andreas Geiger
%         Institute of Measurement and Control Systems (MRT)
%         Karlsruhe Institute of Technology (KIT), Germany

% This is free software; you can redistribute it and/or modify it under the
% terms of the GNU General Public License as published by the Free Software
% Foundation; either version 3 of the License, or any later version.

% This software is distributed in the hope that it will be useful, but WITHOUT ANY
% WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
% PARTICULAR PURPOSE. See the GNU General Public License for more details.

% You should have received a copy of the GNU General Public License along with
% this software; if not, write to the Free Software Foundation, Inc., 51 Franklin
% Street, Fifth Floor, Boston, MA 02110-1301, USA 

function corners = refineCorners(img_du,img_dv,img_angle,img_weight,corners,r)

% image dimensions
width  = size(img_du,2);
height = size(img_dv,1);

% init orientations to invalid (corner is invalid iff orientation=0)
corners.v1 = zeros(size(corners.p,1),2);
corners.v2 = zeros(size(corners.p,1),2);

% for all corners do
for i=1:size(corners.p,1)
  
  % extract current corner location
  cu = corners.p(i,1);
  cv = corners.p(i,2);
  
  % estimate edge orientations
  img_angle_sub  = img_angle(max(cv-r,1):min(cv+r,height),max(cu-r,1):min(cu+r,width));
  img_weight_sub = img_weight(max(cv-r,1):min(cv+r,height),max(cu-r,1):min(cu+r,width));
  [v1,v2] = edgeOrientations(img_angle_sub,img_weight_sub);
   
  corners.v1(i,:) = v1;
  corners.v2(i,:) = v2;
  
  % continue, if invalid edge orientations
  if v1(1)==0 && v1(2)==0 || v2(1)==0 && v2(2)==0
    continue;
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % corner orientation refinement %
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  A1 = zeros(2,2);
  A2 = zeros(2,2);
  
  for u=max(cu-r,1):min(cu+r,width)
    for v=max(cv-r,1):min(cv+r,height)
      
      % pixel orientation vector
      o = [img_du(v,u) img_dv(v,u)];
      if norm(o)<0.1
        continue;
      end
      o = o/norm(o);
      
      % robust refinement of orientation 1
      if abs(o*v1')<0.25 % inlier?
        A1(1,:) = A1(1,:) + img_du(v,u) * [img_du(v,u) img_dv(v,u)];
        A1(2,:) = A1(2,:) + img_dv(v,u) * [img_du(v,u) img_dv(v,u)];
      end
      
      % robust refinement of orientation 2
      if abs(o*v2')<0.25 % inlier?
        A2(1,:) = A2(1,:) + img_du(v,u) * [img_du(v,u) img_dv(v,u)];
        A2(2,:) = A2(2,:) + img_dv(v,u) * [img_du(v,u) img_dv(v,u)];
      end
      
    end
  end
  
  % set new corner orientation
  [v1,foo1] = eig(A1); v1 = v1(:,1)'; corners.v1(i,:) = v1;
  [v2,foo2] = eig(A2); v2 = v2(:,1)'; corners.v2(i,:) = v2;
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %  corner location refinement  %
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  G = zeros(2,2);
  b = zeros(2,1);
  for u=max(cu-r,1):min(cu+r,width)
    for v=max(cv-r,1):min(cv+r,height)
      
      % pixel orientation vector
      o = [img_du(v,u) img_dv(v,u)];
      if norm(o)<0.1
        continue;
      end
      o = o/norm(o);
      
      % robust subpixel corner estimation
      if u~=cu || v~=cv % do not consider center pixel
        
        % compute rel. position of pixel and distance to vectors
        w  = [u v]-[cu cv];
        d1 = norm(w-w*v1'*v1);
        d2 = norm(w-w*v2'*v2);
        
        % if pixel corresponds with either of the vectors / directions
        if d1<3 && abs(o*v1')<0.25 || d2<3 && abs(o*v2')<0.25
          du = img_du(v,u);
          dv = img_dv(v,u);
          H = [du dv]'*[du dv];
          
          G = G + H;
          b = b + H*[u v]';
        end
      end
    end
  end

  % set new corner location if G has full rank
  if rank(G)==2
    corner_pos_old = corners.p(i,:);
    corner_pos_new = (G\b)';
    corners.p(i,:) = corner_pos_new;
    
    % set corner to invalid, if position update is very large
    if norm(corner_pos_new-corner_pos_old)>=4
      corners.v1(i,:) = [0 0];
      corners.v2(i,:) = [0 0];
    end
    
  % otherwise: set corner to invalid
  else
    corners.v1(i,:) = [0 0];
    corners.v2(i,:) = [0 0];
  end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [v1,v2] = edgeOrientations(img_angle,img_weight)

% init v1 and v2
v1 = [0 0];
v2 = [0 0];

% number of bins (histogram parameter)
bin_num = 32;

% convert images to vectors
vec_angle  = img_angle(:);
vec_weight = img_weight(:);

% convert angles from normals to directions
vec_angle = vec_angle+pi/2;
vec_angle(vec_angle>pi) = vec_angle(vec_angle>pi)-pi;

% create histogram
angle_hist = zeros(1,bin_num);
for i=1:length(vec_angle)
  bin = max(min(floor(vec_angle(i)/(pi/bin_num)),bin_num-1),0)+1;
  angle_hist(bin) = angle_hist(bin)+vec_weight(i);
end

% find modes of smoothed histogram
[modes,angle_hist_smoothed] = findModesMeanShift(angle_hist,1);

% if only one or no mode => return invalid corner
if size(modes,1)<=1
  return;
end

% compute orientation at modes
modes(:,3) = (modes(:,1)-1)*pi/bin_num;

% extract 2 strongest modes and sort by angle
modes = modes(1:2,:);
[foo idx] = sort(modes(:,3),1,'ascend');
modes = modes(idx,:);

% compute angle between modes
delta_angle = min(modes(2,3)-modes(1,3),modes(1,3)+pi-modes(2,3));

% if angle too small => return invalid corner
if delta_angle<=0.3
  return;
end

% set statistics: orientations
v1 = [cos(modes(1,3)) sin(modes(1,3))];
v2 = [cos(modes(2,3)) sin(modes(2,3))];
