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

function score = cornerStatistics(img,img_angle,img_weight,cidx)

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
  bin = max(floor(vec_angle(i)/(pi/bin_num)),0)+1;
  angle_hist(bin) = angle_hist(bin)+vec_weight(i);
end

% find modes of smoothed histogram
[modes,angle_hist_smoothed] = findModesMeanShift(angle_hist,2);

% if only one or no mode => return invalid corner
if size(modes,1)<=1
  stat = zeros(1,3);
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

% set statistics: orientations
stat(1,1:2) = modes(:,3);

% set statistics: score
if delta_angle<=0.5 || max(modes(:,2))>10*min(modes(:,2))
  stat = zeros(1,3);
else
  stat(1,3) = cornerCorrelationScore(img,img_weight,stat(1,1),stat(1,2),cidx);
end
