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

function corners = scoreCorners(img,img_angle,img_weight,corners,radius)

width  = size(img,2);
height = size(img,1);

% for all corners do
for i=1:size(corners.p,1)
  
  % corner location
  u = round(corners.p(i,1));
  v = round(corners.p(i,2));
  
  % compute corner statistics @ radius 1
  for j=1:length(radius)
    score(j) = 0;
    if u>radius(j) && u<=width-radius(j) && v>radius(j) && v<=height-radius(j)
      img_sub        = img(v-radius(j):v+radius(j),u-radius(j):u+radius(j));
      img_angle_sub  = img_angle(v-radius(j):v+radius(j),u-radius(j):u+radius(j));
      img_weight_sub = img_weight(v-radius(j):v+radius(j),u-radius(j):u+radius(j));
      score(j)       = cornerCorrelationScore(img_sub,img_weight_sub,corners.v1(i,:),corners.v2(i,:));
    end
  end
  
  % take highest score
  corners.score(i,1) = max(score);
end
