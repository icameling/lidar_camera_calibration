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

function template = createCorrelationPatch(angle_1,angle_2,radius)

% width and height
width  = radius*2+1;
height = radius*2+1;

% initialize template
template.a1 = zeros(height,width,'double');
template.a2 = zeros(height,width,'double');
template.b1 = zeros(height,width,'double');
template.b2 = zeros(height,width,'double');

% midpoint
mu = radius+1;
mv = radius+1;

% compute normals from angles
n1 = [-sin(angle_1) cos(angle_1)];
n2 = [-sin(angle_2) cos(angle_2)];

% for all points in template do
for u=1:width
  for v=1:height
    
    % vector
    vec  = [u-mu v-mv];
    dist = norm(vec);
    
    % check on which side of the normals we are
    s1 = vec*n1';
    s2 = vec*n2';

    if s1<=-0.1 && s2<=-0.1
      template.a1(v,u) = normpdf(dist,0,radius/2);
    elseif s1>=0.1 && s2>=0.1
      template.a2(v,u) = normpdf(dist,0,radius/2);
    elseif s1<=-0.1 && s2>=0.1
      template.b1(v,u) = normpdf(dist,0,radius/2);
    elseif s1>=0.1 && s2<=-0.1
      template.b2(v,u) = normpdf(dist,0,radius/2);
    end
  end
end

% normalize
template.a1 = template.a1/sum(template.a1(:));
template.a2 = template.a2/sum(template.a2(:));
template.b1 = template.b1/sum(template.b1(:));
template.b2 = template.b2/sum(template.b2(:));
