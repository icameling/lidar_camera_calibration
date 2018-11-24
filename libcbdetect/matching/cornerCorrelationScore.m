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

function score = cornerCorrelationScore(img,img_weight,v1,v2)

% center
c = ones(1,2)*(size(img_weight,1)+1)/2;

% compute gradient filter kernel (bandwith = 3 px)
img_filter = -1*ones(size(img_weight,1),size(img_weight,2));
for x=1:size(img_weight,2)
  for y=1:size(img_weight,1)
    p1 = [x y]-c;
    p2 = p1*v1'*v1;
    p3 = p1*v2'*v2;
    if norm(p1-p2)<=1.5 || norm(p1-p3)<=1.5
      img_filter(y,x) = +1;
    end
  end
end

% convert into vectors
vec_weight = img_weight(:);
vec_filter = img_filter(:);

% normalize
vec_weight = (vec_weight-mean(vec_weight))/std(vec_weight);
vec_filter = (vec_filter-mean(vec_filter))/std(vec_filter);

% compute gradient score
score_gradient = max(sum(vec_weight.*vec_filter)/(length(vec_weight)-1),0);

% create intensity filter kernel
template = createCorrelationPatch(atan2(v1(2),v1(1)),atan2(v2(2),v2(1)),c(1)-1);

% checkerboard responses
a1 = sum(template.a1(:).*img(:));
a2 = sum(template.a2(:).*img(:));
b1 = sum(template.b1(:).*img(:));
b2 = sum(template.b2(:).*img(:));

% mean
mu = (a1+a2+b1+b2)/4;

% case 1: a=white, b=black
score_a = min(a1-mu,a2-mu);
score_b = min(mu-b1,mu-b2);
score_1 = min(score_a,score_b);

% case 2: b=white, a=black
score_a = min(mu-a1,mu-a2);
score_b = min(b1-mu,b2-mu);
score_2 = min(score_a,score_b);

% intensity score: max. of the 2 cases
score_intensity = max(max(score_1,score_2),0);

% final score: product of gradient and intensity score
score = score_gradient*score_intensity;
