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

function [modes,hist_smoothed] = findModesMeanShift(hist,sigma)
% efficient mean-shift approximation by histogram smoothing

% compute smoothed histogram
hist_smoothed = zeros(1,length(hist));
for i=1:length(hist)
  j                = -round(2*sigma):+round(2*sigma);
  idx              = mod(i+j-1,length(hist))+1;
  hist_smoothed(i) = sum(hist(idx).*normpdf(j,0,sigma));
end

modes = [];

% check if at least one entry is non-zero
% (otherwise mode finding may run infinitly)
if abs(hist_smoothed-hist_smoothed(1))<1e-5
  return;
end

% mode finding
for i=1:length(hist_smoothed)
  j=i;
  while 1
    h0 = hist_smoothed(j);
    j1 = mod(j+1-1,length(hist))+1;
    j2 = mod(j-1-1,length(hist))+1;
    h1 = hist_smoothed(j1);
    h2 = hist_smoothed(j2);
    if h1>=h0 && h1>=h2
      j=j1;
    elseif h2>h0 && h2>h1
      j=j2;
    else
      break;
    end
  end
  if size(modes,1)==0 ||~any(modes(:,1)==j)
    modes = [modes; j hist_smoothed(j)];
  end
end

% sort
[val idx] = sort(modes(:,2),1,'descend');
modes = modes(idx,:);
