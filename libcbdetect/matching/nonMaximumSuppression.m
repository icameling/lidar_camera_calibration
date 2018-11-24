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

function maxima = nonMaximumSuppression(img,n,tau,margin)

% extract parameters
width  = size(img,2);
height = size(img,1);

% init maxima list
maxima = [];

% non maximum suppression
for i=n+1+margin:n+1:width-n-margin
  for j=n+1+margin:n+1:height-n-margin
    
    maxi   = i;
    maxj   = j;
    maxval = img(j,i);

    for i2=i:i+n
      for j2=j:j+n
        currval = img(j2,i2);
        if currval>maxval
          maxi   = i2;
          maxj   = j2;
          maxval = currval;
        end
      end
    end

    failed = 0;
    for i2=maxi-n:min(maxi+n,width-margin)
      for j2=maxj-n:min(maxj+n,height-margin)
        currval = img(j2,i2);
        if currval>maxval && (i2<i || i2>i+n || j2<j || j2>j+n)
          failed = 1;
          break;
        end
      end
      if failed
        break;
      end
    end
    if maxval>=tau && ~failed
      maxima = [maxima; maxi maxj];
    end
  end
end
