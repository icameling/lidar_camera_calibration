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

function col = colorFromIndex(idx)

idx = idx-1;
idx = mod(idx,18);

switch idx
  case 0,  col = [0.0 0.0 1.0];
  case 1,  col = [0.0 1.0 0.0];
  case 2,  col = [1.0 0.0 0.0];
  case 3,  col = [0.0 1.0 1.0];
  case 4,  col = [1.0 0.0 1.0];
  case 5,  col = [1.0 1.0 0.0];
  case 6,  col = [0.0 1.0 0.5];
  case 7,  col = [0.0 0.5 1.0];
  case 8,  col = [0.0 0.7 0.7];
  case 9,  col = [0.5 0.0 1.0];
  case 10, col = [1.0 0.0 0.5];
  case 11, col = [0.7 0.0 0.7];
  case 12, col = [0.5 1.0 0.0];
  case 13, col = [1.0 0.5 0.0];
  case 14, col = [0.7 0.7 0.0];
  case 15, col = [0.5 1.0 1.0];
  case 16, col = [1.0 0.5 1.0];
  case 17, col = [1.0 1.0 0.5];
end
