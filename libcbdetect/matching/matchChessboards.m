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

function matching = matchChessboards(chessboards,corners)

% try to match checkerboards in multiple images against each other
% warning: experimental
matching = [];
for i=2:length(chessboards)
  matching = [matching; matchAgainstReference(chessboards{1},corners{1},chessboards{i},corners{i})];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function matching = matchAgainstReference(chessboards_ref,corners_ref,chessboards_tar,corners_tar)

% number of chessboards
n_ref = length(chessboards_ref);
n_tar = length(chessboards_tar);

% compute reference and target chessboard center
means_ref = chessboardMeans(chessboards_ref,corners_ref);
means_tar = chessboardMeans(chessboards_tar,corners_tar);

% determine outlier threshold
tau = squareform(pdist(means_ref));
tau = 0.2*max(tau(:));

disp('Matching chessboards ...');

% create plausible correspondences from all possible board combinations
% here a 0 entry means invalid
matchings  = [];

for i_ref=1:n_ref
  for j_ref=1:n_ref
    for i_tar=1:n_tar
      for j_tar=1:n_tar
        
        % check if we have two different boards per image
        if i_ref==j_ref || i_tar==j_tar
          continue;
        end
        
        % compute similarity transformation ref=tar*A+b
        v1 = means_ref(j_ref,:)-means_ref(i_ref,:);
        v2 = means_tar(j_tar,:)-means_tar(i_tar,:);
        s  = norm(v1)/norm(v2);
        r  = acos(min(max((v1*v2')/(norm(v1)*norm(v2)),-1),1)); % min/max for matlab stability
        A  = s*[cos(r) -sin(r);sin(r) cos(r)]';
        b  = -means_tar(i_tar,:)*A+means_ref(i_ref,:);
        
        % project target to reference board coordinates via ref=tar*A+b
        means_tar_2 = means_tar*A+ones(n_tar,1)*b;
        
        % greedily compute matching until hitting outlier threshold
        dist = squareform(pdist([means_ref;means_tar_2]));
        dist = dist(1:size(means_ref,1),size(means_ref,1)+1:end);
        matching = zeros(1,size(means_ref,1));
        
        while 1
          [val idx] = min(dist(:));
          if val>tau
            break;
          end
          [row col] = ind2sub(size(dist),idx);
          
          % same number of rows and columns?
          if size(chessboards_ref{row},1)==size(chessboards_tar{col},1) && ...
             size(chessboards_ref{row},2)==size(chessboards_tar{col},2) || ...
             size(chessboards_ref{row},1)==size(chessboards_tar{col},2) && ...
             size(chessboards_ref{row},2)==size(chessboards_tar{col},1)
            matching(row) = col;
            dist(row,:) = inf;
            dist(:,col) = inf;
            
          % if not => put infinite weight
          else
            dist(row,col) = inf;
          end
        end
        
        % if at least 3 chessboards could be matched
        if length(find(matching>0))>=3
        
          % add matching to matching hypotheses
          matchings(end+1,:) = matching;
        end
      end
    end
  end
end

disp('Scoring hypotheses ...');

% make matching vector unique and sort by number of non-zero entries
[matchings,idx,foo] = unique(matchings,'rows');
nonzeros            = sum(matchings~=0,2);
[nonzeros,idx]      = sort(nonzeros,'descend');
matchings           = matchings(idx,:);

% score hypotheses and compute best fitting rotations
max_score    = -inf;
max_idx      = 0;
max_rotation = [];
for i=1:size(matchings,1)
  [rotation,score] = scoreMatching(matchings(i,:),chessboards_ref,corners_ref,means_ref, ...
                                                  chessboards_tar,corners_tar,means_tar,tau);
  if score>max_score
    max_score    = score;
    max_idx      = i;
    max_rotation = rotation;
  end
end

% no matching found?
if max_idx==0
  
  % return empty matching
  matching = [];
  
else

  % copy matching and rotation to output
  % result will contain:
  % - indices of target chessboards in first row
  % - rotations of target chessboards in second row
  % - columns correspond to reference chessboard indices
  matching(1,:) = matchings(max_idx,:);
  matching(2,:) = max_rotation;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [rotation,score] = scoreMatching(matching,chessboards_ref,corners_ref,means_ref, ...
                                                   chessboards_tar,corners_tar,means_tar,tau)

num_matched = length(find(matching>0));

% compute affine transformation ref=tar*A+b by
% least squares fit to all matched chessboards
H = zeros(num_matched*2,6);
H(1:2:end,1:2) = means_tar(matching(matching>0),:); H(1:2:end,5) = 1;
H(2:2:end,3:4) = means_tar(matching(matching>0),:); H(2:2:end,6) = 1;
x = zeros(num_matched*2,1);
x(1:2:end) = means_ref(matching>0,1);
x(2:2:end) = means_ref(matching>0,2);
y = inv(H'*H)*H'*x;
A = [y(1) y(3); y(2) y(4)];
b = [y(5) y(6)];

score = 0;
rotation = zeros(1,length(matching));

% for all corners in all chessboards in the reference image,
% compute distance to corners in transformed target image
% score is high, if distances are small and many chessboards are matched
% rotation(i) holds the best rotation of the matching(i)'th target board
for i=1:length(matching)
  j=matching(i);
  if j~=0
    [dist,rot] = minimumCornerDistance(chessboards_ref{i},chessboards_tar{j},corners_ref,corners_tar,A,b);
    score = score - dist/tau + 1;
    rotation(i) = rot;
  end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [dist,rot] = minimumCornerDistance(chessboard_ref,chessboard_tar,corners_ref,corners_tar,A,b)

% init distance
dist = inf;
rot  = 0;

% for all rotations do (i=1 => no rotation)
for i=1:4
  p_ref = corners_ref.p(chessboard_ref(:),:);
  p_tar = corners_tar.p(chessboard_tar(:),:)*A + ones(length(chessboard_tar(:)),1)*b;
  
  % check if chessboards have same size
  if size(chessboard_ref,1) == size(chessboard_tar,1) && ...
     size(chessboard_ref,2) == size(chessboard_tar,2)
    
    % compute mean distance, and save, if better than before
    dist_ = p_ref-p_tar;
    dist_ = mean(sqrt(dist_(:,1).^2+dist_(:,2).^2));
    if dist_<dist
      dist = dist_;
      rot  = i;
    end
  end

  % rotate target chessboard clockwise
  chessboard_tar = chessboard_tar(end:-1:1,:)';
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function matching = chessboardMeans(chessboards,corners)

matching = zeros(length(chessboards),2);
for i=1:length(chessboards)
  matching(i,:) = mean(corners.p(chessboards{i}(:),:));
end

