function [corners,chessboards,matching] = startMatching(I)

% for all cameras, recover chessboards
for c=1:length(I)
  corners{c}     = findCorners(I{c},0.02,1);
  chessboards{c} = chessboardsFromCorners(corners{c});
end

% matching
matching = matchChessboards(chessboards,corners);  
