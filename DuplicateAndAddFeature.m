function ret = DuplicateAndAddFeature(past, d_image, thresh_hold)
newPoints = detectMinEigenFeatures(d_image);
newPoints = newPoints.selectStrongest(500);
newPoints = newPoints.Location;
[k,d] = dsearchn( double(past),double(newPoints));
idx = d > thresh_hold ;

l_f = newPoints(idx, :);
sz_1 = size(past(:,1),1);
sz_2 = size(l_f(:,1),1 );
sz = sz_1+ sz_2;

rTmp = zeros(sz,2);

rTmp(1:sz_1, :) = past;
rTmp(sz_1+1:end , :) = l_f;

t = struct;
t.liveIdx = sz_1;
t.feats = rTmp;
t.sz = sz;
ret = t;
end