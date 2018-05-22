%============================= instrinsic
fx = 458.654;
fy = 457.296;
cx = 367.215;
cy = 248.375;
skew_c = 0;
%=============================

%============================= distortion_coef
K1 = -0.28340811;
K2 = 0.07395907;
P2= 0.00019359;
P1 = 1.76187114e-05;
%=============================
%====make camera object
intrinsicMatrix = [fx 0 0 ; skew_c fy 0 ; cx cy 1];
radialDistortion = [K1 K2];
tangentialDistortion = [P1 P2];
cameraParams = cameraParameters('IntrinsicMatrix', intrinsicMatrix , 'RadialDistortion' , radialDistortion, ...
    'TangentialDistortion', tangentialDistortion);



imds =imageDatastore('~/Matlab/Triangulation/testset');


% Convert the images to grayscale.
images = cell(1, numel(imds.Files));
for i = 1:numel(imds.Files)
    images{i}= readimage(imds, i);
    
end

% Undistort the first image.
I = undistortImage(images{1}, cameraParams);

% Detect features. Increasing 'NumOctaves' helps detect large-scale
prevPoints = detectMinEigenFeatures(I,'MinQuality', 0.001);

% view.
viewId = 1;
vSet = viewSet;
vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', ...
    eye(3, 'like', prevPoints.Location), 'Location', ...
    zeros(1, 3, 'like', prevPoints.Location));

prevPoints = prevPoints.Location;
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 6);
initialize(tracker, prevPoints, I);




% Track the points across all views.
for i = 2:50
    % Read and undistort the current image.
    I = undistortImage(images{i}, cameraParams);
    
    % Track the points.
    [currPoints, validIdx] = step(tracker, I);
    vSet = updateConnection(vSet, 1,i, 'Matches',zeros(0,2));
vSet = updateView(vSet, 1, 'Points', prevPoints);
    % Clear the old matches between the points.
    if i < 50
        vSet = updateConnection(vSet, i, i+1, 'Matches', zeros(0, 2));
    end
    vSet = updateView(vSet, i, 'Points', currPoints);
    
    % Store the point matches in the view set.
    matches = repmat((1:size(prevPoints, 1))', [1, 2]);
    matches = matches(validIdx, :);
    vSet = updateConnection(vSet, i-1, i, 'Matches', matches);
end

% Find point tracks across all views.
%matchFeatures
tracks = findTracks(vSet);

% Find point tracks across all views.
camPoses = poses(vSet);

% Triangulate initial locations for the 3-D world points.
xyzPoints = triangulateMultiview(tracks, camPoses,...
    cameraParams);

% Refine the 3-D world points and camera poses.
[xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(...
    xyzPoints, tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
    'PointsUndistorted', true);
% Display the refined camera poses.
figure;
plotCamera(camPoses, 'Size', 0.2);
hold on

% Exclude noisy 3-D world points.
goodIdx = (reprojectionErrors < 5);

% Display the dense 3-D world points.
pcshow(xyzPoints(goodIdx, :), 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
grid on
hold off

% Specify the viewing volume.
% loc1 = camPoses.Location{1};
% xlim([loc1(1)-5, loc1(1)+4]);
% ylim([loc1(2)-5, loc1(2)+4]);
% zlim([loc1(3)-1, loc1(3)+20]);
camorbit(0, -30);

title('Dense Reconstruction');

