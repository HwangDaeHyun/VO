clear all;
close all;
warning off;

imds =imageDatastore('~/Matlab/Triangulation/testset');
images= cell(1,numel(imds.Files));
for i = 1: numel(imds.Files)
    I = readimage(imds,i);
    %I = imresize(I,0.5);
    images{i} = I;
end
I1 = images{200};
I2 = images{205};
figure
imshowpair(I1, I2, 'montage');
title('Original Images');

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
%========

I1 = Image_undistortion(I1);
I2 = Image_undistortion(I2);
figure
imshowpair(I1, I2, 'montage');
title('Undistorted Images');

% Detect feature points
imagePoints1 = detectMinEigenFeatures(I1, 'MinQuality', 0.01);


% Create the point tracker
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);

% Initialize the point tracker
imagePoints1 = imagePoints1.Location;
initialize(tracker, imagePoints1, I1);

% Track the points
[imagePoints2, validIdx] = step(tracker, I2);
matchedPoints1 = imagePoints1(validIdx, :);
matchedPoints2 = imagePoints2(validIdx, :);

% Visualize correspondences
figure
showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2);
title('Tracked Features');

% Estimate the fundamental matrix
[E, epipolarInliers] = estimateEssentialMatrix(...
    matchedPoints1, matchedPoints2, cameraParams, 'Confidence', 99.99);

% Find epipolar inliers
inlierPoints1 = matchedPoints1(epipolarInliers, :);
inlierPoints2 = matchedPoints2(epipolarInliers, :);

% Display inlier matches
figure
showMatchedFeatures(I1, I2, inlierPoints1, inlierPoints2);
title('Epipolar Inliers');

[orient, loc] = relativeCameraPose(E, cameraParams, inlierPoints1, inlierPoints2);

% Detect dense feature points. Use an ROI to exclude points close to the
% image edges.
roi = [30, 30, size(I1, 2) - 30, size(I1, 1) - 30];
imagePoints1 = detectMinEigenFeatures(I1, ...
    'MinQuality', 0.001);
updateView
% Create the point tracker
% tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);

% Initialize the point tracker
% imagePoints1 = imagePoints1.Location;
% initialize(tracker, imagePoints1, I1);

% Track the points
% [imagePoints2, validIdx] = step(tracker, I2);
% matchedPoints1 = imagePoints1(validIdx, :);
% matchedPoints2 = imagePoints2(validIdx, :);

% Compute the camera matrices for each position of the camera
% The first camera is at the origin looking along the Z-axis. Thus, its
% rotation matrix is identity, and its translation vector is 0.
camMatrix1 = cameraMatrix(cameraParams, eye(3), [0 0 0]);

% Compute extrinsics of the second camera
[R, t] = cameraPoseToExtrinsics(orient, loc);
camMatrix2 = cameraMatrix(cameraParams, R, t);

% Compute the 3-D points
points3D = triangulate(inlierPoints1, inlierPoints2, camMatrix1, camMatrix2);
idx = points3D(:,3) > 0;
points3D = points3D(idx, :);


% Get the color of each reconstructed point
numPixels = size(I1, 1) * size(I1, 2);
allColors = reshape(I1, [numPixels, 1]);
colorIdx = sub2ind([size(I1, 1), size(I1, 2)], round(matchedPoints1(:,2)), ...
    round(matchedPoints1(:, 1)));
color = allColors(colorIdx, :);

% Create the point cloud
ptCloud = pointCloud(points3D);

% Visualize the camera locations and orientations
cameraSize = 0.3;
figure
plotCamera('Size', cameraSize, 'Color', 'r', 'Label', '1', 'Opacity', 0);
hold on
grid on
plotCamera('Location', loc, 'Orientation', orient, 'Size', cameraSize, ...
    'Color', 'b', 'Label', '2', 'Opacity', 0);

% Visualize the point cloud
pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);

% Rotate and zoom the plot
camorbit(0, -30);
camzoom(2);

% Label the axes
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis')
zlim([-10, 70]);
ylim([-100 100]);
xlim([-100 100]);
title('Up to Scale Reconstruction of the Scene');