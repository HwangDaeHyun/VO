
close all;
warning off;
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
%========
%INIT=========================================================================+
%init pyramid level , feature treshold , kernel, detect count , patch size
% startFrame number, feature count
p_level = 1;
feats_thr = 0;
kernel = fspecial('gaussian',3, 0.5);
dCount = 1000;
sFrame_no = 110;
feature_cnt = -1;
redetect_cnt = 0;

zoom.action = 'off';
	

%INIT_END=====================================================================+


% set image directory
imds =imageDatastore('~/Matlab/Triangulation/testset');
images= cell(1,numel(imds.Files));
for i = 1: numel(imds.Files)
    I = readimage(imds,i);
    %I = imresize(I,0.5);
    images{i} = I;
end

s = 150; e = 155; % start & end idx
validIdx = [];
valid_vec = {};
id = 0;

frame.ftids = [];
frames = {};

nextFrame_no = sFrame_no+5;

%% ==========================pre tracking for initialize map ====================================
imagePoints = cell(1, numel(imds.Files));

tracker = vision.PointTracker('MaxBidirectionalError',1,'NumPyramidLevels',5, 'MaxIterations', 20);
imagePoints{1} = detectMinEigenFeatures(images{sFrame_no});
imagePoints{1} = imagePoints{1}.selectStrongest(1000);
imagePoints{1} = imagePoints{1}.Location;
initialize(tracker,imagePoints{1}, images{sFrame_no});


%Track the Points
[imagePoints{2}, validIdx] = step(tracker, images{nextFrame_no});
matchedPoints1 = imagePoints{1}(validIdx,:);
matchedPoints2 = imagePoints{2}(validIdx,:);


uPts_1 = undistortPoints(matchedPoints1 , cameraParams);
uPts_2 = undistortPoints(matchedPoints2 , cameraParams);

%Estimate the fundamental matrix
[E , epipolarInliers] = estimateEssentialMatrix(uPts_1, uPts_2, cameraParams, 'Confidence', 99.99);

%Find epipolar inliers
inlierPoints1 = uPts_1(epipolarInliers, :);
inlierPoints2 = uPts_2(epipolarInliers, :);
[orient, loc] = relativeCameraPose(E, cameraParams, inlierPoints1, inlierPoints2);

[R,t] = cameraPoseToExtrinsics(orient, loc);
camMatrix1 = cameraMatrix(cameraParams, eye(3), [0 0 0]);
camMatrix2 = cameraMatrix(cameraParams, R,t);

points3D = cell(1, numel(imds.Files));
points3D{1} = TriangulatePoints(inlierPoints1, inlierPoints2, camMatrix1, camMatrix2);

%points3D = triangulate(inlierPoints1, inlierPoints2, camMatrix1, camMatrix2);%
idx = points3D{1}(:,3) > 0;
worldPoints = points3D{1}(idx, :);

%undistortPoints
ptCloud = pointCloud(worldPoints);

% Visualize the camera locations and orientations
cameraSize = 0.3;

% figure
% plotCamera('Size', cameraSize, 'Color', 'r', 'Opacity', 0);
% 
% hold on
% plotCamera('Location', loc, 'Orientation', orient, 'Size', cameraSize, ...
%     'Color', 'r', 'Opacity', 0);
% 
% % Visualize the point cloud
% pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
%     'MarkerSize', 45);
% 
% % Rotate and zoom the plot
% camorbit(0, -30);
% camzoom(1);
% hold off


%% reInitialize for tracking
tracker.release();

initialize(tracker,matchedPoints1(epipolarInliers,:), images{sFrame_no});
%estimateWorldCameraPose
cnt =0;
%% tracking for all image
live_idx = size(matchedPoints1(epipolarInliers,:), 1);
t_set = undistortPoints(matchedPoints1(epipolarInliers,:),cameraParams);    %for triangulation with the former key frame
idx_3d= 1;
ridx_3d = 1;
color = 'r';
interval = 1;
isFirst = true;
kf_cnt =1;
camzoom(1.5);
for it =sFrame_no+1 : numel(images)-1
    pIdx = it-sFrame_no;
    
    %% tracking
    [imagePoints{pIdx+1} ,validIdx] = step(tracker, images{it});
    validIdx2 = validIdx(1:live_idx);
    matchedPoints1 = imagePoints{pIdx}(validIdx2,:);
    matchedPoints2 = imagePoints{pIdx+1}(validIdx2,:);
    uPts1 = undistortPoints(matchedPoints2, cameraParams);
    
    %% pose_estimation (p3p)
    [worldOrientation,worldLocation,reprojectErr] = estimateWorldCameraPose(double(uPts1), double(points3D{idx_3d}(validIdx2,:,:)), cameraParams);
    %pose optimization
    options = optimoptions('fminunc', 'MaxFunEvals', 5000);
    [R,t] = cameraPoseToExtrinsics(worldOrientation, worldLocation);
    camMatrix2 = cameraMatrix(cameraParams, R, t);
    %     pose = fminunc(@reprojectenError,  camMatrix, options, double(uPts1),points3D{idx_3d}(validIdx2,:,:));
    
    
    disp(it)
    %% redetect
    if(interval > 5 || numel(imagePoints{pIdx}(validIdx2,:)) < feats_thr)
        color= 'r';
        interval = 1;
        %triangulateMultiview
        
        kf_cnt=kf_cnt+1;
        idx_3d = idx_3d+1;
        [points3D{idx_3d}, err] = triangulate(t_set(validIdx,:), undistortPoints(imagePoints{pIdx+1}(validIdx,:),cameraParams), camMatrix1, camMatrix2);
        
        %draw new 3D Points
        worldPoints = points3D{idx_3d}(live_idx+1:end,:) ;
        wIdx = (worldPoints(:,1) < 50 & worldPoints(:,1)>-20 & worldPoints(:,2) >-20 & worldPoints(:,2) <15 & worldPoints(:,3) < 50 & worldPoints(:,3) >=0);
        worldPoints = worldPoints(wIdx,:,:);
        ptCloud = pointCloud(worldPoints);
        %  pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', 45);
        
        %get new features
        tmp = DuplicateAndAddFeature(imagePoints{pIdx+1}(validIdx,:), images{it}, 3);
        live_idx = tmp.liveIdx;
        imagePoints{pIdx+1} = tmp.feats;
        tracker.release();
        initialize(tracker,imagePoints{pIdx+1}, images{it});
        
        redetect_cnt = redetect_cnt+1;
        t_set = undistortPoints(tmp.feats, cameraParams);
        camMatrix1 = camMatrix2;
        
    else
        color = 'g';
        interval= interval+1;
        
    end
    
    %% draw
    %     xlim([-20 50]);
    %     ylim([-20 15]);
    %     zlim([0 50]);
    
    axis off;
    %  plotCamera('Location',worldLocation, 'Orientation', worldOrientation,'Size', cameraSize, 'Color', color, 'Opacity',0);
    
    imshow(images{it});
    hold on;
    scatter(imagePoints{pIdx+1}(:,1), imagePoints{pIdx+1}(:,2), 'r.');
    hold off;
    %saveas(gcf,strcat('~/Matlab/frame_result/',num2str(it-sFrame_no+1 , '%04d'),'.jpg'));
    
          filename = strcat('~/Matlab/frame_result/',num2str(it-sFrame_no+1 , '%04d'));
    
         print(filename,'-dpng','-r0');
    
    close all;
end
%% find  reprojection err  for pose optimization
function error = reprojectenError(P,p,x)
pose = P';
feature_2d = p;
feature_3d = x;
feature_3d(:,4) = 1;
projection = pose*(feature_3d');
projection_n = projection(1:2,:)./repmat(projection(3,:),2,1);
calc = projection_n - feature_2d';
error = sqrt(sum(sum(calc.^2,1)));

end