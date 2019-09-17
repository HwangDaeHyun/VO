%======Parmaeter=
%@ Param1 = features set : [features1, features2]
%@ Param2 = camera Pose  : [pose1, pose2]
%@ Parma3 = cameraMatrix 
%======
function ret_point = TriangulatePoints(pt1, pt2,pose1, pose2)
    pose1 = pose1';
    pose2 = pose2';
    A =zeros(4,4);
    point3Dset = zeros(numel(pt1(:,1)),3);
    for i=1: numel(pt1(:,1))
    n_p1 = pt1(i,:);
    n_p2 = pt2(i,:);
    A(1,:) =  n_p1(1)*pose1(3,:)-pose1(1,:);
    A(2,:) =  n_p1(2)*pose1(3,:)-pose1(2,:);
    A(3,:) =  n_p2(1)*pose2(3,:)-pose2(1,:);
    A(4,:) =  n_p2(2)*pose2(3,:)-pose2(2,:);
    
    [~,~,V] = svd(A);
    X= V(:,end);
    X= X/X(end);
    point3Dset(i,:) = X(1:3)';

    end
    ret_point =point3Dset;
end

