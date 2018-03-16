
% 这个文件用来计算相机外参 还有 手眼标定
%

%%
% Create a set of calibration images.
path = '/home/zcc/projects/matlabProjects/hand_eye_calib/data2';
% path = '/home/zcc/projects/matlabProjects/hand_eye_calib/img/图片/第一组';
 images = imageDatastore(path, 'FileExtensions', '.png');
 
%%
% Detect the checkerboard corners in the images.
[imagePoints,boardSize] = detectCheckerboardPoints(images.Files);

%%
% Generate the world coordinates of the checkerboard corners in the
% pattern-centric coordinate system, with the upper-left corner at (0,0).
% The square size is in millimeters.
squareSize = 12;
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

%%
% Calibrate the camera.
I = readimage(images,1); 
imageSize = [size(I,1), size(I,2)];
cameraParams = estimateCameraParameters(imagePoints,worldPoints, ...
                              'ImageSize',imageSize);

%%
% 将所有位姿保存到一个数组中
% num_of_images = size(images.Files, 1);
num_of_images = 20;
cHw = zeros(4, 4, num_of_images);
for i = 1 : num_of_images
    %%
    % Load image at new location.
    imOrig = imread(images.Files{i});

    %%
    % Undistort image.
    [im,newOrigin] = undistortImage(imOrig,cameraParams,'OutputView','full');
    
    %%
    % 对前面检测到的点进行去畸变
%     original_points = imagePoints(:,:, i);
%     undistored_points = undistortPoints(original_points, cameraParams);
    %%
    % Find reference object in new image.
    [original_points,boardSize] = detectCheckerboardPoints(im);
    %%
    % Compensate for image coordinate system shift.
    undistorted_points = [original_points(:,1) + newOrigin(1), ...
                 original_points(:,2) + newOrigin(2)]; 
    %%
    % Compute new extrinsics.
    [rotationMatrix, translationVector] = extrinsics(...
    undistorted_points,worldPoints,cameraParams);
    %%
    % 直接取标定结果中的数据作为外参
    rotation_vector = cameraParams.RotationVectors(i,:);
    translation_vector = cameraParams.TranslationVectors(i,:);
    %%
    % Compute camera pose.
%     [orientation, location] = extrinsicsToCameraPose(rotationMatrix, ...
%       translationVector);
  
  %%
  % 显示角点
%   figure(2);
%   imshow(im), hold on;
%   plot(original_points(:,1),original_points(:,2),'ro');
%   plot(original_points(1,1), original_points(1,2), 'yx');  hold off;
  
  %%
  % 将相机位置显示在图像中
%   figure(1);
%     plotCamera('Location',location,'Orientation',orientation,'Size',20);
%     hold on
%     pcshow([worldPoints,zeros(size(worldPoints,1),1)], 'VerticalAxisDir','down','MarkerSize',40);
%     pcshow([0,0,0], [1.0,0,0], 'VerticalAxisDir','down','MarkerSize',40);
   
    cHw(1:3, 1:3, i) = rotationMatrix';
%     wHc(1:3, 4, i) = translationVector';
%     wHc(1:3, 1:3, i) = rotationVectorToMatrix(rotation_vector)';
    cHw(1:3, 4, i) = translation_vector';
    cHw(4, 1:4, i) = [0 0 0 1];
end


%%
% 读入机械手的位置数据
bHg = cHw; % 初始化,主要是大小
positions = importdata([path '/position_robot.txt']);
for i = 1 : num_of_images
    pos = positions( i, 2:end);
    orientation = rotationVectorToMatrix(pos(4:6));
    location = pos(1:3);
    bHg(1:3, 1:3, i) = orientation';
    bHg(1:3, 4,   i) = location';
    bHg(4, :, i) = [0 0 0 1];
end

gHc = handEye(bHg, cHw);
disp(gHc);

rot_vect = -rotationMatrixToVector(gHc(1:3, 1:3));
theta = norm(rot_vect);
r = rot_vect / theta;
disp('旋转向量:');
disp(r);
disp(['旋转角' num2str(theta * 180 / pi)]);



