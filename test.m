clear all;
% load the image
mov = load_sequence_color('C:\Users\yu702\Desktop\GV15\cw2\Lab 5 files-20180303\gjbLookAtTargets','gjbLookAtTarget_', 0, 71, 4, 'jpg');
numImg = 72;
%% collect the initial frame
prompt = 'Please input a number from 1 to 72\n';
initialFrame = input(prompt);
Steps = 4;
%% This part aims to compute the distance matrix

disMat = zeros(numImg,numImg);
for i = 1:numImg
    for j = 1:numImg 
        I_i = mov(:,:,:,i);
        I_j = mov(:,:,:,j);
        dis = I_i - I_j;
        for k = 1:3
            disMat(i,j) = disMat(i,j) + norm(dis(:,:,k),2);
        end
    end
end
%% This part aims to compute the new distance matrix with the 
% consideration of the trajectory-similiarity.

disMat = disMat / (numImg);
% collect the two frames before the current frame and on frame after the 
% current frame.
m = 2;
disMat_new = disMat;
for i = 3:numImg-1
    for j = 3:numImg-1
        disMat_new(i,j) = 1/15 * disMat(i-2,j-2) + 4/15 * disMat(i-1,j-1) + 6/15 * disMat(i,j) + 4/15 * disMat(i+1,j+1);
    end
end

%% this part aims to generate the graph with nodes and edges
nodeX(1) = 0;
nodeY(1) = 0;
weightMat(1) = 0;
k = 1;
% just add the edge where the distance between two frames is small enough.
for i = 1:numImg
    findSmall = 0; 
    for j  = 1:numImg
        % if the distance is smaller than 0.8, add the edge
        if disMat_new(i,j)<0.8
            findSmall = 1; 
            nodeX(k) = i;
            nodeY(k) = j;
            weightMat(k) = disMat_new(i,j);
            k = k + 1;
        end
    end
    % if no edge between two nodes, find the minimum distance of frame i
    if findSmall == 0
        mat = disMat_new(i,:)
        weightMat(k) = min(mat(find(mat~=0)));
        nodeX(k) = i;
        nodeY(k) = find(disMat_new(i,:)==weightMat(k));
        k = k + 1;
    end
end
%% generate the sparse matrix and allow user to input lines
    
DG = sparse(nodeX,nodeY,weightMat,72,72); 
imshow(mov(:,:,:,initialFrame));
% collect the input points
[x,y] = getline(gca);
    
%% interpolate a point between two points
newX = [x(1)];
newY = [y(1)];
for pointIndex = 1: length(x)-1
    current_pX = x(pointIndex);
    current_pY = y(pointIndex);
    next_pX = x(pointIndex+1);
    next_pY = y(pointIndex+1);
    % alpha is the step length
    alpha = (next_pX - current_pX)/Steps;
    % X is the list of existent x
    % Y is the list of existent y
    X = [current_pX next_pX]';
    Y = [current_pY next_pY]';
    % Xi is the new list of x
    % Yi is the new list of y
    Xi = (current_pX:alpha:next_pX)';
    Yi = interp1q(X,Y,Xi);
    % add new list of x and y to the points list
    newX = [newX Xi(2:end)'];
    newY = [newY Yi(2:end)'];
end
%% find shortest path with optical flow
load('flows.mat');
% The whole_path is the final path of output 
whole_path = [initialFrame];
% the initial firstFrame and nextFrame is the initial frame
firstFrame = initialFrame;
nextFrame = initialFrame;
% calculate the shortest path for every segment line
for pointIndex = 1 : length(newX)-1
    % find the path from the first frame to every frame
    [dist,path,pred] = graphshortestpath(DG,firstFrame);
    % get the number of paths
    [~, pathNum] = size(path);
    % end_p is the next point which is the destination of this
    % segment
    end_p = [newY(pointIndex+1);newX(pointIndex+1)];
    % initialise the minimum distance between the end_p and the
    % endpoint of the path.
    minDis = inf;
    % find the path whose advected location comes closest to the end_p
    for pathIndex = 1:pathNum
        % the current start point is current_P
        current_p = [newY(pointIndex);newX(pointIndex)];
        % initilise the next_p 
        next_p = current_p;
        % get the path
        Path = path{pathIndex};
        % if the length of path is 1, do nothing
        % if not, try to find the flow between the current frame and
        % next frame in the path 
        for i = 1:length(Path)-1
            if length(Path)==1
            else
                currentFrame = Path(i);
                nextFrame = Path(i+1);
                if currentFrame>nextFrame
                    k = (currentFrame-1) * (currentFrame-2) / 2 + nextFrame;
                    flow = flows_a(floor(current_p(1)),floor(current_p(2)),:,k);
                else
                    k = (nextFrame-1) * (nextFrame-2) /2 + currentFrame;
                    flow = -flows_a(floor(current_p(1)),floor(current_p(2)),:,k);
                end
                flow = squeeze(flow);
                % add flows to the current_p
                current_p = [current_p(1) + flow(2);current_p(2)+flow(1)];
            end
        end
        % if the current point is closer to the end_p, record the
        % minimum distance, path, the current point and the next frame of 
        %the path
        if norm(current_p - end_p) < minDis
            end_X(pointIndex) = current_p(1);
            end_Y(pointIndex) = current_p(2);
            minDis = norm(current_p - end_p);
            Paths = Path;
            firstFrame = nextFrame;
        end
    end
    % add the path of this segment to the final path
    whole_path = [whole_path Paths(2:end)];
end

%% show and save the result

for num = 1:length(whole_path)
    imshow(mov(:,:,:,whole_path(num)));
    imwrite(mov(:,:,:,whole_path(num)),['result\',num2str(num),'.jpg'])
end
%% show path comparison
figure;
imshow(mov(:,:,:,initialFrame));
hold on;
endList_X = [newX(1) end_Y];
endList_Y = [newY(1) end_X];
% plot the actually path
plot(endList_X,endList_Y,'r-');
hold on;
% plot the input path
plot(x,y,'g-');

