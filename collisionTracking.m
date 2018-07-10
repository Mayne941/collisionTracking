%  SPHERICAL OBJECT TRACKING SCRIPT
%  RICHARD MAYNE 2018
%  CC License: Attribution-NonCommercial 4.0 International
%  Some elements of code are adapted from Matlab documentation

%  PART 1: READ FRAME 1, SEGMENTATION BY COLOUR
%%%%%%%%%%% FRAME 1
frame1 = imread('1.jpg'); %FNAME
%imshow(frame1);

lab_frame1 = rgb2lab(frame1); % Convert to cielab colour space

ab = lab_frame1(:,:,2:3);
nrows = size(ab,1);
ncols = size(ab,2);
ab = reshape(ab,nrows*ncols,2);

nColors = 2;  % Can up to 3 for more accuracy
% repeat the clustering 3 times to avoid local minima
[cluster_idx, cluster_center] = kmeans(ab,nColors,'distance','sqEuclidean', ...
                                      'Replicates',1);
 
                                  
pixel_labels = reshape(cluster_idx,nrows,ncols);
imshow(pixel_labels,[]);
hold on
imshow(pixel_labels,[]);   % Draw and output frame 1 segmentation
export_fig('frame1.png')
hold off

%%
% PART 2: READ FRAME 2, SEGMENTATION BY COLOUR
%%%%%%%%%%%% FRAME 2

frame2 = imread('2.jpg'); %FNAME
%imshow(frame2);

lab_frame2 = rgb2lab(frame2);

ab = lab_frame2(:,:,2:3);
nrows = size(ab,1);
ncols = size(ab,2);
ab = reshape(ab,nrows*ncols,2);

nColors = 2;
% repeat the clustering 3 times to avoid local minima
[cluster_idx, cluster_center] = kmeans(ab,nColors,'distance','sqEuclidean', ...
                                      'Replicates',1);
                                  
pixel_labels = reshape(cluster_idx,nrows,ncols);
imshow(pixel_labels,[]);

hold on
imshow(pixel_labels,[]);
export_fig('frame2.png')
hold off

%%%%%%% // Optional code for further segmentation clusters

%segmented_images = cell(1,3);
%rgb_label = repmat(pixel_labels,[1 1 3]);

%for k = 1:nColors
%    color = frame2;
%    color(rgb_label ~= k) = 0;
%    segmented_images{k} = color;
%end

%imshow(segmented_images{1}), title('objects in cluster 1');
%imshow(segmented_images{2}), title('objects in cluster 2');
%imshow(segmented_images{3}), title('objects in cluster 3');

%imwrite(segmented_images{2},'frame2.png')

%%%%%%%  \\

%% PART 3: TRACKING WITH TWOSTAGE CIRCULAR HOUGH TRANSFORM

rgbF1 = imread('frame1.png');
imshow(rgbF1);

%d = imdistline; % find approx size of marble (90--100px) % delete(d)

%rgbF1 = imcomplement(rgbF1);  %% Does it need inverting??

%grey_imageF1 = rgb2gray(rgbF1);
%imshow(grey_imageF1)

% ADJUST Px RADIUS AND SENSITIVITY TO CORRECT RANGE

[centresF1,radiiF1] = imfindcircles(rgbF1,[40 48],... % px range of RADIUS
    'Sensitivity',0.976,'method','twostage') % Sensitivity between 0 and 1
        % ,'ObjectPolarity','dark') if circles are darker than bg

imshow(rgbF1)
h1 = viscircles(centresF1,radiiF1);
%%
%%Img2

rgbF2 = imread('frame2.png');
imshow(rgbF2);

%grey_imageF2 = rgb2gray(rgbF2);
%imshow(grey_imageF2)

[centresF2,radiiF2] = imfindcircles(rgbF2,[40 49],... % px range of RADIUS
    'Sensitivity',0.98,'method','twostage') %between 0 and 1
        % ,'ObjectPolarity','dark') if circles are darker than bg

imshow(rgbF2)
h2 = viscircles(centresF2,radiiF2);
        

A1x = centresF1(2,1); %Marble A, frame 1 %% If script tracks the wrong one, change to 1,1
A1y = centresF1(2,2);
A2x = centresF2(2,1); %Marble A, frame 2
A2y = centresF2(2,2);
B2x = centresF2(1,1); %Marble B, frame 2
B2y = centresF2(1,2);

%%%%%%%%%%%%%% // use in case of switchover emergencies!
%A2x = centresF2(1,1); %Marble B, frame 2
%A2y = centresF2(1,2);
%B2x = centresF2(2,1); %Marble A, frame 2
%B2y = centresF2(2,2);
%%%%%%%%%%%%%% \\
%%%%%%%%%%%%%%

hold on
imshow(rgbF2)
%h1 = viscircles(centresF1,radiiF1); %% Check if camera moved?
h2 = viscircles(centresF2,radiiF2);
line([A1x,A2x],[A1y,A2y])
export_fig('paths.png')  %%Draw and output
hold off

distanceTravelled = sqrt((A2x - A1x)^2 + (A2y - A1y)^2) % A1-A2 dist in px!

%% PART 4: CALCULATION OF OFFSET, DATA STORAGE

%%%%%%%% Continue path line
A = [A1x,A2x]; %% Line segment coords
B = [A1y,A2y]; 
plot(A,B,'*')
hold on
imshow(rgbF2)
h = viscircles(centresF2,radiiF2); %Redraw image
line(A,B)
hold off

slope = (A2y-A1y)/(A2x-A1x);
xLeft = B2x; % X extension left
yLeft = slope * (xLeft - A1x) + A1y;
xRight = A2x; % right extension
yRight = slope * (xRight - A1x) + A1y;
line([xLeft, xRight], [yLeft, yRight], 'Color', 'r', 'LineWidth', 3);

%%%%%%% draw line B2-A2
line([A2x,B2x],[A2y,B2y],'color','g'); %Draw line between centres of A2 and B2
distA2B2 = sqrt((A2x - B2x)^2 + (A2y - B2y)^2); %calculate length of hypotenuse

%%%%%%% Define 3 points about B2-A2-Line extension, calculate angle
%%%%%%% IMPORTANT: CHECK LABELLING OF P0--2 DEPENDING ON ROTATION
P1 = [B2x,B2y]; %Centre B2  x1 y1
P0 = [A2x,A2y]; %Centre A2  x2 y2
P2 = [xLeft,yLeft]; %End of extended path line x3 y3


%ang = atan2(abs((A2x-B2x)*(yLeft-B2y)-(xLeft-B2x)*(A2y-B2y)), ... 
 %               (A2x-B2x)*(xLeft-B2x)+(A2y-B2y)*(yLeft-B2y));
    % Less efficient than following code

ang = atan2(abs(det([P2-P0;P1-P0])),dot(P2-P0,P1-P0)); % Ang in rad
angleOut = ang*(180/pi) % Angle in deg

%%%%%%% Trigonometry to calculate offset (opposite to hypotenuse of A2-B2)
offset = sind(angleOut)*distA2B2  % using sind as working in degrees

%%%%%%% Write angle to image, save image
txt1 = ['Angle = ',num2str(angleOut)];
txt2 = ['Offset = ',num2str(offset)];
text(A2x,A2y,txt1,'color','m','FontWeight','bold');
text(B2x,B2y,txt2,'color','m','FontWeight','bold');
export_fig('angle.png')

%% PART 5: SAVE TO EXCEL FILE

radiiF2A = radiiF2(1,1); % Define all of the vars to output
radiiF2B = radiiF2(2,1);
APosF1 = [A1x,A1y];
APosF2 = [A2x,A2y];
BPosF2 = [B2x,B2y];
A1A2Dist = distanceTravelled;
A2B2Dist = distA2B2;
InternalAngle = angleOut;
%Offset
%speed = 'Multiply dist by frame diff';

T = table(radiiF2A,radiiF2B,APosF1,APosF2,BPosF2,A1A2Dist,A2B2Dist,InternalAngle,offset)
        %Make table

filename = 'numbers.xlsx';  %% Save spreadsheet
writetable(T,filename,'Sheet',1,'Range','D1')

filenameVariables = 'variables.mat';  %% Save workspace variables
save(filenameVariables)



