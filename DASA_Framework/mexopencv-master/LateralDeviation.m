function [yL, out_img] = LateralDeviation( src, LL ) % show bird's eye view with slide windows fit
% function [yL, result] = LateralDeviation( src, LL ) % show complete re-map lane detection result

%get image info
[height width depth] = size(src);

%% bird's eye view transform
% this parameter maybe adjust according to camera setting (e.g. position, resolution)
src_corners = {[233,280], [277 ,280], [462,512], [50,512]};
wrap_offset = 70;
dst_corners = {[50 + wrap_offset, 0], [462 - wrap_offset, 0], [462 - wrap_offset, 512], [50 + wrap_offset, 512]};

M = cv.getPerspectiveTransform(src_corners, dst_corners);
wrap_img = cv.warpPerspective(src, M);
% figure, imshow(wrap_img), title('Birds eye view'), axis on;

%% image thresholding, binary image

%SobelX
gray = cv.cvtColor(wrap_img, 'RGB2GRAY');
gradx = cv.Sobel(wrap_img, 'XOrder',1, 'YOrder',0, 'DDepth','int16');
gradxabs = cv.convertScaleAbs(gradx);
max(gradxabs)
sxbinary = gradxabs;
f = find(sxbinary<200);%find lower than 200
sxbinary(f)=0; %put all to zero(black)
sxbinary(~f)=255;
% figure, imshow(sxbinary), title('Sobelx filter binary'), axis on;

%color space (RGB)
lowerb = [200 200 200];
upperb = [255 255 255]; % to extract the white lane marker
mask = cv.inRange(wrap_img, lowerb, upperb);
res = cv.bitwise_and(wrap_img, wrap_img, 'Mask', mask);
w = find(res>200);
res(w) = 255;
% figure, imshow(res), title('white mask'), axis on;

%combine filter
l = cv.bitwise_or(res, sxbinary);
lowerb = [255 255 255];
upperb = [255 255 255];
mask = cv.inRange(l, lowerb, upperb);
lane = cv.bitwise_and(l, l, 'Mask', mask);
g = find(lane>0);
lane(g) = 150;

%% this part is for eliminate the unwanted lane marker
% %mask left
% lane(1:50,1:120,1:3)=0;
% lane(51:100,1:90,1:3)=0;
% lane(101:150,1:60,1:3)=0;
% lane(150:200,1:30,1:3)=0;
% % lane(201:250,1:30,1:3)=0;
% 
% %mask right
% lane(1:50,392:512,1:3)=0;
% lane(51:100,422:512,1:3)=0;
% lane(101:150,452:512,1:3)=0;
% lane(150:200,482:512,1:3)=0;
% % lane(201:250,482:512,1:3)=0;

% figure, imshow(lane), title('lane bit'), axis on;

%% identify Lane Lines
% get image histogram
H = sum(sum(lane,3),1);
H2 = sum(lane,3);
% figure, plot(H);

% set slide window base for left and right lane boundaries
[maxleft leftxbase]=max(H(1:width/2-40));
[maxright rightxbase]=max(H(width/2+40:width));
rightxbase = rightxbase+width/2;

%Set Nr of slide windows are 8, the margin from center is 65 pixels and
%only over 400 non-zero points in window could be count as a slides window
nrWin = 8; margin = 65; minpix = 400;
winheight = height/nrWin;

[nonzeroy nonzerox] = find(lane);
%Current positions to be updated for each window
leftx_current = leftxbase;
rightx_current = rightxbase;
%Create empty lists to receive left and right lane pixel indices
left_lane_inds = [];
right_lane_inds = [];

% window = 0;
out_img = lane;
for window = 0:(nrWin-1)
    %Identify window boundaries in x and y (and right and left)
    win_y_low = height - (window+1) * winheight;
    win_y_high = height - window * winheight;
    win_xleft_low = max(0,(leftx_current - margin));
    win_xleft_high = min(512,(leftx_current + margin));
    win_xright_low = max(0,(rightx_current - margin));
    win_xright_high = min(512,(rightx_current + margin));
    %Draw the windows on the visualization image
    out_img = cv.rectangle(out_img,[win_xleft_low,win_y_low],[win_xleft_high,win_y_high],'Color',[0,255,0],'Thickness',2);
    out_img = cv.rectangle(out_img,[win_xright_low,win_y_low],[win_xright_high,win_y_high],'Color',[0,255,0],'Thickness',2);
    
    %Identify the nonzero pixels in x and y within the window
    ylowin = floor(win_y_low)+1
    yhighin = ceil(win_y_high)-1
    [good_left_indy good_left_indx]= find(H2((floor(win_y_low)+1):(ceil(win_y_high)-1),(win_xleft_low+1):(win_xleft_high-1)));
      %store left lane boundary pixel x-axis value in slide winow
      if size(good_left_indx,1)> minpix
          for i = 1:size(good_left_indx,1)
              left_lane_inds = [left_lane_inds,[(good_left_indx(i)+win_xleft_low);(good_left_indy(i)+floor(win_y_low))]];
          end
      end
      %decide next slide window center's x-axis corrdinate
      if size(good_left_indx,1)> minpix
          leftx_current = mean(good_left_indx)+win_xleft_low;
      end
      
      %store right lane boundary pixel x-axis value in slide winow
      [good_right_indy good_right_indx]= find(H2((floor(win_y_low)+1):(ceil(win_y_high)-1),(win_xright_low+1):(win_xright_high-1)));
      
      if size(good_right_indx,1)> minpix
          for i = 1:size(good_right_indx,1)
              right_lane_inds = [right_lane_inds,[(good_right_indx(i)+win_xright_low);(good_right_indy(i)+floor(win_y_low))]];
          end
      end
      % decide next slide window center's x-axis corrdinate
      if size(good_right_indx,1)> minpix
          rightx_current = mean(good_right_indx)+win_xright_low;
      end
end

%polyfit lane boundaries
pl = polyfit(left_lane_inds(2,:),left_lane_inds(1,:),2);
lineyl = linspace(0,511,512);
linexl = polyval(pl,lineyl);

pr = polyfit(right_lane_inds(2,:),right_lane_inds(1,:),2);
lineyr = linspace(0,511,512);
linexr = polyval(pr,lineyr);
% figure, plot(left_lane_inds(1,:),left_lane_inds(2,:),'r'), title('lane line'), axis on;

% figure, imshow(out_img), hold on;
% plot(left_lane_inds(1,:),left_lane_inds(2,:),'r'); hold on;
% plot(right_lane_inds(1,:),right_lane_inds(2,:),'b'); hold on;
% plot(linexl,lineyl,'w'); hold on;
% plot(linexr,lineyr,'w');
% title('lane rec'), axis on;

%% view re-transform back
color_warp = zeros(size(wrap_img));
pts = [];
for i = 1:512
    pts = [pts,[linexl(i);lineyl(i)]];
    pts = [pts,[linexr(i);lineyr(i)]];
end
pts = pts';
color_warp = cv.fillPoly(color_warp, pts, 'Color', [0,0,255]);
% figure, imshow(color_warp), title('color region'), axis on;


src_corners = {[233,280], [277 ,280], [462,512], [50,512]};
wrap_offset = 70;
dst_corners = {[50 + wrap_offset, 0], [462 - wrap_offset, 0], [462 - wrap_offset, 512], [50 + wrap_offset, 512]};
M2b = cv.getPerspectiveTransform(dst_corners, src_corners);
new_warp = cv.warpPerspective(color_warp, M2b);
new_warp = uint8(new_warp);
result = cv.addWeighted(src, 1, new_warp, 0.7, 0);
% figure, imshow(result), title('view re-tranform'), axis on;

%% calculate lateral deviation at different look-ahead distance(LL)
if LL == 0.9
    yL_leftx = polyval(pl,478);
    yL_rightx = polyval(pr,478);
else if LL == 0.675
        yL_leftx = polyval(pl,481);
        yL_rightx = polyval(pr,481);
    else if LL == 0.55
            yL_leftx = polyval(pl,483);
            yL_rightx = polyval(pr,483);
        end
    end
end

scale_leftx = polyval(pl,512);
scale_rightx = polyval(pr,512);

scale = 0.32/(scale_rightx - scale_leftx);
yL = (256.5-(yL_leftx+yL_rightx)/2) * scale;

end

