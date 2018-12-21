%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Image Processing and Vision
% Tracking of moving objects using two cameras
%
% Done by: 
% Diogo Morgado
% Luis Lopes
% Miguel Paulino
% Valter Piedade
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [objects,cam2toW] = track3D_part2(imgseq1, imgseq2, cam_params)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Variable Initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
imgs1=zeros(480,640,length(imgseq1));
imgsd1=zeros(480,640,length(imgseq1));
imgs2=zeros(480,640,length(imgseq2));
imgsd2=zeros(480,640,length(imgseq2));
objects_cam1=struct('X', [], 'Y', [], 'Z', [],'frames_tracked',[]);
objects_cam2=struct('X', [], 'Y', [], 'Z', [],'frames_tracked',[]);
objects_cam1_hist=struct('hist',[],'frames', []);
objects_cam2_hist=struct('hist',[],'frames', []);
objects=struct('X', [], 'Y', [], 'Z', [],'frames_tracked',[]);

for i=1:length(imgseq1)
    imgs1(:,:,i)=rgb2gray(imread(imgseq1(i).rgb));
    imgs2(:,:,i)=rgb2gray(imread(imgseq2(i).rgb));
    load(imgseq1(i).depth);
    imgsd1(:,:,i)=double(depth_array)/1000;
%     figure(1);
%     subplot(1,2,1);
%     imagesc(imgsd1(:,:,i));
    load(imgseq2(i).depth);
    imgsd2(:,:,i)=double(depth_array)/1000;
%     subplot(1,2,2);
%     imagesc(imgsd2(:,:,i));
%     pause(0.1);
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Backgroud for both rgb and depth cameras
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bgdepth_cam1=median(imgsd1(:,:,:),3);
bgrgb_cam1=median(imgs1(:,:,:),3);
bgrgb_cam2=median(imgs2(:,:,:),3);
bgdepth_cam2=median(imgsd2(:,:,:),3);
% figure(2);
% subplot(2,2,1);
% imagesc(bgdepth_cam1);
% subplot(2,2,2);
% imagesc(bgdepth_cam2);
% subplot(2,2,3);
% imagesc(bgrgb_cam1);
% subplot(2,2,4);
% imagesc(bgrgb_cam2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Find of the matching points using SIFT for all the images of both cameras
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:length(imgseq1)
    load(imgseq1(i).depth);
    xyz_cam1=get_xyz_asus(depth_array(:),[480 640], 1:480*640, cam_params.Kdepth ,1,0);
    rgbd_cam1=get_rgbd(xyz_cam1,imread(imgseq1(i).rgb), cam_params.R, cam_params.T, cam_params.Krgb);
    load(imgseq2(i).depth);
    xyz_cam2=get_xyz_asus(depth_array(:),[480 640], 1:480*640, cam_params.Kdepth ,1,0);
    rgbd_cam2=get_rgbd(xyz_cam2,imread(imgseq2(i).rgb), cam_params.R, cam_params.T, cam_params.Krgb);    
    [~,d1]=vl_sift(im2single(rgb2gray(rgbd_cam1)),'edgethresh', 500);
    [~,d2]=vl_sift(im2single(rgb2gray(rgbd_cam2)),'edgethresh', 500);
    matches = vl_ubcmatch(d1,d2);
    num_matches(i)=length(matches);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matching for the image with the most matches
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[~, index]=max(num_matches);
%get rbgd points from xyz points cam1
load(imgseq1(index).depth);
xyz_cam1=get_xyz_asus(depth_array(:),[480 640], 1:480*640, cam_params.Kdepth ,1,0);
rgbd_cam1=get_rgbd(xyz_cam1,imread(imgseq1(index).rgb), cam_params.R, cam_params.T, cam_params.Krgb);
%get rbgd points from xyz points cam2
load(imgseq2(index).depth);
xyz_cam2=get_xyz_asus(depth_array(:),[480 640], 1:480*640, cam_params.Kdepth ,1,0);
rgbd_cam2=get_rgbd(xyz_cam2,imread(imgseq2(index).rgb), cam_params.R, cam_params.T, cam_params.Krgb);
%get features for both cameras
[f1,d1]=vl_sift(im2single(rgb2gray(rgbd_cam1)),'edgethresh', 500);
[f2,d2]=vl_sift(im2single(rgb2gray(rgbd_cam2)),'edgethresh', 500);
%find the matching features between the two images
matches = vl_ubcmatch(d1,d2);
%get coordinates of the matching features
cam1=[(fix(f1(1,matches(1,:))))' (fix(f1(2,matches(1,:))))'];
cam2=[(fix(f2(1,matches(2,:))))' (fix(f2(2,matches(2,:))))'];

% image with the good points in each image
% figure(3);
% subplot(1,2,1);
% imagesc(imgs1(:,:,index));
% hold on;
% plot(cam1(:,1),cam1(:,2),'*r');
% hold off;
% subplot(1,2,2);
% imagesc(imgs2(:,:,index));
% hold on;
% plot(cam2(:,1),cam2(:,2),'*r');
% hold off;

%image with the matched points
% figure(4);
% ax = axes;
% showMatchedFeatures(imgs1(:,:,index), imgs2(:,:,index), cam1, cam2,'montage','Parent',ax);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RANSAC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ind_cam1=sub2ind([480 640],cam1(:,2),cam1(:,1));
ind_cam2=sub2ind([480 640],cam2(:,2),cam2(:,1));
%get xyz points of the matching features for both cameras
xyz_points1=xyz_cam1(ind_cam1,:);
xyz_points2=xyz_cam2(ind_cam2,:);

%choose the good points
inds=find((xyz_points1(:,3).*xyz_points2(:,3))>0);
xyz_points1=xyz_points1(inds,:);
xyz_points2=xyz_points2(inds,:);

%Test random sets of 4 points
niter=2000;
error_thresh=0.4;
aux=fix(rand(4*niter,1)*length(xyz_points1)+1);

for i=1:niter-4
    xyz_aux1=xyz_points1(aux(4*i:4*i+3),:);
    xyz_aux2=xyz_points2(aux(4*i:4*i+3),:);
    [~,~,trans]=procrustes(xyz_aux1,xyz_aux2,'scaling',false,'reflection',false);
    R(:,:,i)=trans.T; T(:,:,i)=trans.c(1,:);
    error=xyz_points1-xyz_points2*trans.T-ones(length(xyz_points2),1)*trans.c(1,:);
    numinliers(i)=length(find(sum(error.*error,2)<error_thresh^2));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get the extrinsic parameters matrix for the points with the most inliers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[~, index]= max(numinliers);
R=R(:,:,index);
T=T(:,:,index);
error=xyz_points1-xyz_points2*R-ones(length(xyz_points2),1)*T(1,:);
inds=find(sum(error.*error,2)<error_thresh^2);
xyz_points1=xyz_points1(inds,:);
xyz_points2=xyz_points2(inds,:);
rgbd1=reshape(rgbd_cam1,[640*480 3]);
rgbd2=reshape(rgbd_cam2,[640*480 3]);
[~,~,trans]=procrustes(xyz_points1,xyz_points2,'scaling',false,'reflection',false);
%image 2 in the coordinates of the world
xyz21=xyz_cam2*trans.T+ones(length(xyz_cam2),1)*trans.c(1,:);
pc1=pointCloud(xyz_cam1,'Color',rgbd1);
pc2=pointCloud(xyz_cam2,'Color',rgbd2);
pc3=pointCloud(xyz21,'Color',rgbd2);
% merge of the both cam images in the world coordinates
pc4=pcmerge(pc1,pc3,0.01);
% figure(5);
% subplot(2,2,1);showPointCloud(pc1);
% subplot(2,2,2);showPointCloud(pc2);
% subplot(2,2,3);showPointCloud(pc3);
% subplot(2,2,4);showPointCloud(pc4);
%storing the extrinsic parameters matrix (rotation and translacion)
cam2toW=struct('R',trans.T,'T', trans.c(1,:)');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tracking based on centroids and color
% same as part1 just doing it two times
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:(length(imgseq1))
    %BackGround Subtraction
    imdiff_cam1=abs(imgsd1(:,:,i)-bgdepth_cam1)>.2;
    imdiff_cam2=abs(imgsd2(:,:,i)-bgdepth_cam2)>.2;
    % Morfological Filter
    imgdiffiltered_cam1=imopen(imdiff_cam1,strel('disk',5));
    imgdiffiltered_cam2=imopen(imdiff_cam2,strel('disk',5));
    
%     figure(6);
%     subplot(1,2,1);
%     imagesc(imgdiffiltered_cam1);
%     subplot(1,2,2);
%     imagesc(imgdiffiltered_cam2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gradient for identifying aligned objects for both cams
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    value_cam1=find(imgdiffiltered_cam1==1);
    value_cam2=find(imgdiffiltered_cam2==1);
    grad_cam1=zeros(480,640);
    grad_cam2=zeros(480,640);
    dept_cam1=imgsd1(:,:,i);
    dept_cam2=imgsd2(:,:,i);
    grad_cam1(value_cam1)=dept_cam1(value_cam1);
    grad_cam2(value_cam2)=dept_cam2(value_cam2);
    
%     figure(7);
    [FX1,FY1]=gradient(grad_cam1);
    [FX2,FY2]=gradient(grad_cam2);
    gradValX1=abs(FX1);gradValY1=abs(FY1);
    gradValX2=abs(FX2);gradValY2=abs(FY2);
%     subplot(2,2,1);imagesc(gradValX1);
%     subplot(2,2,2);imagesc(gradValX2);
%     subplot(2,2,3);imagesc(gradValY1);
%     subplot(2,2,4);imagesc(gradValY2);
    
    img_size = size(imgsd1(:,:,i));
    [I, J] = ind2sub(img_size(1:2), find(imgdiffiltered_cam1 == 1));
    r = I(I > 1 & I < img_size(1) & J > 1 & J < img_size(2));
    c = J(I > 1 & I < img_size(1) & J > 1 & J < img_size(2));
    indlen = length(r);
    
    for ind = 1:indlen
        neigh_v = [gradValX1(r(ind)-1,c(ind)-1)
            gradValX1(r(ind)-1,c(ind))
            gradValX1(r(ind)-1,c(ind)+1)
            gradValX1(r(ind),c(ind)-1)
            gradValX1(r(ind),c(ind)+1)
            gradValX1(r(ind)+1,c(ind)-1)
            gradValX1(r(ind)+1,c(ind))
            gradValX1(r(ind)+1,c(ind)+1)];
        v = abs(gradValX1(r(ind),c(ind)) - neigh_v);
        neigh_u = [gradValY1(r(ind)-1,c(ind)-1)
            gradValY1(r(ind)-1,c(ind))
            gradValY1(r(ind)-1,c(ind)+1)
            gradValY1(r(ind),c(ind)-1)
            gradValY1(r(ind),c(ind)+1)
            gradValY1(r(ind)+1,c(ind)-1)
            gradValY1(r(ind)+1,c(ind))
            gradValY1(r(ind)+1,c(ind)+1)];
        u = abs(gradValY1(r(ind),c(ind)) - neigh_u);
        
        if(~isempty(find(v > 0.2)>0) || ~isempty(find(u > 0.2)>0))
            imgdiffiltered_cam1(r(ind),c(ind)) = 0;
        end
    end
    img_size = size(imgsd2(:,:,i));
    [I, J] = ind2sub(img_size(1:2), find(imgdiffiltered_cam2 == 1));
    r = I(I > 1 & I < img_size(1) & J > 1 & J < img_size(2));
    c = J(I > 1 & I < img_size(1) & J > 1 & J < img_size(2));
    indlen = length(r);
    
    for ind = 1:indlen
        neigh_v = [gradValX2(r(ind)-1,c(ind)-1)
            gradValX2(r(ind)-1,c(ind))
            gradValX2(r(ind)-1,c(ind)+1)
            gradValX2(r(ind),c(ind)-1)
            gradValX2(r(ind),c(ind)+1)
            gradValX2(r(ind)+1,c(ind)-1)
            gradValX2(r(ind)+1,c(ind))
            gradValX2(r(ind)+1,c(ind)+1)];
        v = abs(gradValX2(r(ind),c(ind)) - neigh_v);
        neigh_u = [gradValY2(r(ind)-1,c(ind)-1)
            gradValY2(r(ind)-1,c(ind))
            gradValY2(r(ind)-1,c(ind)+1)
            gradValY2(r(ind),c(ind)-1)
            gradValY2(r(ind),c(ind)+1)
            gradValY2(r(ind)+1,c(ind)-1)
            gradValY2(r(ind)+1,c(ind))
            gradValY2(r(ind)+1,c(ind)+1)];
        u = abs(gradValY2(r(ind),c(ind)) - neigh_u);
        
        if(~isempty(find(v > 0.2)>0) || ~isempty(find(u > 0.2)>0))
            imgdiffiltered_cam2(r(ind),c(ind)) = 0;
        end
    end
    
%     figure(8);
%     subplot(1,2,1);imagesc(imgdiffiltered_cam1);
%     subplot(1,2,2);imagesc(imgdiffiltered_cam2);
    
    bw2_cam1=bwareaopen(imgdiffiltered_cam1,1250);
    bw2_cam2=bwareaopen(imgdiffiltered_cam2,1250);
    
%labeling of the identified objects in both cams
    [bw31,M1]=bwlabel(bw2_cam1);
    [bw32,M2]=bwlabel(bw2_cam2);
%     figure(9);
%     subplot(1,2,1);imagesc(bw31);
%     subplot(1,2,2);imagesc(bw32);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Box calculation for both cams, using centroids and color for identifying
% different objects
% Same as part 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%     figure(10);
%     clf;
    for j=1:M1
        ind=find(bw31==j);
        load(imgseq1(i).depth);
        aux=zeros(480,640);
        aux(ind)=depth_array(ind);
        xyz_cam1=get_xyz_asus(aux(:),[480 640], find(aux>0.2 & aux<6000), cam_params.Kdepth,1,0);
        if ~any(aux(:))
            continue
        end
        img_rgb_object1 = zeros(480,640,3);
        aux_rgb1 = imread(imgseq1(i).rgb);
        for d = 1:3
            aux_rgb_object1 = zeros(480,640);
            aux_image1 = aux_rgb1(:,:,d);
            aux_rgb_object1(ind) = aux_image1(ind);
            img_rgb_object1(:,:,d) = aux_rgb_object1;
        end
        img_rgbd1 = get_rgbd(xyz_cam1,img_rgb_object1,cam_params.R,cam_params.T, cam_params.Krgb);
        [Counts1 BinLocations1]=imhist(rgb2gray(img_rgbd1));
        hist1=Counts1';
        hist1(1) = 0;
        
        pc1=pointCloud(xyz_cam1);
%         showPointCloud(pc1);
        
        Z=pc1.Location(:,3);
        zmax=max(Z);
        zmin=min(Z(Z~=0));
        
        Y=pc1.Location(:,2);
        ymax=max(Y);
        ymin=min(Y(Y~=0));
        
        X=pc1.Location(:,1);
        xmax=max(X(X~=0));
        xmin=min(X);
        
        if zmax==0
            continue;
        end
        
        % draw box
        X = [xmin;xmin;xmin;xmin;xmin];
        Y = [ymin;ymin;ymax;ymax;ymin];
        Z = [zmin;zmax;zmax;zmin;zmin];
%         hold on;
%         plot3(X,Y,Z,'r');
        X = [xmax;xmax;xmax;xmax;xmax];
%         hold on;
%         plot3(X,Y,Z,'r');
        
        Z = [zmin;zmin;zmin;zmin;zmin];
        X = [xmin;xmin;xmax;xmax;xmin];
        Y = [ymin;ymax;ymax;ymin;ymin];
%         hold on;
%         plot3(X,Y,Z,'r');
        Z = [zmax;zmax;zmax;zmax;zmax];
%         hold on;
%         plot3(X,Y,Z,'r');
%         pause(0.1);
        
        % centroid calculation
        centroid=[xmax-((xmax-xmin)/2), ymax-((ymax-ymin)/2), zmax-((zmax-zmin)/2)];
        
        %case of the first object
        if isempty(objects_cam1(1).X)
            objects_cam1_hist(1).hist=hist1;
            objects_cam1_hist(1).frames=i;
            objects_cam1(1).X=[xmin xmin xmin xmin xmax xmax xmax xmax];
            objects_cam1(1).Y=[ymin ymin ymax ymax ymin ymin ymax ymax];
            objects_cam1(1).Z=[zmin zmax zmin zmax zmin zmax zmin zmax];
            objects_cam1(1).frames_tracked=[i];
        else
            n=0;
            for k = 1:length(objects_cam1)
                if objects_cam1(k).frames_tracked(end)~=i-1
                    continue;
                end
                h_prev1 = objects_cam1_hist(k).hist(end,:);
                h_err1 = exp(-sum(abs(hist1-h_prev1)/(0.5*(sum(h_prev1)+sum(hist1)))));
                cent = [max(objects_cam1(k).X(end,:))-((max(objects_cam1(k).X(end,:))-min(objects_cam1(k).X(end,:)))/2), max(objects_cam1(k).Y(end,:))-((max(objects_cam1(k).Y(end,:))-min(objects_cam1(k).Y(end,:)))/2), max(objects_cam1(k).Z(end,:))-((max(objects_cam1(k).Z(end,:))-min(objects_cam1(k).Z(end,:)))/2)];
                dist=norm(centroid-cent);
                %verifying if it is the same object
                if dist < 1.8 && h_err1>0.2
                    objects_cam1_hist(k).hist=vertcat(objects_cam1_hist(k).hist, hist1);
                    objects_cam1_hist(k).frames=horzcat(objects_cam1_hist(k).frames,i);
                    objects_cam1(k).X=vertcat(objects_cam1(k).X, [xmin xmin xmin xmin xmax xmax xmax xmax]);
                    objects_cam1(k).Y=vertcat(objects_cam1(k).Y, [ymin ymin ymax ymax ymin ymin ymax ymax]);
                    objects_cam1(k).Z=vertcat(objects_cam1(k).Z, [zmin zmax zmin zmax zmin zmax zmin zmax]);
                    objects_cam1(k).frames_tracked=horzcat(objects_cam1(k).frames_tracked, i);
                    n=1;
                end
                
            end
            % in case of aa new object
            if n==0
                im=length(objects_cam1)+1;
                objects_cam1_hist(im).hist=hist1;
                objects_cam1_hist(im).frames=i;
                objects_cam1(im).X=[xmin xmin xmin xmin xmax xmax xmax xmax];
                objects_cam1(im).Y=[ymin ymin ymax ymax ymin ymin ymax ymax];
                objects_cam1(im).Z=[zmin zmax zmin zmax zmin zmax zmin zmax];
                objects_cam1(im).frames_tracked=i;
            end
            
        end
        
    end
%     figure(11);
%     clf;
    for j=1:M2
        ind=find(bw32==j);
        load(imgseq2(i).depth);
        aux=zeros(480,640);
        aux(ind)=depth_array(ind);
        xyz_cam2=get_xyz_asus(aux(:),[480 640], find(aux>0.2 & aux<6000), cam_params.Kdepth,1,0);
        if ~any(aux(:))
            continue
        end
        img_rgb_object2 = zeros(480,640,3);
        aux_rgb2 = imread(imgseq2(i).rgb);
        for d = 1:3
            aux_rgb_object2 = zeros(480,640);
            aux_image2 = aux_rgb2(:,:,d);
            aux_rgb_object2(ind) = aux_image2(ind);
            img_rgb_object2(:,:,d) = aux_rgb_object2;
        end
        img_rgbd2 = get_rgbd(xyz_cam2,img_rgb_object2,cam_params.R,cam_params.T, cam_params.Krgb);
        [Counts2 BinLocations2]=imhist(rgb2gray(img_rgbd2));
        hist2=Counts2';
        hist2(1) = 0;
        pc1=pointCloud(xyz_cam2);
%         showPointCloud(pc1);
        
        Z=pc1.Location(:,3);
        zmax=max(Z);
        zmin=min(Z(Z~=0));
        
        Y=pc1.Location(:,2);
        ymax=max(Y);
        ymin=min(Y(Y~=0));
        
        X=pc1.Location(:,1);
        xmax=max(X(X~=0));
        xmin=min(X);
        
        if zmax==0
            continue;
        end
        
        % draw box
        X = [xmin;xmin;xmin;xmin;xmin];
        Y = [ymin;ymin;ymax;ymax;ymin];
        Z = [zmin;zmax;zmax;zmin;zmin];
%         hold on;
%         plot3(X,Y,Z,'r');
        X = [xmax;xmax;xmax;xmax;xmax];
%         hold on;
%         plot3(X,Y,Z,'r');
        
        Z = [zmin;zmin;zmin;zmin;zmin];
        X = [xmin;xmin;xmax;xmax;xmin];
        Y = [ymin;ymax;ymax;ymin;ymin];
%         hold on;
%         plot3(X,Y,Z,'r');
        Z = [zmax;zmax;zmax;zmax;zmax];
%         hold on;
%         plot3(X,Y,Z,'r');
%         pause(0.1);
        
        % calculating the background
        centroid=[xmax-((xmax-xmin)/2), ymax-((ymax-ymin)/2), zmax-((zmax-zmin)/2)];
        
        %case of the first object
        if (isempty(objects_cam2(1).X)==1)
            objects_cam2_hist(1).hist=hist2;
            objects_cam2_hist(1).frames=i;
            objects_cam2(1).X=[xmin xmin xmin xmin xmax xmax xmax xmax];
            objects_cam2(1).Y=[ymin ymin ymax ymax ymin ymin ymax ymax];
            objects_cam2(1).Z=[zmin zmax zmin zmax zmin zmax zmin zmax];
            objects_cam2(1).frames_tracked=[i];
        else
            n=0;
            for k = 1:length(objects_cam2)
                if objects_cam2(k).frames_tracked(end)~=i-1
                    continue;
                end
                h_prev2 = objects_cam2_hist(k).hist(end,:);
                h_err2 = exp(-sum(abs(hist2-h_prev2)/(0.5*(sum(h_prev2)+sum(hist2)))));
                cent = [max(objects_cam2(k).X(end,:))-((max(objects_cam2(k).X(end,:))-min(objects_cam2(k).X(end,:)))/2), max(objects_cam2(k).Y(end,:))-((max(objects_cam2(k).Y(end,:))-min(objects_cam2(k).Y(end,:)))/2), max(objects_cam2(k).Z(end,:))-((max(objects_cam2(k).Z(end,:))-min(objects_cam2(k).Z(end,:)))/2)];
                dist=norm(centroid-cent);
                % verfying if its the same object
                if dist < 1.8 && h_err2>0.2
                    objects_cam2_hist(k).hist=vertcat(objects_cam2_hist(k).hist, hist2);
                    objects_cam2_hist(k).frames=horzcat(objects_cam2_hist(k).frames,i);
                    objects_cam2(k).X=vertcat(objects_cam2(k).X, [xmin xmin xmin xmin xmax xmax xmax xmax]);
                    objects_cam2(k).Y=vertcat(objects_cam2(k).Y, [ymin ymin ymax ymax ymin ymin ymax ymax]);
                    objects_cam2(k).Z=vertcat(objects_cam2(k).Z, [zmin zmax zmin zmax zmin zmax zmin zmax]);
                    objects_cam2(k).frames_tracked=horzcat(objects_cam2(k).frames_tracked, i);
                    n=1;
                end
                
            end
            % in case of a new object
            if n==0
                im=length(objects_cam2)+1;
                objects_cam2_hist(im).hist=hist2;
                objects_cam2_hist(im).frames=i;
                objects_cam2(im).X=[xmin xmin xmin xmin xmax xmax xmax xmax];
                objects_cam2(im).Y=[ymin ymin ymax ymax ymin ymin ymax ymax];
                objects_cam2(im).Z=[zmin zmax zmin zmax zmin zmax zmin zmax];
                objects_cam2(im).frames_tracked=i;
            end
            
        end
        
    end
end
% converting the objects identified in cam2 to the world coordinates
for i=1:length(objects_cam2)
    aux1=zeros(length(objects_cam2(i).X(:)),3);
    aux1(:,1)=objects_cam2(i).X(:);
    aux1(:,2)=objects_cam2(i).Y(:);
    aux1(:,3)=objects_cam2(i).Z(:);
    aux21=aux1*cam2toW.R+ones(length(aux1),1)*cam2toW.T';
    objects_cam2(i).X=reshape(aux21(:,1),length(aux21)/8,8);
    objects_cam2(i).Y=reshape(aux21(:,2),length(aux21)/8,8);
    objects_cam2(i).Z=reshape(aux21(:,3),length(aux21)/8,8);
    
end

% o1 = objects_cam1;
% o2 = objects_cam2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Checking all the objects identied in both cam to see if they are the same
% or new objects
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
c=1;
for i=1:length(objects_cam1)
    if isempty(objects_cam1)
        break
    end
    next1=0;
    for f1=1:length(objects_cam1(i).frames_tracked)
        match = 0;
        for j=1:length(objects_cam2)
            for f2=1:length(objects_cam2(j).frames_tracked)
                if objects_cam1(i).frames_tracked(f1)==objects_cam2(j).frames_tracked(f2)
                    cent1 = [max(objects_cam1(i).X(f1,:))-((max(objects_cam1(i).X(f1,:))-min(objects_cam1(i).X(f1,:)))/2), max(objects_cam1(i).Y(f1,:))-((max(objects_cam1(i).Y(f1,:))-min(objects_cam1(i).Y(f1,:)))/2), max(objects_cam1(i).Z(f1,:))-((max(objects_cam1(i).Z(f1,:))-min(objects_cam1(i).Z(f1,:)))/2)];
                    cent2 = [max(objects_cam2(j).X(f2,:))-((max(objects_cam2(j).X(f2,:))-min(objects_cam2(j).X(f2,:)))/2), max(objects_cam2(j).Y(f2,:))-((max(objects_cam2(j).Y(f2,:))-min(objects_cam2(j).Y(f2,:)))/2), max(objects_cam2(j).Z(f2,:))-((max(objects_cam2(j).Z(f2,:))-min(objects_cam2(j).Z(f2,:)))/2)];
                    dist = norm(cent1-cent2);
                    if dist<0.4
                        frames=horzcat(objects_cam1(i).frames_tracked,objects_cam2(j).frames_tracked);
                        xx=vertcat(objects_cam1(i).X,objects_cam2(j).X);
                        yy=vertcat(objects_cam1(i).Y,objects_cam2(j).Y);
                        zz=vertcat(objects_cam1(i).Z,objects_cam2(j).Z);
                        [uniq, indexs]=unique(frames);
                        objects(c).frames_tracked=uniq;
                        for k = 1:length(uniq)
                            objects(c).X(k,:)=xx(indexs(k),:);
                            objects(c).Y(k,:)=yy(indexs(k),:);
                            objects(c).Z(k,:)=zz(indexs(k),:);
                        end
                        objects_cam1(i)=[];
                        objects_cam2(j)=[];
                        c=c+1;
                        match = 1;
                        next1=1;
                        break;
                    end
                end
            end
            if match == 1
                break;
            end
        end
        if match == 0
            objects(c).frames_tracked=objects_cam1(i).frames_tracked;
            objects(c).X=objects_cam1(i).X;
            objects(c).Y=objects_cam1(i).Y;
            objects(c).Z=objects_cam1(i).Z;
            objects_cam1(i)=[];
            c=c+1;
            break
        end
        if next1 == 1
            break;
        end
    end
end
if ~isempty(objects_cam2)
    for j=1:length(objects_cam2)
        objects(c).frames_tracked=objects_cam2(j).frames_tracked;
        objects(c).X=objects_cam2(j).X;
        objects(c).Y=objects_cam2(j).Y;
        objects(c).Z=objects_cam2(j).Z;
        %           objects_cam2(j)=[];
        c=c+1;
    end
end


end
