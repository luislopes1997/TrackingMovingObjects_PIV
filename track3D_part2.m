function [objects,cam2toW] = track3D_part2(imgseq1, imgseq2, cam_params)

imgs1=zeros(480,640,length(imgseq1));
imgsd1=zeros(480,640,length(imgseq1));
imgs2=zeros(480,640,length(imgseq2));
imgsd2=zeros(480,640,length(imgseq2));
objects=struct('X', [], 'Y', [], 'Z', [],'frames_tracked',[]);

    for i=1:length(imgseq1)
        imgs1(:,:,i)=rgb2gray(imread(imgseq1(i).rgb));
        imgs2(:,:,i)=rgb2gray(imread(imgseq2(i).rgb));
        load(imgseq1(i).depth);
        imgsd1(:,:,i)=double(depth_array)/1000;
        figure(1);
        subplot(1,2,1);
        imagesc(imgsd1(:,:,i));
        load(imgseq2(i).depth);
        imgsd2(:,:,i)=double(depth_array)/1000;
        subplot(1,2,2);
        imagesc(imgsd2(:,:,i));
        pause(0.1);

    end
    
    % Calculate BackGround
    bgdepth_cam1=median(imgsd1(:,:,:),3);
    bgrgb_cam1=median(imgs1(:,:,:),3);
    bgrgb_cam2=median(imgs2(:,:,:),3);
    bgdepth_cam2=median(imgsd2(:,:,:),3);
    figure(2);
    subplot(2,2,1);
    imagesc(bgdepth_cam1);
    subplot(2,2,2);
    imagesc(bgdepth_cam2);
    subplot(2,2,3);
    imagesc(bgrgb_cam1);
    subplot(2,2,4);
    imagesc(bgrgb_cam2);
    
%     % Harris corner detector (good points)
%     figure(4);
%     subplot(1,2,1);
%     [cim1, r1, c1] = harris(bgrgb_cam1, 2, 300, 2, 0);
%     imagesc(bgrgb_cam1);
%     hold on;
%     plot(c1,r1,'r+');
%     subplot(1,2,2);
%     [cim2, r2, c2] = harris(bgrgb_cam2, 2, 300, 2, 0);
%     imagesc(bgrgb_cam2);
%     hold on;
%     plot(c2,r2,'r+');
    
    % Another Harris implemation same results
%     figure();
%     corners = detectHarrisFeatures(bgrgb_cam2, 'FilterSize', 3);
%     imshow(bgrgb_cam2); hold on;
%     plot(corners.selectStrongest(300));

    % find matching points using SIFT
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
    
    %Matching for the image with the most matches
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
    figure(3);
    subplot(1,2,1);
    imagesc(imgs1(:,:,index));
    hold on;
    plot(cam1(:,1),cam1(:,2),'*r');
    hold off;
    subplot(1,2,2);
    imagesc(imgs2(:,:,index));
    hold on;
    plot(cam2(:,1),cam2(:,2),'*r');
    hold off;
    
    %image with the matched points
    figure(4);
    ax = axes;
    showMatchedFeatures(imgs1(:,:,index), imgs2(:,:,index), cam1, cam2,'montage','Parent',ax);
    
    
    
    %Ransac    
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
    
    [~, index]= max(numinliers);
    R=R(:,:,index);
    T=T(:,:,index);
    error=xyz_points1-xyz_points2*R-ones(length(xyz_points2),1)*T(1,:);
    inds=find(sum(error.*error,2)<error_thresh^2);
    xyz_points1=xyz_points1(inds,:);
    xyz_points2=xyz_points2(inds,:);
    [~,~,trans]=procrustes(xyz_points1,xyz_points2,'scaling',false,'reflection',false); 
    xyz21=xyz_cam2*trans.T+ones(length(xyz_cam2),1)*trans.c(1,:);
    xyz_total=[xyz_cam1(:,1) xyz_cam1(:,2) xyz_cam1(:,3); xyz21(:,1) xyz21(:,2) xyz21(:,3)];
    rgbd1=reshape(rgbd_cam1,[640*480 3]);
    rgbd2=reshape(rgbd_cam2,[640*480 3]);
    rgbd_total=[rgbd1(:,1) rgbd1(:,2) rgbd1(:,3); rgbd2(:,1) rgbd2(:,2) rgbd2(:,3)];
    pc1=pointCloud(xyz_cam1,'Color',rgbd1);
    pc2=pointCloud(xyz_cam2,'Color',rgbd2);
    pc3=pointCloud(xyz21,'Color',rgbd2);
    pc4=pointCloud(xyz_total,'Color',rgbd_total);
    figure(5);
    subplot(2,2,1);showPointCloud(pc1);    
    subplot(2,2,2);showPointCloud(pc2);
    subplot(2,2,3);showPointCloud(pc3);
    subplot(2,2,4);showPointCloud(pc4);
    cam2toW=struct('R',trans.T,'T', trans.c(1,:)');
    
    for i=1:(length(imgseq2))
        %BackGround Subtraction
        load(imgseq1(i).depth);
        xyz_cam1=get_xyz_asus(depth_array(:),[480 640], 1:480*640, cam_params.Kdepth ,1,0);
        load(imgseq2(i).depth);
        xyz_cam2=get_xyz_asus(depth_array(:),[480 640], 1:480*640, cam_params.Kdepth ,1,0);
        xyz21=xyz_cam2*cam2toW.R+ones(length(xyz_cam2),1)*cam2toW.T';
        xyz_total=[xyz_cam1(:,1) xyz_cam1(:,2) xyz_cam1(:,3); xyz21(:,1) xyz21(:,2) xyz21(:,3)];
        im_total(:,:,i)=cam_params.Kdepth*[cam2toW.R cam2toW.T]*[xyz_total' ; ones(1,length(xyz_total))];
        %figure()
        imdiff=abs(imgsd2(:,:,i)-bgdepth_cam2)>.2;

        % Morfological Filter
        imgdiffiltered=imopen(imdiff,strel('disk',5));

        figure(8);
        imagesc(imgdiffiltered);
        value=find(imgdiffiltered==1);
        grad=zeros(480,640);
        dept=imgsd2(:,:,i);
        grad(value)=dept(value);

        figure(9);
        [FX,FY]=gradient(grad);
        gradValX=abs(FX);
        gradValY=abs(FY);
        subplot(1,2,1);
        imagesc(gradValX);
        subplot(1,2,2);
        imagesc(gradValY);
        
        img_size = size(imgsd2(:,:,i));
        [I, J] = ind2sub(img_size(1:2), find(imgdiffiltered == 1));
        r = I(I > 1 & I < img_size(1) & J > 1 & J < img_size(2));
        c = J(I > 1 & I < img_size(1) & J > 1 & J < img_size(2));
        indlen = length(r);
        for ind = 1:indlen
            neigh_v = [gradValX(r(ind)-1,c(ind)-1)
                       gradValX(r(ind)-1,c(ind))
                       gradValX(r(ind)-1,c(ind)+1)
                       gradValX(r(ind),c(ind)-1)
                       gradValX(r(ind),c(ind)+1)
                       gradValX(r(ind)+1,c(ind)-1)
                       gradValX(r(ind)+1,c(ind))
                       gradValX(r(ind)+1,c(ind)+1)];
            v = abs(gradValX(r(ind),c(ind)) - neigh_v);
            neigh_u = [gradValY(r(ind)-1,c(ind)-1)
                       gradValY(r(ind)-1,c(ind))
                       gradValY(r(ind)-1,c(ind)+1)
                       gradValY(r(ind),c(ind)-1)
                       gradValY(r(ind),c(ind)+1)
                       gradValY(r(ind)+1,c(ind)-1)
                       gradValY(r(ind)+1,c(ind))
                       gradValY(r(ind)+1,c(ind)+1)];
            u = abs(gradValY(r(ind),c(ind)) - neigh_u);

            if(~isempty(find(v > 0.2)>0) || ~isempty(find(u > 0.2)>0))
                imgdiffiltered(r(ind),c(ind)) = 0;
            end
        end

        figure(10);
        imagesc(imgdiffiltered);

        bw2=bwareaopen(imgdiffiltered,1000);

        [bw3,M]=bwlabel(bw2);
        figure(11);
        imagesc(bw3);

        
        figure(12);
        clf;
        for j=1:M        
            ind=find(bw3==j);
            load(imgseq2(i).depth);
            aux=zeros(480,640);
            aux(ind)=depth_array(ind);
            xyz_cam2=get_xyz_asus(aux(:),[480 640], find(aux>0.2 & aux<6000), cam_params.Kdepth,1,0);
            pc1=pointCloud(xyz_cam2);  
            showPointCloud(pc1);

            Z=pc1.Location(:,3);
            zmax=max(Z)
            zmin=min(Z(Z~=0));

            Y=pc1.Location(:,2);
            ymax=max(Y)
            ymin=min(Y(Y~=0))

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
            hold on;
            plot3(X,Y,Z,'r');   
            X = [xmax;xmax;xmax;xmax;xmax];
            hold on;
            plot3(X,Y,Z,'r'); 
    
            Z = [zmin;zmin;zmin;zmin;zmin];
            X = [xmin;xmin;xmax;xmax;xmin];
            Y = [ymin;ymax;ymax;ymin;ymin];
            hold on;
            plot3(X,Y,Z,'r');   
            Z = [zmax;zmax;zmax;zmax;zmax];
            hold on;
            plot3(X,Y,Z,'r'); 
            pause(0.1);
           
            % tracking
            centroid=[xmax-((xmax-xmin)/2), ymax-((ymax-ymin)/2), zmax-((zmax-zmin)/2)];
            
            if (isempty(objects(1).X)==1)
                objects(1).X=[xmin xmin xmin xmin xmax xmax xmax xmax];
                objects(1).Y=[ymin ymin ymax ymax ymin ymin ymax ymax];
                objects(1).Z=[zmin zmax zmin zmax zmin zmax zmin zmax];
                objects(1).frames_tracked=[i];
            else 
                n=0;
                for k = 1:length(objects)
                    if objects(k).frames_tracked(end)~=i-1
                        continue;
                    end
                    cent = [max(objects(k).X(end,:))-((max(objects(k).X(end,:))-min(objects(k).X(end,:)))/2), max(objects(k).Y(end,:))-((max(objects(k).Y(end,:))-min(objects(k).Y(end,:)))/2), max(objects(k).Z(end,:))-((max(objects(k).Z(end,:))-min(objects(k).Z(end,:)))/2)];
                    dist=norm(centroid-cent);
                    if dist < 1.8
                        objects(k).X=vertcat(objects(k).X, [xmin xmin xmin xmin xmax xmax xmax xmax]);
                        objects(k).Y=vertcat(objects(k).Y, [ymin ymin ymax ymax ymin ymin ymax ymax]);
                        objects(k).Z=vertcat(objects(k).Z, [zmin zmax zmin zmax zmin zmax zmin zmax]);
                        objects(k).frames_tracked=horzcat(objects(k).frames_tracked, i);
                        n=1;
                    end
                      
                end 
                if n==0
                    im=length(objects)+1;
                    objects(im).X=[xmin xmin xmin xmin xmax xmax xmax xmax];
                    objects(im).Y=[ymin ymin ymax ymax ymin ymin ymax ymax];
                    objects(im).Z=[zmin zmax zmin zmax zmin zmax zmin zmax];
                    objects(im).frames_tracked=i;
                end
 
            end                                                                     
                      
        end   
         
    end
   
end
   