function [objects] = track3D_part1(imgseq1, cam_params)

    imgs=zeros(480,640,length(imgseq1));
    imgsd=zeros(480,640,length(imgseq1));
    objects=struct('X', [], 'Y', [], 'Z', [],'frames_tracked',[]);
 
    for i=1:length(imgseq1)
        imgs(:,:,i)=rgb2gray(imread(imgseq1(i).rgb));
        load(imgseq1(i).depth);
        imgsd(:,:,i)=double(depth_array)/1000;
        figure(1);
        imagesc(imgsd(:,:,i));
        pause(0.01);
    end

    % Calculate BackGround
    bgdepth=median(imgsd(:,:,1:30),3);
    bgrgb=median(imgs(:,:,1:30),3);
    figure(2);
    subplot(1,2,1);
    imagesc(bgrgb);
    subplot(1,2,2);
    imagesc(bgdepth);


    for i=1:(length(imgseq1))
    
        figure(1);
        imagesc(imgsd(:,:,i));

        %BackGround Subtraction
        imdiff=abs(imgsd(:,:,i)-bgdepth)>.2;

        % Morfological Filter
        imgdiffiltered=imopen(imdiff,strel('disk',5));

        figure(3);
        imagesc(imgdiffiltered);

        value=find(imgdiffiltered==1);
        grad=zeros(480,640);
        dept=imgsd(:,:,i);
        grad(value)=dept(value);

        figure(4);
        [FX,FY]=gradient(grad);
        gradValX=abs(FX);
        gradValY=abs(FY);
        subplot(1,2,1);
        imagesc(gradValX);
        subplot(1,2,2);
        imagesc(gradValY);
        
        img_size = size(imgsd(:,:,i));
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

            if(~isempty(find(v > 0.15)>0) || ~isempty(find(u > 0.15)>0))
                imgdiffiltered(r(ind),c(ind)) = 0;
            end
        end

        figure(5);
        imagesc(imgdiffiltered);

        bw2=bwareaopen(imgdiffiltered,1500);

        [bw3,M]=bwlabel(bw2);
        figure(6);
        imagesc(bw3);

        
        figure(7);
        clf;
        for j=1:M        
            ind=find(bw3==j);
            load(imgseq1(i).depth);
            aux=zeros(480,640);
            aux(ind)=depth_array(ind);
            xyz1=get_xyz_asus(aux(:),[480 640], find(aux>0.2 & aux<6000), cam_params.Kdepth,1,0);
            pc1=pointCloud(xyz1);
            
            
            showPointCloud(pc1);

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
            %hold off;
            pause(0.5);
            
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
                    if dist < 0.4
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

