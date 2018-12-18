close all;
clear;

imgs_folder = 'images2';

img_rgb=dir(fullfile(imgs_folder,'*.png'));
img_depth=dir(fullfile(imgs_folder,'*.mat'));
img_rgbname={img_rgb.name};
img_depthname={img_depth.name};
rgb=natsortfiles(img_rgbname);
depth=natsortfiles(img_depthname);
imgseq1=repmat(struct('rgb',fullfile(imgs_folder,img_rgb(1).name),'depth',fullfile(imgs_folder,img_depth(1).name)), length(img_rgb)/2, 1);
imgseq2=repmat(struct('rgb',fullfile(imgs_folder,img_rgb(1).name),'depth',fullfile(imgs_folder,img_depth(1).name)), length(img_rgb)/2, 1);


for i=1:length(img_rgb)
    if i<=length(img_rgb)/2
       imgseq1(i)=struct('rgb',fullfile(imgs_folder,rgb(i)),'depth',fullfile(imgs_folder,depth(i)));
    else if i>length(img_rgb)/2
       imgseq2(i-(length(img_rgb)/2))=struct('rgb',fullfile(imgs_folder,rgb(i)),'depth',fullfile(imgs_folder,depth(i)));
        end
    end
end

load('cameraparametersAsus.mat');

[obj, cam2toW]=track3D_part2(imgseq1, imgseq2, cam_params);