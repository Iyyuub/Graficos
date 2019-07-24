%pcloudData=calibra(ptCloud);
% Initcializa sistema de aquisicao e aquire duas imagens
%load object3d.mat;
%%
function calibra_cam1(hObject)

load CalibrationData.mat
handles = guidata(hObject);
%ROS CAMERAS
topic_rgb =handles.topic_rgb;
topic_depth=handles.topic_depth;

data_rgb = receive(topic_rgb,10);
image_rgb = readImage(data_rgb);
data_depth = receive(topic_depth,10);
image_depth = readImage(data_depth);

imrgb_vec=reshape(image_rgb,[640*480,3]);
coord_depth=get_xyz_ros(image_depth(:),[480 640 ],1 :640*480,Depth_cam.K,0);
ptCloud=pointCloud(coord_depth,'color',imrgb_vec);

%%
pc=reshape(coord_depth,[480 640 3]);
im=image_rgb;
set(handles.figure1,'CurrentAxes',handles.axes1);
im(isnan(pc(:)))=0;
imagesc(im);
text(10,10,'Click CENTER');
go=0;
while ~go,
    [xc,yc]=ginput(1);
    hold on;plot(xc,yc,'+','MarkerSize',20);hold  off
    answer = questdlg('Is the center ok?','Center point','Ok','New Image','Ok');
    set(handles.figure1,'CurrentAxes',handles.axes1);
    switch answer,
        case 'Ok'
            go=1;
        case 'New Image'
            clf;
            data_rgb = receive(topic_rgb,10);
            im = readImage(data_rgb);
            data_depth = receive(topic_depth,10);
            image_depth = readImage(data_depth);
            imagesc(im);%adquire imagem nova
    end
end
pcentro=squeeze(pc(fix(yc),fix(xc),:));
go=0;
set(handles.figure1,'CurrentAxes',handles.axes1);
while ~go,
    [x1,y1]=ginput(1);
    hold on;plot(x1,y1,'+','MarkerSize',20);hold  off
    answer = questdlg('Is the X direction ok?','X Direction','Ok','New Image','Ok');
    set(handles.figure1,'CurrentAxes',handles.axes1);
    switch answer,
        case 'Ok'
            go=1;
        case 'New Image'
            clf;imagesc(im);%adquire imagem nova
            hold on;plot(xc,yc,'+','MarkerSize',20);hold  off
    end
end
%%
px=squeeze(pc(fix(y1),fix(x1),:));
[plane,inlierind,outlierind]=pcfitplane(ptCloud,0.05);
pcplaneCloud=select(ptCloud,inlierind);
err=plane.Parameters*[px;1];
if abs(err)>0.1,
    fprintf('second plane \n');
    ptCloudremain=select(ptCloud,outlierind);
    [plane,inlierind2,outlierind2]=pcfitplane(ptCloudremain,0.05);
    outliertot=outlierind;
    outlierind=[inlierind;outlierind(outlierind2)];
    inlierind=outliertot(inlierind2);
    if abs(plane.Parameters*[px;1]>0.1)
        close all;
        error('Second plane ...still wrong... too many planes !');
    end
end
pcplaneCloud=select(ptCloud,inlierind);
pcoutplaneCloud=select(ptCloud,outlierind);
set(handles.figure1,'CurrentAxes',handles.axes2);
pcshow(pcplaneCloud);view(0,90);
set(handles.figure1,'CurrentAxes',handles.axes4);
pcshow(pcoutplaneCloud);
pcplane=pcplaneCloud.Location'-pcentro*ones(1,length(pcplaneCloud.Location));
%%
[Rp s v]=svd(pcplane,'econ');
if det(Rp) <0,
    fprintf('SVD: Reflection instead of rotaion'); 
    Rp=Rp*diag([1 1 -1]);
end
%Rplane=[1 0 0;0 -1 0;0 0 -1]*Rp';
Rplane=Rp';
ppx=Rplane*(px-pcentro);
ppx=ppx/norm(ppx);
ppy=cross([0 0 1]',ppx);
Rplane=[ppx ppy [0;0;1]]'*Rplane;
%%
answer = questdlg('When you are ready press OK','Center point','Ok','Cancel','Ok');

pcoutplane=pcoutplaneCloud.Location'-pcentro*ones(1,length(pcoutplaneCloud.Location));
pplaneNorm=Rplane*pcplane;
poutplaneNorm=Rplane*pcoutplane;

set(handles.figure1,'CurrentAxes',handles.axes2);
pcshow(pointCloud(pplaneNorm','color',pcplaneCloud.Color));
xlabel('X ');ylabel('Y ');line([0 0;1 0],[0 0;0 1],zeros(2),'LineWidth',2);
set(handles.figure1,'CurrentAxes',handles.axes4);
pcshow(pointCloud(poutplaneNorm','color',pcoutplaneCloud.Color));
xlabel('X ');ylabel('Y ');
pcloudData.inlierind=inlierind;
pcloudData.outlierind=outlierind;
pcloudData.Rplane=Rplane;
pcloudData.centro=pcentro;
pcloudData.ppx=ppx;
pcloudData.ppy=ppy;
pcloudData.pcplaneCloud=pcplaneCloud;
pcloudData.pcoutplaneCloud=pcoutplaneCloud;
save PlaneData pcloudData