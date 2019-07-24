
%   rosinit('127.0.0.1');


%rostopic info /camera/rgb/image_color

%rostopic info /camera/depth/image


%%
%OSC
%uu=udp('192.168.0.100',57120);
%fopen(uu)
%Setting of variables
%lowestobj = .90; %  points below this distance from the ground are removed
%sizeIM=200; %size of the virtual image for connected componentes%
%objminsize=10; % Minimum number of pixels of connected cponents
%figures_on=1;%Display graphics or not

load PlaneData.mat
load CalibrationData.mat
topic_rgb = handles.topic_rgb;
%topic_depth = rossubscriber('/camera/depth/image');
%%%%Atention - changed topic
topic_depth=handles.topic_depth;

data_rgb = receive(topic_rgb,10);
data_depth = receive(topic_depth,10);

%data_depth_rect = receive(topic_depth_rect,10);
image_rgb = readImage(data_rgb);
image_depth = readImage(data_depth);

Origin=pcloudData.centro*ones(1,640*480);
Rot_plan=pcloudData.Rplane;
%%Clean figures
set(handles.figure1,'CurrentAxes',handles.axes1);cla;hold off
set(handles.figure1,'CurrentAxes',handles.axes2);cla;hold off
set(handles.figure1,'CurrentAxes',handles.axes3);cla;hold off
set(handles.figure1,'CurrentAxes',handles.axes4);cla;hold off
plot(rand(3),rand(3));
%%
%create variables
coord_depth=single(zeros(480*640,3));
% creating an image
IM = zeros(handles.sizeIM+1,handles.sizeIM+1);
%

%%
%set figure size
data_rgb = receive(topic_rgb,10);
data_depth = receive(topic_depth,10);
image_rgb = readImage(data_rgb);
image_depth = readImage(data_depth);


%%
tracks=NaN(50,20,2);
start=1;
while(RUN)
    tic
    handles=guidata(hObject);
    figures_on=handles.graphics_on;
    
    data_rgb = receive(topic_rgb,10);
    data_depth = receive(topic_depth,10);
    
    image_rgb = readImage(data_rgb);
    image_depth = readImage(data_depth);
    imrgb_vec=reshape(image_rgb,[640*480,3]);
    coord_depth=get_xyz_ros(image_depth(:),[480 640 ],1 :640*480,Depth_cam.K,0);
    %projection on the ground     
    Pt = Rot_plan*(coord_depth' -Origin );
    %Remove points below threshold
    idx_proj = find(Pt(3,:)>handles.lowestobj);
    x_p = Pt(1,idx_proj);
    y_p = Pt(2,idx_proj);
    %HACK ----min values defined in the startup
    %max_x=max(x_p);
    max_x=handles.chao(2);
    %max_y=max(y_p);
    max_y=handles.chao(4);
%    min_x=min(x_p);
    min_x=handles.chao(1);
%    min_y=min(y_p);
    min_y=handles.chao(3);
    x_p(x_p<handles.chao(1))=handles.chao(1);
    x_p(x_p>handles.chao(2))=handles.chao(2);
    y_p(y_p<handles.chao(3))=handles.chao(3);
    y_p(y_p>handles.chao(4))=handles.chao(4);
    % UNTIL HERE - should put this automatic
    IM(:)=0;
    u=handles.sizeIM*(x_p-min_x)/(max_x-min_x)+1;
    v=handles.sizeIM*(y_p-min_y)/(max_y-min_y)+1;
    inds=sub2ind(size(IM),round(v),round(u));
    for i=1:length(inds);IM(inds(i))=IM(inds(i))+1;end;
    
    %connected components - OBJ MASS SHOULD BE TESTED LATER...
    [labeled,numObjects]=bwlabel(IM>handles.objmass,8);
    % find the center of objects
    s = regionprops(labeled,'Centroid','Area');
    objects=zeros(length(s),4);
    for k = 1:length(s)
        if (s(k).Area >handles.objminsize)&&(s(k).Area <handles.objmaxsize),
            xc=(s(k).Centroid(1)-1)*(max_x-min_x)/handles.sizeIM+min_x;
            yc=(s(k).Centroid(2)-1)*(max_y-min_y)/handles.sizeIM+min_y;
            objects(k,:)=[xc yc s(k).Area 1];
%            oscsend(handles.osc,'/cada/multi','ifff',k,xc,yc,double(s(k).Area));
        end
    end
    fprintf('-------------\n');
    toc;
    if figures_on,
        set(handles.figure1,'CurrentAxes',handles.axes1);
        imagesc(imresize(image_rgb,[240 320]));
        set(handles.figure1,'CurrentAxes',handles.axes2);cla;
        pcshow(pcdownsample(pointCloud(Pt','color',reshape(image_rgb,[480*640 3])),'gridAverage',0.01));
        %view(180,45);
        set(handles.figure1,'CurrentAxes',handles.axes3);
        imagesc(labeled);
        hold on
        objectsgood=objects(objects(:,4)>0,:);
        
        for k = 1:size(objectsgood,1)
                plot(objectsgood(k,1),objectsgood(k,2),'o','MarkerSize',20);
        end
        fprintf('Number of objects %d \n',size(objectsgood,1));
        hold off;grid on;
        set(handles.figure1,'CurrentAxes',handles.axes4);cla;
        hold on;
        fprintf('Objects %d \n',size(objects,1))
        plot(0,0,'+','MarkerSize',20);axis(handles.chao);
        for k=1:size(objects,1),
            if objects(k,4),% only plot if passes test
                plot(objects(k,1),objects(k,2),'o','MarkerSize',20);
                text(objects(k,1),objects(k,2),num2str(objects(k,3)));
            end
        end
        hold off
    end
    toc
    drawnow;
    %% Tracking
    if start==1
        X=objectsgood(:,1:2);
        tracks(1,1:size(X,1),:)=X;
        frame=2;
        start=0;
    else
        Y=objectsgood(:,1:2);
        [Idx, D]=knnsearch(X,Y,'K',3,'Distance','euclidean');
        C=10*ones(size(Y,1),size(X,1));
        for i=1:size(Y,1)
            C(i,Idx(i,:))=D(i,:);
        end
        [M,uR,uC]=matchpairs(C,10);
        set(handles.figure1,'CurrentAxes',handles.axes4);
        plot(X(:,1),X(:,2),'x',Y(:,1),Y(:,2),'o'); axis(handles.chao);
        hold on
        k=1;
        if frame==1 l=50, else l=frame-1, end
        for j=1:20
            for i=1:size(M,1)
                if(tracks(l,j,1)==X(M(i,2),1) & tracks(l,j,2)==X(M(i,2),2))
                    tracks(frame,j,:)=Y(M(i,1),:);
                end
            end
            for i=1:size(uC)
                if(tracks(l,j,1)==X(uC(i),1) & tracks(l,j,2)==X(uC(i),2))
                    tracks(:,j,:)=NaN;
                end
            end
            if((isnan(tracks(frame,j,:))) & (k<=size(uR)))
                tracks(frame,j,:)=Y(uR(k),:);
                k=k+1
            end

        end
        set(handles.figure1,'CurrentAxes',handles.axes4);
        plot(tracks(:,:,1),tracks(:,:,2));
        X=Y;
        frame=mod(frame,50)+1;
    end

end
RUN=0;







