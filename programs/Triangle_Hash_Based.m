%%
myTrainingDir = 'C:\Users\abaum\Documents\data\CoopScenes\seq_1\pcl\train';
myFiles = dir(fullfile(myTrainingDir,'*.pcd'));
numFiles = length(myFiles);
% veloReader = velodyneFileReader(fullfile(myDir, test_file_name),'VLS128');
%%
xlimits = [-150 150];
ylimits = [-150 150];
zlimits = [-12 25];

% KEIN pcplayer nötig, nur Limits behalten
% player = pcplayer(xlimits,ylimits,zlimits);
% xlabel(player.Axes,'X (m)');
% ylabel(player.Axes,'Y (m)');
% zlabel(player.Axes,'Z (m)');

% player2 = pcplayer(xlimits,ylimits,zlimits);
% xlabel(player2.Axes,'X (m)');
% ylabel(player2.Axes,'Y (m)');
% zlabel(player2.Axes,'Z (m)');

%%

total_frame = numFiles;

FOV_vert = 22.5;
FOV_horiz = 360;
vert_total_grids = 128;
azimuth_resltn = 0.3515625;
elevation_resltn = FOV_vert/vert_total_grids;
horiz_total_grids = FOV_horiz/azimuth_resltn;

tensor_size = [vert_total_grids, horiz_total_grids, total_frame];
range_matrix = zeros(tensor_size);
frame_num = 0;

for k = 1:length(myFiles)
    % if ~player.isOpen()
    %     break;
    % end
    fprintf('Training: file %d/%d verarbeitet (frame_num=%d, %s)\n', ...
        k, numFiles, frame_num, baseFileName);

    baseFileName = myFiles(k).name;
    fullFileName = fullfile(myTrainingDir, baseFileName);

    ptCloud = pcread(fullFileName);

%     [ptCloud_roi_filtered] = roi_filter(ptCloudObj,roi_mask,xLim,yLim);

    % view(player, ptCloud.Location, ptCloud.Intensity);
%     pause(0.01);

    frame_num = frame_num + 1;

    if (frame_num > total_frame)

%         str_raw_lidar = sprintf('raw_lidarData');
%         save(str_raw_lidar, 'raw_lidarData');

        break;

    end

    % Read the PCD file using your own code
    xyz_positions = reshape(ptCloud.Location,[],3);
    intensity_values = reshape(ptCloud.Intensity,[],1);
    x = xyz_positions(:,1)';
    y = xyz_positions(:,2)';
    z = xyz_positions(:,3)';

    [azimuth, elevation, r] = cart2sph(x, y, z);

    % apply hash function to transform into tensor format
    azimuth_degree = rad2deg(azimuth);
    azimuth_degree(azimuth_degree<0) = azimuth_degree(azimuth_degree<0) + FOV_horiz;
    azimuth_degree_shift = azimuth_degree;
    azimuth_grids_idx = mod(floor(azimuth_degree_shift/azimuth_resltn) + 1, horiz_total_grids);
    azimuth_grids_idx(isnan(azimuth_grids_idx)) = 0; % non-return points

    elevation_degree = rad2deg(elevation);
    elevation_degree(elevation_degree<0) = elevation_degree(elevation_degree<0) + FOV_vert;
    elevation_degree_shift = elevation_degree;
    elevation_grids_idx = mod(floor(elevation_degree_shift/elevation_resltn) + 1, vert_total_grids);
    elevation_grids_idx(isnan(elevation_grids_idx)) = 0; % non-return points

    for j = 1: length(azimuth_grids_idx)

        channel_idx = elevation_grids_idx(j);
        azimuth_idx = azimuth_grids_idx(j);

        if azimuth_idx ~= 0 && channel_idx ~= 0 % no point cloud data

           if (range_matrix(channel_idx, azimuth_idx, frame_num) == 0) || range_matrix(channel_idx, azimuth_idx, frame_num) > r(j)  % keep the closer point

               range_matrix(channel_idx, azimuth_idx, frame_num) = r(j);

          end

        end

    end

end

%% train background range thresholds
dist_max = 200;

range_thrld_matrix = ones(vert_total_grids, horiz_total_grids) * dist_max;

for i = 1 : vert_total_grids

    for j = 1 : horiz_total_grids
    
        distances = reshape(range_matrix(i,j,:),[],1);
        
        thrld_value = thresholding(distances);
        
        range_thrld_matrix(i,j) = thrld_value;
        
    end 
    
end

%%  save filtered results points
frame_num = 0;
saveBackground = 0;
%frames = [200, 400, 600, 800, 1000, 1500, 2000];

myTestingDir = 'C:\Users\abaum\Documents\data\CoopScenes\seq_1\pcl\test';
myTestFiles = dir(fullfile(myTestingDir,'*.pcd'));
lidarData = {};
outputFolder = "C:\Users\abaum\Documents\data\CoopScenes\seq_1\pcl\baseline";
learning_rate = 1e-9;
backgroundOutFolder = "./AlbanyGeorge/Backgrounds/Triangle";

if ~exist(outputFolder, 'dir')
   mkdir(outputFolder)
end

if ~exist(backgroundOutFolder, 'dir')
   mkdir(backgroundOutFolder)
end

for k = 1:length(myTestFiles)
    % if ~player.isOpen()
    %     break;
    % end

    fullFileName = fullfile(myTestingDir, myTestFiles(k).name);
    ptCloudObj = pcread(fullFileName);

    frame_num = frame_num + 1;

    [ptCloudOut, ~] = cropPointCloud(ptCloudObj, xlimits, ylimits, zlimits);
    
    pointCloud_x = ptCloudOut.Location(:,1);
    pointCloud_y = ptCloudOut.Location(:,2);
    pointCloud_z = ptCloudOut.Location(:,3);
    
    intensities = ptCloudOut.Intensity;
    
    [azimuths, elevations, ranges] = cart2sph(pointCloud_x, pointCloud_y, pointCloud_z);
    
    % apply hash function to transform into tensor format
    azimuth_degree = rad2deg(azimuths);
    azimuth_degree(azimuth_degree<0) = azimuth_degree(azimuth_degree<0) + FOV_horiz;
    azimuth_degree_shift = azimuth_degree;
    azimuth_grids_idx = mod(floor(azimuth_degree_shift/azimuth_resltn) + 1, horiz_total_grids);
    azimuth_grids_idx(isnan(azimuth_grids_idx)) = 0; % non-return points

    elevation_degree = rad2deg(elevations);
    elevation_degree(elevation_degree<0) = elevation_degree(elevation_degree<0) + FOV_vert;
    elevation_degree_shift = elevation_degree;
    elevation_grids_idx = mod(floor(elevation_degree_shift/elevation_resltn) + 1, vert_total_grids);
    elevation_grids_idx(isnan(elevation_grids_idx)) = 0; % non-return points
    
    back_idxes = zeros(size(intensities));

     for pnt_idx = 1: length(azimuth_grids_idx)
    
        channel_idx = elevation_grids_idx(pnt_idx);
        azimuth_idx = azimuth_grids_idx(pnt_idx);
        
        range = ranges(pnt_idx);
        
        intensity = intensities(pnt_idx);
        
        elevation = elevations(pnt_idx);
        
        grid_idx = azimuth_grids_idx(pnt_idx);

        if grid_idx == 0 || channel_idx == 0
            
            back_idxes(pnt_idx) = 1;
            continue;

        end
        
        % make range comparison, background points are further 
        distance_flag = double(range) >= range_thrld_matrix(channel_idx, grid_idx);
        
        if distance_flag
            
            back_idxes(pnt_idx) = 1;
            
        end
        
    end
    
    if saveBackground == 1

        dmd_range_filtered_x = pointCloud_x(back_idxes == 1);
        dmd_range_filtered_y = pointCloud_y(back_idxes == 1);
        dmd_range_filtered_z = pointCloud_z(back_idxes == 1);
        range_filtered_intensities = intensities(back_idxes == 1);

    else

        range_filtered_x = pointCloud_x(back_idxes == 0);
        range_filtered_y = pointCloud_y(back_idxes == 0);
        range_filtered_z = pointCloud_z(back_idxes == 0);
        range_filtered_intensities = intensities(back_idxes == 0);

    end
    
    ptCloud_filtered.Location = [range_filtered_x, range_filtered_y, range_filtered_z];
    ptCloud_filtered.Intensity = range_filtered_intensities;
    
    ptCloud_obj = pointCloud(ptCloud_filtered.Location, 'Intensity', ptCloud_filtered.Intensity);

    ptCloud_obj = pcdenoise(ptCloud_obj); % remove noise

%    [ptCloud_obj] = dbcanDenoise(ptCloud_obj, 1.5, 10);

%         pcshowpair(ptCloudObj, ptCloud_obj);
%         xlim([xlimits(1) xlimits(2)])
%         ylim([ylimits(1) ylimits(2)])
%         zlim([zlimits(1) zlimits(2)])

%    view(player, ptCloud_obj.Location, ptCloud_obj.Intensity);
%         pause(0.5);

%    view(player2, ptCloudOut.Location, ptCloudOut.Intensity);

    lidarData{end + 1} = ptCloud_obj;

   % if  ismember(frame_num, frames)

    str_e = sprintf('CFTA_frame_%d.ply',frame_num);

    if saveBackground == 1

        outputPath = fullfile(backgroundOutFolder, str_e);

    else

        outputPath = fullfile(outputFolder, str_e);

    end

    pcwrite(ptCloud_obj, outputPath);

  %  end
    
end

str_lidarData = "bgmodel.mat";
save(str_lidarData, 'lidarData');
