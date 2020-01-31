% Function to read a bag file of a pointcloud and convert it to pointcloud
% object of Matlab:
% - Arguments:
%     * name_bag - Name of the bag

function [nube] = from_bag_to_var(name_bag)

filescloud = rosbag(name_bag);
bagClouds = select(filescloud ,'Topic','/cloud_timestamp_2');
rosClouds = readMessages(bagClouds);
Cloud_pointcloud1 = rosClouds{1};
Cloud_xyz = readXYZ(Cloud_pointcloud1);
nube = pointCloud(Cloud_xyz);
end

