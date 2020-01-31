filescloud = rosbag('3.bag');
bagClouds = select(filescloud ,'Topic','/cloud_timestamp_2');
rosClouds = readMessages(bagClouds);
Cloud_pointcloud1 = rosClouds{1};
Cloud_xyz = readXYZ(Cloud_pointcloud1);
nube1 = pointCloud(Cloud_xyz);


filescloud = rosbag('4.bag');
bagClouds = select(filescloud ,'Topic','/cloud_timestamp_2');
rosClouds = readMessages(bagClouds);
Cloud_pointcloud3 = rosClouds{1};
Cloud_xyz = readXYZ(Cloud_pointcloud3);
nube3 = pointCloud(Cloud_xyz);
tform = rotm2tform(eul2rotm(deg2rad([180,0,0]),'XYZ'));
tform = affine3d(tform);
nube3 = pctransform(nube3, tform);