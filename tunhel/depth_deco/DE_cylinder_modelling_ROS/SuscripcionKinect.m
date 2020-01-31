rosinit('127.0.0.1');

c = Cyl_modelling();

sub = rossubscriber('/HoleCenter','geometry_msgs/Point',@c.callbackCenter);
subd2 = rossubscriber('/ROICloud',@c.callbackCloud);
