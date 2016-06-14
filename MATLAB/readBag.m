% @author: Jon Gonzales
% Requirements: 
%   * Robotics System Toolbox
%   * Custom message support

% To install custon message support
% link:     http://www.mathworks.com/matlabcentral/fileexchange/49810-robotics-system-toolbox-interface-for-ros-custom-messages
% examples: http://www.mathworks.com/help/robotics/examples/work-with-specialized-ros-messages.html

%% Install custom barc messages
% Notes: 
%   * Uncomment the following commands to install custom messages and
%       replace 'path_to_msggen' with your system path to that folder
%   * Restart MATLAB after this step for the changes to take effect 
%   * If you add / delete / modify msg types, delete the matlabgen folder
%       within the packages folder

% rosgenmsg('packages')     % follow the output instructions
                            % file may be created if non-existent
% addpath('path_to_msggen')
% savepath('path_to_msggen')

%% select bag file
[fileName,pathName] = uigetfile({'data/*.bag'},'Select bag file');
filePath = strcat(pathName,fileName);

%%  Open rosbag 
bag     = rosbag(filePath);    % open bag file
msgs    = readMessages(bag);                    % get all messages
t0      = bag.StartTime;                        % get initial time
bag.AvailableTopics                             % display all recorded topics

%% extract messages
% get individual topic bags
bag_imu     = select(bag,'Topic','/imu/data');
bag_enc     = select(bag,'Topic','/encoder');
bag_ecu     = select(bag,'Topic','/ecu');
bag_img     = select(bag,'Topic','/camera/image_raw/compressed');

% all message data
imu             = readMessages(bag_imu);
encoder         = readMessages(bag_enc);
ecu             = readMessages(bag_ecu);
compressed_imgs = readMessages(bag_img);

% number of messages
n_imu_msg       = size(imu,1);
n_enc_msg       = size(encoder,1);
n_ecu_msg       = size(ecu,1);
n_img_msg       = size(compressed_imgs,1);

%% process imu data
if n_imu_msg > 0
    t_imu   = bag_imu.timeseries.Time - t0;     % time series
    f_imu   = 1/mean(diff(t_imu));              % average sampling frequency
    a_x     = zeros(1,n_imu_msg);
    a_y     = zeros(1,n_imu_msg);  
    yaw     = zeros(1,n_imu_msg);
    w_z     = zeros(1,n_imu_msg);
    for i = 1:n_imu_msg
        a_x(i)  = imu{i}.LinearAcceleration.X;
        a_y(i)  = imu{i}.LinearAcceleration.Y;
        w_z(i)  = imu{i}.AngularVelocity.Z;
        eul     = quat2eul([imu{i}.Orientation.X,...
                                imu{i}.Orientation.Y,...
                                imu{i}.Orientation.Z,...
                                imu{i}.Orientation.W]);
        yaw(i)  = eul(3);
    end
    plot(t_imu, a_x);
    plot(t_imu, a_y);
    plot(t_imu, yaw);
    plot(t_imu, w_z);
end

%% process encoder data
if n_enc_msg > 0
    t_enc   = bag_enc.timeseries.Time - t0;     % time series
    f_enc   = 1/mean(diff(t_enc));              % average sampling frequency
    n_FL    = zeros(1,n_enc_msg);
    n_FR    = zeros(1, n_enc_msg);
    for i=1:n_enc_msg
        n_FL(i)  = encoder{i}.FL;
        n_FR(i)  = encoder{i}.FR;
    end
    plot(t_enc, n_FL); hold on;
    plot(t_enc, n_FR); grid on;
end

%% process ecu data
if n_ecu_msg > 0
    t_ecu       = bag_ecu.timeseries.Time - t0;     % time series
    f_ecu       = 1/mean(diff(t_ecu));              % average sampling frequency
    u_motor_pwm = zeros(1,n_ecu_msg);
    u_servo_pwm = zeros(1, n_ecu_msg);
    for i=1:n_ecu_msg
        u_motor_pwm(i)  = ecu{i}.MotorPwm;
        u_servo_pwm(i)  = ecu{i}.ServoPwm;
    end
    plot(t_ecu, u_motor_pwm)
end

%% process image data
if n_img_msg > 0
    t_img           = bag_img.timeseries.Time - t0;     % time series
    f_img           = 1/mean(diff(t_img));              % average sampling frequency
    img0            = readImage(compressed_imgs{1});
    height          = size(img0,1);
    width           = size(img0,2);
    depth           = size(img0,3);
    all_imgs        = zeros(height,width,depth,n_img_msg,'uint8');
    for i=1:n_img_msg
            all_imgs(:,:,:,i) = readImage(compressed_imgs{i});
    end
    implay(all_imgs,32);
end

%%
display(sprintf('avg enc sampling frequency \t: %f \t[Hz]', f_enc));
display(sprintf('avg imu sampling frequency \t: %f \t[Hz]', f_imu));
display(sprintf('avg img sampling frequency \t: %f \t[Hz]', f_img)); 
display(sprintf('avg ecu command frequency \t: %f \t[Hz]', f_ecu));
display(sprintf('\nPress the green play button to watch the video'));