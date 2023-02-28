function pos_ts = intergrate_vel_ts(vel_ts)
% integrate a time series 

time_list = vel_ts.Time;
pos_data = zeros(size(vel_ts.Data));

for i=2:length(time_list)
    dt = time_list(i) - time_list(i-1);
    pos_data(i,:) = pos_data(i-1,:) + dt* vel_ts.Data(i-1,:);
end

pos_ts = timeseries(pos_data,time_list,'Name',"integrate_time");

end