function  [global_robot_pos,est_global_target_pos] = from_local_to_global( robot_moves, est_local_target_pos)
%FROM_LOCAL_TO_GLOBAL Transforms the local coordinates of the target into
%global coordinates
%   Given the (known) robot position and the estimated local coordinates of
%   the target, returns the estimated global coordinates of the target

%assumption 1: centre of global coordinate system = first robot position
%hence we can simply use u to compute the robot position in global
%coordinates

endT = size(robot_moves,2);
global_robot_pos = zeros(size(robot_moves,1),size(robot_moves,2)) ;%inizializza a zero tutto l'array
est_global_target_pos = zeros(size(robot_moves,1),size(robot_moves,2)) ;
size(est_global_target_pos(:,1))
size(est_local_target_pos(:,1))
est_global_target_pos(:,1) = est_local_target_pos(:,1); %given assumption 1, local target pos = global target pos at time 1
for i = 2:1:endT
    %assume robot translates by the amount specified by the control command robot_moves (i.e.
    %u) <- we actually should use a matrix to model more complex
    %relationships between command and motion ...
    %at time 1, robot is in (0,0), the control at time 1 influences the
    %position at time 2
    global_robot_pos(:,i) = global_robot_pos(:,i-1)+robot_moves(:,i-1);
    est_global_target_pos(:,i) = global_robot_pos(:,i) + est_local_target_pos(:,i) ;
end

end


