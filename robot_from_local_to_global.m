function global_robot_pos = robot_from_local_to_global(robot_moves)
endT = size(robot_moves,2);
global_robot_pos = zeros(2,size(robot_moves,2)) ;%inizializza a zero tutto l'array
for i = 2:1:endT
    global_robot_pos(:,i) = global_robot_pos(:,i-1)+robot_moves(:,i-1);
end
end