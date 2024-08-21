function [status, output] = colconBuild(packages)
    % Construct the build command
    if nargin < 1 || isempty(packages)
        buildCmd = 'colcon build';
    else
        buildCmd = ['colcon build --packages-select ' strjoin(packages, ' ')];
    end

    % Construct the full command
    fullCommand = sprintf('./run_ros2_command.sh "%s"', buildCmd);

    % Run the command
    disp('Running colcon build...');
    [status, output] = system(fullCommand);

    % Display the output
    disp(output);

    % Check for errors
    if status ~= 0
        warning('Error running colcon build');
    else
        disp('colcon build completed successfully');
    end
end

function [status, output] = runROS2Command(command, background)
    if nargin < 2
        background = false;
    end
    
    if background
        fullCommand = sprintf('./run_ros2_command.sh bg "%s"', command);
    else
        fullCommand = sprintf('./run_ros2_command.sh "%s"', command);
    end
    
    [status, output] = system(fullCommand);
    disp(output);
    
    if status ~= 0
        warning('Error running ROS 2 command');
    elseif background
        disp('ROS 2 command started in background');
    end
end

function [status, output] = launchROS2(package, launchFile, args, background)
    if nargin < 3
        args = '';
    end
    if nargin < 4
        background = false;
    end
    command = sprintf('ros2 launch %s %s %s', package, launchFile, args);
    [status, output] = runROS2Command(command, background);
end

function jointStates = getJointStates()
    % Use the ros2 command to get the joint states
    [status, cmdout] = runROS2Command('ros2 topic echo --once /joint_states');
    
    if status == 0
        % Parse the output to extract joint positions
        lines = strsplit(cmdout, '\n');
        positionLineIndex = find(contains(lines, 'position:'));
        
        if ~isempty(positionLineIndex)
            % The 'position:' line is expected to be followed by the actual values
            positionLine = strtrim(lines{positionLineIndex + 1});
            
            % Ensure the line is formatted as expected and extract the values
            if startsWith(positionLine, '-')
                % Extract numbers from the line
                positionStr = extractBetween(positionLine, '[', ']');
                if ~isempty(positionStr)
                    % Convert string to array of numbers
                    jointStates = str2num(positionStr{1});
                else
                    warning('Position data found, but could not extract values.');
                    jointStates = [];
                end
            else
                warning('Unexpected format in position data line: %s', positionLine);
                jointStates = [];
            end
        else
            warning('No position data found in the joint states message.');
            jointStates = [];
        end
    else
        warning('Failed to get joint states. Error: %s', cmdout);
        jointStates = [];
    end
end


%% Build all packages
[buildStatus, buildOutput] = colconBuild();

%% Launch Gazebo and Moveit

[launchStatus, launchOutput] = launchROS2('ur_simulation_gazebo', 'ur_sim_moveit.launch.py', '', true);
pause(10);

%% Run the Demo
[cmdStatus, cmdOutput] = runROS2Command('ros2 run ur3e_control_package initials_demo')
%% Run Joint Space Command
[cmdStatus, cmdOutput] = runROS2Command("ros2 run ur3e_control_package move_command --ros-args -p initial_move:=True -p initial_joint_pos:='[-1.9991989999999866, -1.835606000000002, -2.0968710000000046, -2.349238999999997, -0.4251269999999927, -0.0012429999999703512]'");
%% Run Task Space Command

waypoints = '[[[0.30, 0.14, 0.25], [0.5, 0.5, 0.5, 0.5]]]';
command = sprintf('/path/to/your_script.sh fg "%s"', waypoints);


[cmdStatus, cmdOutput] = runROS2Command(command)

%% Run Task Space Command in the Robot's End Effector Frame
[cmdStatus, cmdOutput] = runROS2Command("ros2 run ur3e_control_package move_command --ros-args -p initial_move:=False -p waypoints:="'[[[0.0,0.1,0.0], [0.0,0.0,0.0,1.0]]]'" -p frame:="ef" -p velocity:=0.02 -p dt:=0.2")


%%
getJointStates();