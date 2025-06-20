% TO STOP USE CTRL+C IN COMMAND WINDOW OF MATLAB

% Parameters
mapWidth = 10;  % meters
mapHeight = 10;
resolution = 10; % cells per meter

% Initialize Map
map.grid = zeros(mapHeight * resolution, mapWidth * resolution);  % occupancy (0 = free, 1 = occupied)
map.type = int(size(map.grid));  % 'tape', 'cube_red', 'cube_blue', 'obstacle'

% Coordinate Conversion Function
worldToGrid = @(x, y) deal(round(y * resolution), round(x * resolution));

% MQTT Setup
% IP of server
brokerAddress = "tcp://mqtt.ics.ele.tue.nl:1883";
% ID and password of each client
client73 = mqttclient(brokerAddress, ...
    Username="robot_73_2", Password="CrodokHu", ...
    ClientID="mapper_73");
subscribe(client73, "/pynqbridge/73/send");

client74 = mqttclient(brokerAddress, ...
    Username="robot_74_2", Password="AshVapt~", ...
    ClientID="mapper_74");
subscribe(client74, "/pynqbridge/74/send");

% Visualization setup
figure;
hImg = imagesc(1 - flipud(map.grid)); 
colormap(gray);
axis equal tight;
hold on;
title('Map of Detected Objects');
xlabel('X (grid cells)');
ylabel('Y (grid cells)');

% --- Legend Setup ---
legendItems = [];  % to hold handles
legendLabels = {}; % to hold labels

% Define mapping from types to display names and colors
legendDefinitions = {
    'tape',            [0 0 0],            'Cliff';
    'cube_red_small',  'r',                'Red Cube';
    'cube_green_small','g',                'Green Cube';
    'cube_blue_small', 'b',                'Blue Cube';
    'cube_white_small', [0.9, 0.9, 0.9],   'White Cube';
    'cube_black_small', [0.1, 0.1, 0.1],   'Black Cube';
    'mountain',        [0.6 0.6 0],        'Mountain';
};

% Create invisible patches to represent each type
for i = 1:size(legendDefinitions,1)
    h = patch(NaN, NaN, legendDefinitions{i,2});
    legendItems(end+1) = h; %#ok<SAGROW>
    legendLabels{end+1} = legendDefinitions{i,3}; %#ok<SAGROW>
end

% Display legend
legend(legendItems, legendLabels, 'Location', 'eastoutside');

sizeObjBig = 4;  offsetBig = sizeObjBig/2;
sizeObjSmall = 2; offsetSmall = sizeObjSmall/2;
sizeMountain = 8; offsetMountain = sizeMountain/2;

% Real-time loop for map update
disp('Listening');
while true
    % Read messages from both robots
    messages = [read(client73); read(client74)];

    % Process each incoming message
    % Process each incoming message
    for m = 1:height(messages)
        try
            rawPayload = messages{m, 2};  % Assumes payload is in column 2
            detection = jsondecode(rawPayload);  % Convert JSON string to struct

            x = detection.x;
            y = detection.y;
            objType = detection.type;

            [i, j] = worldToGrid(x, y);
            if i > 0 && j > 0 && i <= size(map.grid,1) && j <= size(map.grid,2)
                map.grid(i, j) = 1;
                map.type(i, j) = objType;
            end
        catch err
            %warning("Failed to parse message: %s", err.message);
        end
    end


    % Update the map
    cla;
    imagesc(1 - flipud(map.grid));
    colormap(gray);
    axis equal tight;
    hold on;

    for i = 1:size(map.grid, 1)
        for j = 1:size(map.grid, 2)
            t = map.type(i,j);
            if t ~= ""
                px = j;
                py = size(map.grid,1) - i + 1;  % flip for image coordinates

                % case for each type of detection (draws rectangles on top
                % of occupied space/pixel)
                switch t
                    case "tape"
                        rectangle('Position',[px - offsetSmall/2, py - offsetSmall/2, sizeObjSmall, sizeObjSmall], 'FaceColor', [0 0 0], 'EdgeColor', [0 0 0]);
                    case "cube_red_small"
                        rectangle('Position',[px - offsetSmall/2, py - offsetSmall/2, sizeObjSmall, sizeObjSmall], 'FaceColor', 'r', 'EdgeColor', 'r');
                    case "cube_red_big"
                        rectangle('Position',[px - offsetBig/2, py - offsetBig/2, sizeObjBig, sizeObjBig], 'FaceColor', 'r', 'EdgeColor', 'r');
                    case "cube_green_small"
                        rectangle('Position',[px - offsetSmall/2, py - offsetSmall/2, sizeObjSmall, sizeObjSmall], 'FaceColor', 'g', 'EdgeColor', 'g');
                    case "cube_green_big"
                        rectangle('Position',[px - offsetBig/2, py - offsetBig/2, sizeObjBig, sizeObjBig], 'FaceColor', 'g', 'EdgeColor', 'g');
                    case "cube_blue_small"
                        rectangle('Position',[px - offsetSmall/2, py - offsetSmall/2, sizeObjSmall, sizeObjSmall], 'FaceColor', 'b', 'EdgeColor', 'b');
                    case "cube_blue_big"
                        rectangle('Position',[px - offsetBig/2, py - offsetBig/2, sizeObjBig, sizeObjBig], 'FaceColor', 'b', 'EdgeColor', 'b');
                    case "cube_black_small"
                        rectangle('Position',[px - offsetSmall/2, py - offsetSmall/2, sizeObjSmall, sizeObjSmall], 'FaceColor', [0.1, 0.1, 0.1], 'EdgeColor', [0.1, 0.1, 0.1]);
                    case "cube_black_big"
                        rectangle('Position',[px - offsetBig/2, py - offsetBig/2, sizeObjBig, sizeObjBig], 'FaceColor', [0.1, 0.1, 0.1], 'EdgeColor', [0.1, 0.1, 0.1]);
                    case "cube_white_small"
                        rectangle('Position',[px - offsetSmall/2, py - offsetSmall/2, sizeObjSmall, sizeObjSmall], 'FaceColor', [0.9, 0.9, 0.9], 'EdgeColor', [0.9, 0.9, 0.9]);
                    case "cube_white_big"
                        rectangle('Position',[px - offsetBig/2, py - offsetBig/2, sizeObjBig, sizeObjBig], 'FaceColor', [0.9, 0.9, 0.9], 'EdgeColor', [0.9, 0.9, 0.9]);
                    case "mountain"
                        rectangle('Position',[px - offsetMountain/2, py - offsetMountain/2, sizeMountain, sizeMountain], 'FaceColor', [0.6 0.6 0], 'EdgeColor', [0.6 0.6 0]);
                    case "request"
                        [i, j] = worldToGrid(x, y);
                        if i > 0 && j > 0 && i <= size(map.grid,1) && j <= size(map.grid,2)
                            isOccupied = map.grid(i, j) == 1;
                            valueToSend = uint8(isOccupied);
                        else
                            valueToSend = uint8(0);
                        end

                        if strcmp(messages{m,1}, "/pynqbridge/73/send")
                            publish(client73, "/pynqbridge/73/receive", valueToSend);
                        elseif strcmp(messages{m,1}, "/pynqbridge/74/send")
                            publish(client74, "/pynqbridge/74/receive", valueToSend);
                        end
                end
            end
        end
    end

    drawnow;
    pause(0.2);  % Slight delay to avoid CPU overload
end

% JSON Format: {"x": 2.1, "y": 3.4, "type": "tape"}
