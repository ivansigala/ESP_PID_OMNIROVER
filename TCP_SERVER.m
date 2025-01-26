% Server setup
serverIP = "Your networks IP"; 
serverPort = 1234; 
tcpServer = tcpserver(serverIP, serverPort);
disp("Waiting for ESP32 connection...");

while tcpServer.Connected == 0
    pause(0.1);
end
disp('ESP32 connected!');

% Create a queue for user inputs
userInputQueue = java.util.LinkedList();
userInputQueue.add("00000000");

%Matrix derived from the ecuation sistem of the robot
J = [1, -1, -26.1; 1, 1, 26.1; 1, 1, -26.1; 1, -1, 26.1 ];
V = zeros(3,1);
C = zeros(4,1);

% Setup timer for data processing and plotting
dataTimer = timer('ExecutionMode', 'fixedRate', ...
                  'Period', 0.2, ... % Update every 0.1 seconds
                  'TimerFcn', @(~, ~) processData(tcpServer, userInputQueue),...
                  'BusyMode', 'drop'); % Drop queued executions if busy);

start(dataTimer);

% Main thread for user input
while true
    while true
        V(1,1) = str2double(input('Ingresa Vx: ', 's'));
        V(2,1) = str2double(input('Ingresa Vy: ', 's'));
        V(3,1) = str2double(input('Ingresa T : ', 's'));
        reply  = input('Want to continue?: ', 's');
        if reply == ""
            stop(dataTimer);
            delete(dataTimer);
            disp("Bye . . . . ");
            break;
        elseif reply(1) == '-'
            stop(dataTimer);
            delete(dataTimer);
            disp("Bye . . . . ");
            break;
        else
            C = 100* J * V;
            if(C(1,1) > 255 || C(2,1) > 255  || C(3,1) > 255  || C(4,1) > 255  )
                disp('Impossible task exceeds maximun speed');
            else
                break;
            end
        end
    end
    if reply(1) == '-'
        break;
    end
    C = fix(C);
    reply = vectorToHexString(C);
    % Add the new reply to the queue
    disp(reply);
    userInputQueue.add(reply);
end

function processData(tcpServer, userInputQueue)
    persistent userReply timeData motor1 motor2 motor3 motor4 dataIndex startTime;
    persistent plotHandle1 plotHandle2 plotHandle3 plotHandle4;

    % Initialize persistent variables on first execution
    if isempty(timeData)
        timeData = zeros(1, 1000);
        motor1 = zeros(1, 1000);
        motor2 = zeros(1, 1000);
        motor3 = zeros(1, 1000);
        motor4 = zeros(1, 1000);
        dataIndex = 1;
        startTime = datetime('now');
        
        % Initial plot setup
        figure(1);
        hold on; % Ensure multiple plots can be added
        plotHandle1 = plot(timeData, motor1, '-r', 'DisplayName', 'Motor 1');
        plotHandle2 = plot(timeData, motor2, '-b', 'DisplayName', 'Motor 2');
        plotHandle3 = plot(timeData, motor3, '-g', 'DisplayName', 'Motor 3');
        plotHandle4 = plot(timeData, motor4, '-y', 'DisplayName', 'Motor 4');
        xlabel('Time (s)');
        ylabel('Motor Values');
        title('Real-Time Motor Data from ESP32');
        legend('show');
        grid on;
        hold off;
    end

    if tcpServer.NumBytesAvailable >= 8
        % Read 8 bytes from the TCP server
        dataReceived = read(tcpServer, 8, "uint8");
        
        % Decode data using hexStringToVector function
        res = hexStringToVector(char(dataReceived));

        % Update motor data
        motor1(dataIndex) = res(1);
        motor2(dataIndex) = res(2);
        motor3(dataIndex) = res(3);
        motor4(dataIndex) = res(4);

        % Calculate elapsed time
        currentTime = seconds(datetime('now') - startTime);
        timeData(dataIndex) = currentTime;

        % Update rolling window for plot
        windowSize = 200;
        startIdx = max(1, dataIndex - windowSize + 1);

        % Update plots
        set(plotHandle1, 'XData', timeData(startIdx:dataIndex), 'YData', motor1(startIdx:dataIndex));
        set(plotHandle2, 'XData', timeData(startIdx:dataIndex), 'YData', motor2(startIdx:dataIndex));
        set(plotHandle3, 'XData', timeData(startIdx:dataIndex), 'YData', motor3(startIdx:dataIndex));
        set(plotHandle4, 'XData', timeData(startIdx:dataIndex), 'YData', motor4(startIdx:dataIndex));
        
        drawnow limitrate; % Efficient plot update
        
        % Increment data index
        dataIndex = dataIndex + 1;
    end

    % Send user input to the ESP32 if available
    if ~userInputQueue.isEmpty()
        userReply = userInputQueue.poll(); % Get latest user command
    end
    write(tcpServer, uint8(char(userReply)));
end


function reply = vectorToHexString(C)
    % Validate input
    if length(C) ~= 4
        error('Input must be a 4-element vector.');
    end

    reply = blanks(8); % Preallocate an 8-character string
    temp = blanks(2);

    for i = 1:4
        val = C(i);
        % Logic to convert the value to two hex characters
        if val <= 15 && val >= 0
            reply(2*i-1) = '0';
            reply(2*i) = dec2hex(val);
        elseif val < 0 && val >= -15
            reply(2*i-1) = 'G';
            if val < 0 && val >= -9
                reply(2*i) = char(dec2hex(-val) + 23);
            else
                reply(2*i) = char(dec2hex(-val) + 16);
            end
        elseif val < -15
            hexStr = dec2hex(-val);
            reply(2*i-1:2*i) = hexStr;
            if val < -159
                reply(2*i-1) = char(reply(2*i-1) + 16);
            else
                reply(2*i-1) = char(reply(2*i-1) + 23);
            end
            if hexStr(2) >= 65 && hexStr(2) <= 70
                reply(2*i) = char(reply(2*i) + 16);
            else
                reply(2*i) = char(reply(2*i) + 23);
            end
        else
            reply(2*i-1:2*i) = dec2hex(val);
        end
    end
    temp(1:2) = reply(1:2);
    reply(1:2) = reply(3:4);
    reply(3:4) = temp(1:2);
end

function C = hexStringToVector(hexStr)
    % Validate input
    if length(hexStr) ~= 8
        error('Input must be an 8-character string.');
    end

    C = zeros(1, 4); % Preallocate a 4-element vector

    for i = 1:4
        % Extract the pair of hex characters
        hexPair = hexStr(2*i-1:2*i);

        % Convert to integer based on encoding logic
        if hexPair(1) == '0'
            % Case 1: Small positive value
            val = hex2dec(hexPair(2));
        else
            % Case 2: Negative value
            if hexPair(1) > 70
                if hexPair(1) > 70 && hexPair(1) < 80
                    val = hex2dec(hexPair(1) - 23) * (-16);
                else 
                    val = hex2dec(hexPair(1) - 16) * (-16);
                end
                if hexPair(2) > 70 && hexPair(2) < 80
                    val = val + hex2dec(hexPair(2) - 23) * (-1);
                else 
                    val = val + hex2dec(hexPair(2) - 16) * (-1);
                end
            else
                val = hex2dec(hexPair(1:2)); 
            end
        end

        % Store the result
        C(i) = val;
    end
    temp = C(1);
    C(1) = C(2);
    C(2) = temp;
end
