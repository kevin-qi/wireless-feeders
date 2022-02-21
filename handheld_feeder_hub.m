clear all

global hello
hello = "dfsd"
arduinoPort = "COM4";
disp(serialportlist("available"));
assert(ismember(arduinoPort,serialportlist("available")), "Serial port not detected, please check correct serialport!")

arduinoObj = serialport(arduinoPort, 115200);
arduinoObj.UserData = struct("Data","","Count",1);
arduinoObj.UserData.dataQueue = {};
configureTerminator(arduinoObj, "CR/LF");

flush(arduinoObj);

todayDate = datetime([datetime('today')],'Format','yyMMdd');

fileID = fopen('logs.txt', 'a+');
fprintf(fileID, "Date: %s\n", todayDate);
configureCallback(arduinoObj, "terminator", @readArduinoLogs);


function readArduinoLogs(src, ~)
    % Read the ASCII data from the serialport object.
    data = readline(src);

    fileID = fopen('logs.txt', 'a+');
    fprintf(fileID, "%s\n",data);
    fclose(fileID);

    src.UserData.Count = src.UserData.Count + 1;
    src.UserData.Data = append(src.UserData.Data, data, '\n');
    src.UserData.dataQueue{end+1} = data;
    % Convert the string data to numeric type and save it in the UserData
    % property of the serialport object.

    % If 1001 data points have been collected from the Arduino, switch off the
    % callbacks and plot the data.
    if(src.UserData.Count > 5)
        configureCallback(src, "off");
       
    end 
end