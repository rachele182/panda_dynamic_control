%% try to add forces
disp('Program started');
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
try
        pause(1);
        if (clientID>-1)
            disp('Connected to remote API server');
            vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait);
            if (res==vrep.simx_error_noerror)
                while(vrep.simxGetConnectionId(clientID)>-1)
                   force=[0 0 100];
                   packedData=vrep.simxPackInts(force);
                   vrep.simxWriteStringStream(clientID,'signal',packedData,vrep.simx_opmode_oneshot);
                   pause(5);
                end
            end
        else
            disp('Failed connecting to remote API server');
        end 

catch err
        vrep.simxFinish(clientID); % close the line if still open
        vrep.delete(); % call the destructor!
end
disp('Program ended');