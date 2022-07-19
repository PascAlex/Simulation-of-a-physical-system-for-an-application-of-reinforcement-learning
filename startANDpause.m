 disp('Program started');
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');
        
         %start simulation
        vrep.simxSynchronous(clientID,true);
%         [ret_start]=vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
   
      t = 1:50; %da qua comincio ad azionare i motori
           for i=1:length(t)   
              disp('continue')               
%             [ret_ping,pingTime]=vrep.simxGetPingTime(clientID);   
              [ret_start1]=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
              vrep.simxSynchronousTrigger(clientID);
%             [ret_ping,pingTime]=vrep.simxGetPingTime(clientID);
              disp('stop') 
              [ret_pause1]=vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot);
%              vrep.simxSynchronousTrigger(clientID);             
           end     
           
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);

        % Now close the connection to V-REP:    
        vrep.simxFinish(clientID);        
         end
    vrep.delete(); % call the destructor!
    
    disp('Program ended');
