%quando il ciclo è dispari si ferma la simulazione quando invece il ciclo è
%pari la simulazione riprende;

%function Jointdueversolalto()
    disp('Program started');
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');
       
        [ret1,Joint1]=vrep.simxGetObjectHandle(clientID,'Revolute1',vrep.simx_opmode_blocking);
        [ret2,Joint2]=vrep.simxGetObjectHandle(clientID,'Revolute2',vrep.simx_opmode_blocking);       
        [ret3,pallina]=vrep.simxGetObjectHandle(clientID,'Sphere_dyn',vrep.simx_opmode_blocking);
        [ret4,piatto]=vrep.simxGetObjectHandle(clientID,'Plate_dyn',vrep.simx_opmode_blocking);         
        [ret6,position_plate]=vrep.simxGetObjectPosition(clientID,piatto,-1,vrep.simx_opmode_streaming);
        [ret5,position]=vrep.simxGetJointPosition(clientID,Joint2,vrep.simx_opmode_streaming); 
        position_max = pi/8;
      
        
        v_ret = [ret1 ret2 ret3 ret4 ]          
         if (v_ret==0)
             disp('ok')
         else 
             disp('error program ended')
             vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
             vrep.simxFinish(clientID);
         end   
           
        %start simulation
        vrep.simxSynchronous(clientID,true);
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
        
        t = 1:50; %da qua comincio ad azionare i motori
           for i=1:length(t)
            vrep.simxSynchronousTrigger(clientID);
            [ret_ping,pingTime]=vrep.simxGetPingTime(clientID);
            [ret8,position]=vrep.simxGetJointPosition(clientID,Joint2,vrep.simx_opmode_buffer); 
            fprintf(1,'********** cycle %d, actual position %f \n',i, position);

            vrep.simxPauseCommunication(clientID,1);       
            [ret_velocity1]=vrep.simxSetJointTargetVelocity(clientID,Joint2,1000,vrep.simx_opmode_oneshot);
            [ret_force1]=vrep.simxSetJointForce(clientID,Joint2,5,vrep.simx_opmode_streaming);            
                                                 
            vrep.simxPauseCommunication(clientID,0);
           
            

             
           end      

        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);

        % Now close the connection to V-REP:    
        vrep.simxFinish(clientID);        
        
       else
        disp('Failed connecting to remote API server');
    end
    vrep.delete(); % call the destructor!
    
    disp('Program ended');
%end
       
