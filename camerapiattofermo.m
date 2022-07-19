
 a = -0.23
 b = 0.23;
 r = (b-a)*rand(1,1) + a;          
 g = (b-a)*rand(1,1) + a;
x0 = r;
y0 = g;
timestep = 0.01;
dimxPiatto = 0.5;
dimyPiatto = 0.5;


% function simpleSynchronousTest()
    disp('Program started');
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');
       
        [ret1,Joint1]=vrep.simxGetObjectHandle(clientID,'Revolute1',vrep.simx_opmode_blocking);
        [ret2,Joint2]=vrep.simxGetObjectHandle(clientID,'Revolute2',vrep.simx_opmode_blocking);
        [ret3,camera]=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking);
        [ret4,pallina]=vrep.simxGetObjectHandle(clientID,'Sphere_dyn',vrep.simx_opmode_blocking);
        [ret5,piatto]=vrep.simxGetObjectHandle(clientID,'Plate_dyn',vrep.simx_opmode_blocking);
        [ret6a,position]=vrep.simxGetJointPosition(clientID,Joint2,vrep.simx_opmode_streaming);
        [ret6b,position]=vrep.simxGetJointPosition(clientID,Joint1,vrep.simx_opmode_streaming);
         position_min = -pi/20;  
         position_max =  pi/20;
         position_zero = 0;
        v_ret=[ret1 ret2 ret3 ret4 ret5];         
         if (v_ret==0)
             disp('ok')
         else 
             disp('error program ended')
             vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
             vrep.simxFinish(clientID);
         end   
       
        [ret_pos_pallia,pos_pallina]=vrep.simxGetObjectPosition(clientID,pallina,piatto,vrep.simx_opmode_streaming);
        [ret_position_rand]=vrep.simxSetObjectPosition(clientID,pallina,piatto,[r,g,0.028],vrep.simx_opmode_oneshot);
       
        [ret11]=vrep.simxSetJointPosition(clientID,Joint2,0.01,vrep.simx_opmode_oneshot_wait); 
        
        
   
        %start simulation
        vrep.simxSynchronous(clientID,true);
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
        [ret7,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_streaming);
        vrep.simxSynchronousTrigger(clientID);
        t = 1:100; %da qua comincio ad azionare i motori
        torque=ones(50,1);%2*randn(50,1);
        f1=figure();
        
       
        for i=1:length(t)
            vrep.simxSynchronousTrigger(clientID);
            [ret_ping,pingTime]=vrep.simxGetPingTime(clientID);
            [ret9,position]=vrep.simxGetJointPosition(clientID,Joint2,vrep.simx_opmode_buffer); 
            fprintf(1,'********** cycle %d, actual position %f \n',i, position);
             
         
            
%             if (position < position_min)       %ricontrolla la posizione per poi confrontarla con
%                 disp('change1')                   % il vincolo e vedere se continuare o bloccarsi.
%                 break
%             end 
            [ret8,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_buffer);
            figure(f1)
            imshow(image);
            drawnow;
            
            [xnew,ynew,x_pnew,y_pnew] = RGBballdetection(image,x0,y0,timestep,dimxPiatto,dimyPiatto);
            vrep.simxPauseCommunication(clientID,1);
            [ret_velocity1]=vrep.simxSetJointTargetVelocity(clientID,Joint2,0,vrep.simx_opmode_oneshot);      
            [ret_force1]=vrep.simxSetJointForce(clientID,Joint2,7.12,vrep.simx_opmode_streaming);            
            vrep.simxPauseCommunication(clientID,0);
           
            x0=xnew;
            y0=ynew;
            
            
            
        end
        close(f1) 
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
        vrep.simxFinish(clientID);    
    else
        disp('Failed connecting to remote API server');
    end
    vrep.delete(); % call the destructor!
    
    disp('Program ended');

        
        
    