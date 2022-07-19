
   
% function incremento()
   x0 = 0;
   y0 = 0;
   timestep = 0.01;
   dimxPiatto = 0.5;
   dimyPiatto = 0.5;
   force = 1;
    disp('Program started');
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');
        %qua richiamo gli oggetti da v-rep
                [ret1,Joint1]=vrep.simxGetObjectHandle(clientID,'Revolute1',vrep.simx_opmode_blocking);
                [ret2,Joint2]=vrep.simxGetObjectHandle(clientID,'Revolute2',vrep.simx_opmode_blocking);
                [ret3,camera]=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking);
                [ret4,pallina]=vrep.simxGetObjectHandle(clientID,'Sphere_dyn',vrep.simx_opmode_blocking);
                [ret5,piatto]=vrep.simxGetObjectHandle(clientID,'Plate_dyn',vrep.simx_opmode_blocking);
                %qua richiamo le forze e le posizioni dei due giunti per la
                %prima volta
                [ret6a,position1]=vrep.simxGetJointPosition(clientID,Joint1,vrep.simx_opmode_blocking);
                [ret7a,position2]=vrep.simxGetJointPosition(clientID,Joint2,vrep.simx_opmode_blocking);
                [ret8a,force1]=vrep.simxGetJointForce(clientID,Joint1,vrep.simx_opmode_blocking);               
                [ret9a,force2]=vrep.simxGetJointForce(clientID,Joint2,vrep.simx_opmode_blocking);
                
                %qua imposto il sistema come preferisco averlo prima della
                %simulazione.
                [ret_velocity1]=vrep.simxSetJointTargetVelocity(clientID,Joint1,0,vrep.simx_opmode_oneshot_wait);                  
                [ret_velocity2]=vrep.simxSetJointTargetVelocity(clientID,Joint2,1000,vrep.simx_opmode_oneshot_wait);
                [ret_force1]=vrep.simxSetJointForce(clientID,Joint1,5,vrep.simx_opmode_oneshot_wait);  
                [ret_force2]=vrep.simxSetJointForce(clientID,Joint2,7.23,vrep.simx_opmode_oneshot_wait);           
                [ret10]=vrep.simxSetJointTargetPosition(clientID,Joint1,0,vrep.simx_opmode_oneshot_wait);
                [ret11]=vrep.simxSetJointTargetPosition(clientID,Joint2,0,vrep.simx_opmode_oneshot_wait);            
                [ret_camera,camera]=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking);
                [ret11]=vrep.simxSetJointPosition(clientID,Joint2,0,vrep.simx_opmode_oneshot_wait); 
                %start simulation
                vrep.simxSynchronous(clientID,true);
                vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
                [ret7,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_streaming);
                vrep.simxSynchronousTrigger(clientID);
                t = 1:50; %da qua comincio ad azionare i motori
                f1=figure();
       
        for i= 1:length(t)  
            vrep.simxSynchronousTrigger(clientID);
            [ret_ping,pingTime]=vrep.simxGetPingTime(clientID);
  
            [ret_vision2,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_buffer);
            figure(f1)
            imshow(image);
            drawnow;
            step=i;
            [xnew,ynew,x_pnew,y_pnew] = RGBballdetection(image,x0,y0,timestep,dimxPiatto,dimyPiatto);
            fprintf(1,'********** cycle %d, actual position %f \n',i, position2);

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
      
%  end
