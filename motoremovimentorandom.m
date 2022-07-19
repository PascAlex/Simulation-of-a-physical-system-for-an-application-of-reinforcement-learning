%per ora questo programma in maniera randomica sceglie di azionare uno dei
%due joint e sempre in maniera randomica sceglie in quale direzione
%azionarlo, (ci sono pure i vincoli sul piatto e la telecamera che
%restituisce le coordinate del baricentro della pallina.

function direzione=motore(su_giu,dx_sx)
   x0 = 0;
   y0 = 0;
   timestep = 0.01;
   dimxPiatto = 0.5;
   dimyPiatto = 0.5;
   su = 1;
   giu = -1;
   fermo = 0;
   dx = 1;
   sx =-1;
   v1 = [su;giu;fermo];
   su_giu = v1(randi(3,1))
   v2 = [dx;sx;fermo];
   dx_sx = v2(randi(3,1))
   v3 = [su_giu;dx_sx];   
   direzione = v3(randi(2,1))
   position_set1 = pi/16;
   position_set2 = 0.09;

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
        [ret_camera,camera]=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking);
        
        %start simulation
        vrep.simxSynchronous(clientID,true);
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
        [ret7,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_streaming);
        vrep.simxSynchronousTrigger(clientID);
        t = 1:50; %da qua comincio ad azionare i motori
        torque=ones(50,1);%2*randn(50,1);
        f1=figure();
        
        for i= 1:length(t)
            vrep.simxSynchronousTrigger(clientID);
            [ret_ping,pingTime]=vrep.simxGetPingTime(clientID);
            [ret9a,position]=vrep.simxGetJointPosition(clientID,Joint1,vrep.simx_opmode_buffer); 
            [ret9b,position]=vrep.simxGetJointPosition(clientID,Joint2,vrep.simx_opmode_buffer);
            [ret_vision2,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_buffer);
            figure(f1)
            imshow(image);
            drawnow;
            step=i;
            [xnew,ynew,x_pnew,y_pnew] = RGBballdetection(image,x0,y0,timestep,dimxPiatto,dimyPiatto,step);
       
            if (direzione == su_giu)
                fprintf(1,'********** cycle %d, actual position %f \n',i, position);
                  vrep.simxPauseCommunication(clientID,1);
                  [ret_velocity1]=vrep.simxSetJointTargetVelocity(clientID,Joint2,1000*su_giu,vrep.simx_opmode_oneshot);      
                  [ret_force1]=vrep.simxSetJointForce(clientID,Joint2,5,vrep.simx_opmode_streaming);            
                  vrep.simxPauseCommunication(clientID,0);
                   x0=xnew;
                   y0=ynew;
                  if (abs(position) > position_set1)
                      disp('stop1')
                      close(f1)
                      vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
                      vrep.simxFinish(clientID);   
                  end 
            elseif (direzione == dx_sx)
                fprintf(1,'++++++++++ cycle %d, actual position %f \n',i, position);
                  vrep.simxPauseCommunication(clientID,1);
                  [ret_velocity2]=vrep.simxSetJointTargetVelocity(clientID,Joint1,1000*dx_sx,vrep.simx_opmode_oneshot);      
                  [ret_force2]=vrep.simxSetJointForce(clientID,Joint1,5,vrep.simx_opmode_streaming);            
                  vrep.simxPauseCommunication(clientID,0);
                   x0=xnew;
                   y0=ynew;
                  if (abs(position) > position_set2)
                      disp('stop2')
                      close(f1)
                      vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
                      vrep.simxFinish(clientID);   
                  end 
            end
        end 
        close(f1)
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
        vrep.simxFinish(clientID);    
    else
        disp('Failed connecting to remote API server');
    end
    vrep.delete(); % call the destructor!
    
    disp('Program ended');     
        
    

    
        
        
 end