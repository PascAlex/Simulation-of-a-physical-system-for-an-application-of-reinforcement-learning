x0 = 0;
y0 = 0;
timestep = 0.01;
dimxPiatto = 0.5;
dimyPiatto = 0.5;

% function vincolipiatto/pallina()
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
        [ret6,position1]=vrep.simxGetJointPosition(clientID,Joint2,vrep.simx_opmode_streaming);
        [ret7,position2]=vrep.simxGetJointPosition(clientID,Joint1,vrep.simx_opmode_streaming);
         position_min1 = -0.05;
         position_min2 = -pi/4;
         position_max =  0.05;
         position_zero = 0;
         
         
            %start simulation
        vrep.simxSynchronous(clientID,true);
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
        [ret_vision1,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_streaming);
        vrep.simxSynchronousTrigger(clientID);
        t = 1:100;     
        torque=ones(100,1);%2*randn(50,1);
        f1=figure();
        
        %per 10 cycle il piatto deve stare fermo
        for i= 1:10
            vrep.simxSynchronousTrigger(clientID);
            [ret_ping1,pingTime]=vrep.simxGetPingTime(clientID);
            [ret8,position]=vrep.simxGetJointPosition(clientID,Joint2,vrep.simx_opmode_buffer); 
            fprintf(1,'********** cycle %d, actual position %f \n',i, position);
            [ret_vision2,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_buffer);
            figure(f1)
            imshow(image);
            drawnow;
            step=i;
            [xnew,ynew,x_pnew,y_pnew] = RGBballdetection(image,x0,y0,timestep,dimxPiatto,dimyPiatto,step);
            vrep.simxPauseCommunication(clientID,1);
            [ret_velocity1]=vrep.simxSetJointTargetVelocity(clientID,Joint2,1000*sign(torque(i)),vrep.simx_opmode_oneshot);      
            [ret_force1]=vrep.simxSetJointForce(clientID,Joint2,2.4*torque(i),vrep.simx_opmode_streaming);            
            vrep.simxPauseCommunication(clientID,0);
           
            x0=xnew;
            y0=ynew;
            if (abs(xnew)  >= 0.24 || abs(ynew)  >= 0.24)
               disp('stop1')     %qua si dovrebbe fermare se la pallina supera il bordo del piatto
            
               break   
            end 
        end 
        
        %qua il piatto comincia a scendere verso il basso ed è vincolato a
        %non superare una determinata posizione minima;
        for i=1:length(t)
            vrep.simxSynchronousTrigger(clientID);
            [ret_ping2,pingTime]=vrep.simxGetPingTime(clientID);
            [ret9,position]=vrep.simxGetJointPosition(clientID,Joint2,vrep.simx_opmode_buffer); 
            fprintf(1,'++++++++++ cycle %d, actual position %f \n',i, position);
                
            if (position < position_min1) 
            [ret_vision3,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_buffer);
            figure(f1)
            imshow(image);
            drawnow;
            step=i;
            [xnew,ynew,x_pnew,y_pnew] = RGBballdetection(image,x0,y0,timestep,dimxPiatto,dimyPiatto,step);
            vrep.simxPauseCommunication(clientID,1);
            [ret_velocity2]=vrep.simxSetJointTargetVelocity(clientID,Joint2,100*sign(torque(i)),vrep.simx_opmode_oneshot); %il piatto per la prima volta scende giu con una     
            [ret_force2]=vrep.simxSetJointForce(clientID,Joint2,0.5*torque(i),vrep.simx_opmode_streaming);                     %determinata velocità e una forza;
            vrep.simxPauseCommunication(clientID,0);
           
           
            if (abs(xnew)  >= 0.24 || abs(ynew)  >= 0.24)
               disp('stop2')     %qua si dovrebbe fermare sel la pallina supera il bordo del piatto
            
               vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
               vrep.simxFinish(clientID);       
            end  
            else                              %ricontrolla la posizione per poi confrontarla con
              disp('change1')                 % il vincolo e vedere se continuare o bloccarsi.
                for i= 1:length(t)
                    vrep.simxSynchronousTrigger(clientID);
                    [ret_ping3,pingTime]=vrep.simxGetPingTime(clientID);
                    [ret10,position]=vrep.simxGetJointPosition(clientID,Joint2,vrep.simx_opmode_buffer); 
                    fprintf(1,'%%%%%%%%%% cycle %d, actual position %f \n',i, position);
                    %superata la posizione minima il piatto torna a salire
                    %ma se supera lo zero lo faccio scendere di pocchissimo
                    %per poi bloccare il joint2
                    [ret_vision4,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_buffer);
                    figure(f1)
                    imshow(image);
                    drawnow;
                    step=i;
                    [xnew,ynew,x_pnew,y_pnew] = RGBballdetection(image,x0,y0,timestep,dimxPiatto,dimyPiatto,step);
                    vrep.simxPauseCommunication(clientID,1);
                    [ret_velocity3]=vrep.simxSetJointTargetVelocity(clientID,Joint2,100*sign(torque(i)),vrep.simx_opmode_oneshot); %il piatto per la prima volta scende giu con una     
                    [ret_force3]=vrep.simxSetJointForce(clientID,Joint2,3.9*torque(i),vrep.simx_opmode_streaming);                     %determinata velocità e una forza;
                    vrep.simxPauseCommunication(clientID,0);
                  
                            if (abs(xnew)  >= 0.24 || abs(ynew)  >= 0.24)
                             disp('stop3')     %qua si dovrebbe fermare sel la pallina supera il bordo del piatto
                             vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
                             vrep.simxFinish(clientID);    

                            end 
                              if (position > position_max)
                                  disp('change')
                                  for i= 1:length(t)
                                      vrep.simxSynchronousTrigger(clientID);
                                      [ret_ping4,pingTime]=vrep.simxGetPingTime(clientID);
                                      [ret11,position]=vrep.simxGetJointPosition(clientID,Joint2,vrep.simx_opmode_buffer); 
                                      fprintf(1,'$$$$$$$ cycle %d, actual position %f \n',i, position);
                                      [ret_vision5,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_buffer);
                                      figure(f1);
                                      imshow(image);
                                      drawnow;
                                      step=i;
                                      [xnew,ynew,x_pnew,y_pnew] = RGBballdetection(image,x0,y0,timestep,dimxPiatto,dimyPiatto,step);
                                      vrep.simxPauseCommunication(clientID,1);
                                      [ret_velocity4]=vrep.simxSetJointTargetVelocity(clientID,Joint2,-100*sign(torque(i)),vrep.simx_opmode_oneshot); %il piatto per la prima volta scende giu con una     
                                      [ret_force4]=vrep.simxSetJointForce(clientID,Joint2,0.3*torque(i),vrep.simx_opmode_streaming);                     %determinata velocità e una forza;
                                      vrep.simxPauseCommunication(clientID,0);
                                       
                                        if (abs(xnew)  >= 0.24 || abs(ynew)  >= 0.24)
                                           disp('stop4')     %qua si dovrebbe fermare sel la pallina supera il bordo del piatto
                                           vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
                                           vrep.simxFinish(clientID);    
                                           break    
                                        end 
                                      
                                          if (position < position_zero)
                                              disp('stop joint2')
                                              vrep.simxPauseCommunication(clientID,1);
                                              [ret_velocity4]=vrep.simxSetJointTargetVelocity(clientID,Joint2,0,vrep.simx_opmode_oneshot); %il piatto per la prima volta scende giu con una     
                                              [ret_force4]=vrep.simxSetJointForce(clientID,Joint2,0,vrep.simx_opmode_streaming);                     %determinata velocità e una forza;
                                              vrep.simxPauseCommunication(clientID,0);
                                              
                                           break
                                          end
                                  end
                              end
                    x0=xnew;
                    y0=ynew;
                    if (abs(xnew)  >= 0.24 || abs(ynew)  >= 0.24)
                       disp('stop5')     %qua si dovrebbe fermare sel la pallina supera il bordo del piatto

                       break    
                    end 
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
            
  
            
    
         
         
         
         
         
         
         
         
