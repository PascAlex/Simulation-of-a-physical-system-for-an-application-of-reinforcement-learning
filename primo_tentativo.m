classdef PrimoTentativo < rl.env.MATLABEnvironment
    %PRIMOTENTATIVO: Template for defining custom environment in MATLAB.    
    
    %% Properties (set properties' attributes accordingly)
    properties
        % Specify and initialize environment's necessary properties           

        %force 
        forcej2 = 5;
        forcej1 = 5;
        
        %velocity 
        velj1 = 1000;
        velj2 = 1000;
       

        RewardForNotFalling = 1
        
        % Penalty when the cart-pole fails to balance
        PenaltyForFalling = -10 
    end
    
    properties
        % Initialize system state [vb0x,vb0y,thj1_0,thj2_0,vj1_0,vj2_0,fj1_0,fj2_0]'
                vj1_0 = 0;     %velocità iniziale del joint1
                vj2_0 = 0;     %velocità iniziale del joint2
                fj1_0 = 5;     %forza inziale del joint1
                fj2_0 = 7.23;  %forza iniziale del joint2                                              
                thj1_0 = 0;    %angolo inziale in radianti del joint1
                thj2_0 = 0;    %angolo inziale in radianti del joint2
                vb0x = 0;      %velocità iniziale della pallina lungo x
                vb0y = 0;      %velocità iniziale della pallina lungo y
       
        State = zeros(8,1)
    end
    
    properties(Access = protected)
        % Initialize internal flag to indicate episode termination
        IsDone = false        
    end

    %% Necessary Methods
    methods              
        % Contructor method creates an instance of the environment
        % Change class name and constructor name accordingly
        function this = PrimoTentativo()
            % Initialize Observation settings
            disp('Program started');
            vrep=remApi('remoteApi');           
            vrep.simxFinish(-1);  
            clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5); 
            vrep.simxSynchronous(clientID,true);
                [ret1,Joint1]=vrep.simxGetObjectHandle(clientID,'Revolute1',vrep.simx_opmode_blocking);
                [ret2,Joint2]=vrep.simxGetObjectHandle(clientID,'Revolute2',vrep.simx_opmode_blocking);
                [ret3,camera]=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking);
                [ret4,pallina]=vrep.simxGetObjectHandle(clientID,'Sphere_dyn',vrep.simx_opmode_blocking);
                [ret5,piatto]=vrep.simxGetObjectHandle(clientID,'Plate_dyn',vrep.simx_opmode_blocking);
                %qua richiamo le forze e le posizioni dei due giunti per la
                %prima volta
                [ret6,position1]=vrep.simxGetJointPosition(clientID,Joint1,vrep.simx_opmode_blocking);
                [ret7,position2]=vrep.simxGetJointPosition(clientID,Joint2,vrep.simx_opmode_blocking);
                [ret8,force1]=vrep.simxGetJointForce(clientID,Joint1,vrep.simx_opmode_blocking);               
                [ret9,force2]=vrep.simxGetJointForce(clientID,Joint2,vrep.simx_opmode_blocking);
                [ret_pos_pallina,pos_pallina]=vrep.simxGetObjectPosition(clientID,pallina,piatto,vrep.simx_opmode_oneshot_wait);
                            

            
            ObservationInfo = rlNumericSpec([8 1]);
            ObservationInfo.Name = 'Ball on Plate States';
            ObservationInfo.Description = 'pos_ballx, pos_bally, vel_ballx, vel_bally, theta_j1, theta_j2, vel_j1, vel_j2';
            
            % Initialize Action settings
  
            ActionInfo = rlNumericSpec([2 1]);
            ActionInfo.Name = 'Ball on Plate Action';
           
            % The following line implements built-in functions of RL env
            this = this@rl.env.MATLABEnvironment(ObservationInfo,ActionInfo);
            this.var_vrep = vrep
            this.var_clientID = clientID
            this.var_joint1 = Joint1
            this.var_joint2 = Joint2
            this.var_pallina = pallina
            this.var_piatto = piatto
            this.var_camera = camera
         
            % Initialize property values and pre-compute necessary values

        end
        
        
        Observation = [xnew;ynew;x_pnew;y_pnew;thj1new;thj2new;vj1new;vj2new];
        
        
        % Apply system dynamics and simulates the environment with the 
        % given action for one step.
        function [Observation,Reward,IsDone,LoggedSignals] = step(this,Action)
            LoggedSignals = [];
         
            % Get action
            x0 = this.var_pb0x;
            y0 = this.var_pb0y;
            this.var_vrep.simxStartSimulation(this.var_clientID,this.var_vrep.simx_opmode_blocking);           
            this.var_vrep.simxSynchronousTrigger(this.var_clientID);
            [ret_vision1,resolution,image]=this.var_vrep.simxGetVisionSensorImage2(this.var_clientID,this.var_camera,0,this.var_vrep.simx_opmode_streaming);
            [ret12a,position1]=this.var_vrep.simxGetJointPosition(this.var_clientID,this.var_joint1,this.var_vrep.simx_opmode_oneshot);
            [ret13a,position2]=this.var_vrep.simxGetJointPosition(this.var_clientID,this.var_joint2,this.var_vrep.simx_opmode_oneshot);
%             this.var_vrep.simxSynchronousTrigger(this.var_clientID);
            [ret_ping,pingTime]=this.var_vrep.simxGetPingTime(this.var_clientID);
            [ret12,position1]=this.var_vrep.simxGetJointPosition(this.var_clientID,this.var_joint1,this.var_vrep.simx_opmode_buffer); 
            [ret13,position2]=this.var_vrep.simxGetJointPosition(this.var_clientID,this.var_joint2,this.var_vrep.simx_opmode_buffer);
            [ret_vision2,resolution,image]=this.var_vrep.simxGetVisionSensorImage2(this.var_clientID,this.var_camera,0,this.var_vrep.simx_opmode_buffer);
%             f1 = this.qualcosa;
            figure(this.qualcosa);    
            imshow(image);
                drawnow;
            step=i;
            [xnew,ynew,x_pnew,y_pnew] = RGBballdetection(image,x0,y0,this.timestep,this.dimxPiatto,this.dimyPiatto,step);
            fprintf(1,'********** cycle %d, actual position %f \n',i, position2);
            this.var_vrep.simxPauseCommunication(this.var_clientID,1);
            [ret_velocity3]=this.var_vrep.simxSetJointTargetVelocity(this.var_clientID,this.var_joint2,this.velj2,this.var_vrep.simx_opmode_streaming);      
            [ret_force3]=this.var_vrep.simxSetJointForce(this.var_clientID,this.var_joint2,this.forcej2,this.var_vrep.simx_opmode_oneshot);            
            this.var_vrep.simxPauseCommunication(this.var_clientID,0);
            fprintf(1,'++++++++++ cycle %d  actual force %f \n',i,5);             
                   x0=xnew;
                   y0=ynew;
            [ret_pause]=this.var_vrep.simxPauseSimulation(this.var_clientID,this.var_vrep.simx_opmode_oneshot_wait);      

            
            


           % Unpack state vector
            xnew = this.State(3);
            ynew = this.State(4);
            this.velj1 = this.State(7);
            thid.velj2 = this.State(8);
            
            
            % Cache to avoid recomputation
            

            % Apply motion equations            
            
            
            % Euler integration
             Observation = this.State + this.Ts.*[xnew;ynew;x_pnew;y_pnew;position1;position2;this.velj1;this.velj2];

            % Update system states
            this.State = Observation;
            
            % Check terminal condition
            pbx = Observation(1);
            pby = Observation(2);
            thj1 = Observation(5);
            thj2 = Observation(6);
            IsDone = pb == [0,0,0.028];%ricordati che devi metteree la condizione finale 
            this.IsDone = IsDone;
            
            % Get reward
            Reward = getReward(this);
            
            % (optional) use notifyEnvUpdated to signal that the 
            % environment has been updated (e.g. to update visualization)
            notifyEnvUpdated(this);
        end
        
        % Reset environment to initial state and output initial observation
        function InitialObservation = reset(this)

        end
    end
    %% Optional Methods (set methods' attributes accordingly)
    methods               
        % Helper methods to create the environment
        % Reward function
        function Reward = getReward(this)
            if ~this.IsDone
                Reward = this.RewardForNotFalling;
            else
                Reward = this.PenaltyForFalling;
            end          
        end
        
        
        
        % (optional) Properties validation through set methods
        function set.State(this,state)
            validateattributes(state,{'numeric'},{'finite','real','vector','numel',8},'','State');
            this.State = double(state(:));
            notifyEnvUpdated(this);
        end
       
       
        function set.Ts(this,val)
            validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','Ts');
            this.Ts = val;
        end
       
        function set.RewardForNotFalling(this,val)
            validateattributes(val,{'numeric'},{'real','finite','scalar'},'','RewardForNotFalling');
            this.RewardForNotFalling = val;
        end
        function set.PenaltyForFalling(this,val)
            validateattributes(val,{'numeric'},{'real','finite','scalar'},'','PenaltyForFalling');
            this.PenaltyForFalling = val;
        end
    end
%     function delete = delete(this)
%         close(f1)
%         this.var_vrep.simxStopSimulation(this.var_clientID,this.var_vrep.simx_opmode_blocking);
%         this.var_vrep.simxFinish(this.var_clientID);  
%         this.var_vrep.delete(this); % call the destructor!
%     end
    methods (Access = protected)
        % (optional) update visualization everytime the environment is updated 
        % (notifyEnvUpdated is called)
        function envUpdatedCallback(this)
        end
    end
end
