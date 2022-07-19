classdef BallonPlate < rl.env.MATLABEnvironment
    %SECONDOTENTATIVO: Template for defining custom environment in MATLAB.    
    
    %% Properties (set properties' attributes accordingly)
    properties
        % Specify and initialize environment's necessary properties    
        %variabili a b che servono al calcolo della posizione casuale iniziale della pallina
        %sono in pixel e dovrebberò andare da 0 a 128 però ho scartato i
        %primi 13 e gli ultimi 12 cosi la pallina si può posizionare poi in un quadrato di 0.4x0.4 e non 0.5x0.5        
        a = 13;
        b = 115;
        
        %valori posizione pallina goal
        xgoal = 0;
        ygoal = 0;
        
        %limite della pallina oltre al quale la simulazione andrebbe fermata   
        limit_ball = 0.24;
        
        %limite dei due motori che impedisce di portare il piatto in
        %una posizione estrema
        theta_limit = pi/8;
           
     
        %varibili vuote che poi mi aiuteranno a richiamare gli oggetti
        %in altre funzioni
        var_vrep = [];
        var_clientID = [];
        var_joint1 = [];
        var_joint2 = [];
        var_pallina = [];
        var_piatto = [];
        var_camera = [];
        var_pb0x = [];
        var_pb0y = [];
        var_figure = [];
        var_x0 = [];
        var_y0 = [];
        var_xnew = [];
        var_ynew = [];
        var_xpnew = [];
        var_ypnew = [];
        var_thj1new = [];
        var_thj2new = [];
        var_vj1new =[];
        var_vj2new =[];
        
        var_sensor = [];
        var_cyl2 = [];

        %costanti per la camera
        timestep = 0.01;
        dimxPiatto = 0.5;
        dimyPiatto = 0.5;
        
        %limiti delle due forze dei joint
        var_LowerLimit = [-100;-100];
        var_UpperLimit = [100;100];
        
        % Sample time
        Ts = 0.01       
        counter_Ts = 0; % contatore di passi
        
        % valore che serve per il calcolo del reward positivo
        espT = 0.001
        
       
        % Penalty when the cart-pole fails to balance
        PenaltyForFalling = -10 
    end
    
    properties
        % Initialize system state [x,dx,theta,dtheta]'
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
        IsFailure = false
    end

    %% Necessary Methods
    methods              
        % Contructor method creates an instance of the environment
        % Change class name and constructor name accordingly
        function this = BallonPlate()
            % Initialize Observation settings
            %qua creo il collegamento con vrep e chiamo gli oggetti che poi
            %andrò ad utilizzare
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
                
                %questi li aggiungo ora sperando che funzioni il reset
                [ret_forcesensor,sensor]=vrep.simxGetObjectHandle(clientID,'Force_sensor',vrep.simx_opmode_blocking);
                [ret_cyl2,cyl2]=vrep.simxGetObjectHandle(clientID,'Cylinder_link_dyn2',vrep.simx_opmode_blocking);
                
                
                %qua richiamo le forze e le posizioni dei due giunti per la
                %prima volta
                [ret6,position1]=vrep.simxGetJointPosition(clientID,Joint1,vrep.simx_opmode_streaming);
                [ret7,position2]=vrep.simxGetJointPosition(clientID,Joint2,vrep.simx_opmode_streaming);
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
            this.var_vrep     = vrep
            this.var_clientID = clientID
            this.var_joint1   = Joint1
            this.var_joint2   = Joint2
            this.var_pallina  = pallina
            this.var_piatto   = piatto
            this.var_camera   = camera
            this.var_sensor   = sensor 
            this.var_cyl2     = cyl2
            this.var_pb0x     = [];
            this.var_pb0y     = [];
            this.var_figure   = [];
            this.var_x0       = [];
            this.var_y0       = [];
            this.var_xnew     = [];
            this.var_ynew     = [];
            this.var_xpnew    = [];
            this.var_ypnew    = [];
            this.var_thj1new  = [];
            this.var_thj2new  = [];
            this.var_vj1new   = [];
            this.var_vj2new   = [];

           
            % Initialize property values and pre-compute necessary values
            updateActionInfo(this);
        end
        
        % Apply system dynamics and simulates the environment with the 
        % given action for one step.
        function [Observation,Reward,IsDone,LoggedSignals] = step(this,Action)
            LoggedSignals = [];
      
            % Get action 
            
            Force = getForce(this,Action);  
            
            this.counter_Ts = this.counter_Ts + 1;
            if (this.counter_Ts == 1)
            this.var_x0 = this.var_pb0x;
            this.var_y0 = this.var_pb0y;
            end
            this.var_vrep.simxStartSimulation(this.var_clientID,this.var_vrep.simx_opmode_blocking);    
            this.var_vrep.simxSynchronousTrigger(this.var_clientID);
            [ret_ping,pingTime]=this.var_vrep.simxGetPingTime(this.var_clientID);            
            [ret_vision1,resolution,image]=this.var_vrep.simxGetVisionSensorImage2(this.var_clientID,this.var_camera,0,this.var_vrep.simx_opmode_streaming);
            [ret12,thj1new]=this.var_vrep.simxGetJointPosition(this.var_clientID,this.var_joint1,this.var_vrep.simx_opmode_buffer); 
            [ret13,thj2new]=this.var_vrep.simxGetJointPosition(this.var_clientID,this.var_joint2,this.var_vrep.simx_opmode_buffer);
            [ret_vision2,resolution,image]=this.var_vrep.simxGetVisionSensorImage2(this.var_clientID,this.var_camera,0,this.var_vrep.simx_opmode_buffer);
            figure(this.var_figure);    
             imshow(image);
             drawnow;

            [xnew,ynew,x_pnew,y_pnew] = RGBballdetection(image,this.var_x0,this.var_y0,this.timestep,this.dimxPiatto,this.dimyPiatto);
            fprintf(1,'********** cycle %d, actual position %f \n',i, thj1new);
            fprintf(1,'********** cycle %d, actual position %f \n',i, thj2new)
            [ret_ping,pingTime]=this.var_vrep.simxGetPingTime(this.var_clientID);      
            this.var_vrep.simxPauseCommunication(this.var_clientID,1);
            [ret_velocity3]=this.var_vrep.simxSetJointTargetVelocity(this.var_clientID,this.var_joint2,1000*sign(Force(2)),this.var_vrep.simx_opmode_oneshot);      
            [ret_force3]=this.var_vrep.simxSetJointForce(this.var_clientID,this.var_joint2,abs(Force(2)),this.var_vrep.simx_opmode_oneshot);  
            [ret_velocity4]=this.var_vrep.simxSetJointTargetVelocity(this.var_clientID,this.var_joint1,1000*sign(Force(1)),this.var_vrep.simx_opmode_oneshot);      
            [ret_force4]=this.var_vrep.simxSetJointForce(this.var_clientID,this.var_joint1,abs(Force(1)),this.var_vrep.simx_opmode_oneshot); 
            this.var_vrep.simxPauseCommunication(this.var_clientID,0);
            fprintf(1,'++++++++++ cycle %d  actual forcej1 %f \n',i,Force(1));
            fprintf(1,'++++++++++ cycle %d  actual forcej2 %f \n',i,Force(2));
                   this.var_x0=xnew;
                   this.var_y0=ynew;
            
                   this.var_xnew = xnew;
                   this.var_ynew = ynew;
                   this.var_xpnew = x_pnew;
                   this.var_ypnew = y_pnew;
            
            %calcolo della vj1 e vj2 utilizzando sempre la posizione
            %angolare nuova meno quella vecchia che si trova nello stato
            %precedente fratto il tempo che per ora ho messo un valore
            %casuale non so se è quello giusto
            vj1new = (thj1new - this.State(5))/this.Ts   
            vj2new = (thj2new - this.State(6))/this.Ts
            this.var_vj1new = vj1new;
            this.var_vj2new = vj2new;
            this.var_thj1new = thj1new;
            this.var_thj2new = thj2new;
            Observation = [xnew;ynew;x_pnew;y_pnew;thj1new;thj2new;this.var_vj1new;this.var_vj2new];
               
            % Update system states
            this.State = Observation;
            
            %Check terminal condition
            
            %la pallina supera il range limite oppure i due joint superano
            %l'angolo limite       
            IsDone = abs(this.var_xnew)>this.limit_ball || abs(this.var_ynew)>this.limit_ball || abs(this.var_thj1new)>this.theta_limit || abs(this.var_thj2new)>this.theta_limit           
            this.IsFailure = IsDone;
            
            % Get reward
            Reward = getReward(this);
            
            % (optional) use notifyEnvUpdated to signal that the 
            % environment has been updated (e.g. to update visualization)
            notifyEnvUpdated(this);
        end
        
        % Reset environment to initial state and output initial observation
        function InitialObservation = reset(this)
                %qua tramite il reset metto il macchinario nelle condizioni
                %iniziali settandogli forza velocità e posizione casuale
                %della pallina sul piatto
                p0x = (this.b-this.a)*rand(1,1) + this.a; 
                p0y = (this.b-this.a)*rand(1,1) + this.a;
                p0x = 128-p0x;
                p0y = 128-p0y;
                pb0x = (p0x -64) * (0.5/128);
                pb0y = (p0y -64) * (0.5/128);
                this.var_pb0x = pb0x;
                this.var_pb0y = pb0y;
                pb_0 = [pb0x, pb0y, 0.0105];
                this.IsFailure = false;

                [ret_velocity1]=this.var_vrep.simxSetJointTargetVelocity(this.var_clientID ,this.var_joint1,this.vj1_0,this.var_vrep.simx_opmode_oneshot_wait);                  
                [ret_velocity2]=this.var_vrep.simxSetJointTargetVelocity(this.var_clientID,this.var_joint2,this.vj2_0,this.var_vrep.simx_opmode_oneshot_wait);
                [ret_force1]=this.var_vrep.simxSetJointForce(this.var_clientID,this.var_joint1,this.fj1_0,this.var_vrep.simx_opmode_oneshot_wait);  
                [ret_force2]=this.var_vrep.simxSetJointForce(this.var_clientID,this.var_joint2,this.fj2_0,this.var_vrep.simx_opmode_oneshot_wait);           
                [ret10]=this.var_vrep.simxSetJointPosition(this.var_clientID,this.var_joint1,this.thj1_0,this.var_vrep.simx_opmode_oneshot_wait);
                [ret11]=this.var_vrep.simxSetJointPosition(this.var_clientID,this.var_joint2,this.thj2_0,this.var_vrep.simx_opmode_oneshot_wait);  
                [ret_position_rand]=this.var_vrep.simxSetObjectPosition(this.var_clientID,this.var_pallina,this.var_piatto,pb_0,this.var_vrep.simx_opmode_oneshot_wait);
                
                
                [ret_position_piatto]=this.var_vrep.simxSetObjectPosition(this.var_clientID,this.var_piatto,-1,[-2.8375,1.15,0.6924],this.var_vrep.simx_opmode_oneshot);
                [ret_position_j1]=this.var_vrep.simxSetObjectPosition(this.var_clientID,this.var_joint1,-1,[-3.325,1.15,0.69],this.var_vrep.simx_opmode_oneshot_wait);
                [ret_position_j2]=this.var_vrep.simxSetObjectPosition(this.var_clientID,this.var_joint2,-1,[-3.1625,1.15,0.69],this.var_vrep.simx_opmode_oneshot_wait);
                [ret_position_sensor]=this.var_vrep.simxSetObjectPosition(this.var_clientID,this.var_sensor,-1,[-3.0875,1.15,0.69],this.var_vrep.simx_opmode_oneshot_wait);
                [ret_position_cyl2]=this.var_vrep.simxSetObjectPosition(this.var_clientID,this.var_cyl2,-1,[-3.1625,1.15,0.69],this.var_vrep.simx_opmode_oneshot_wait);
             
                
                
                [ret7,resolution,image]=this.var_vrep.simxGetVisionSensorImage2(this.var_clientID,this.var_camera,0,this.var_vrep.simx_opmode_streaming);
                if (isempty(this.var_figure) == 1)
                   disp('creo finestra')
                   this.var_figure=figure();
                   [ret_vision2,resolution,image]=this.var_vrep.simxGetVisionSensorImage2(this.var_clientID,this.var_camera,0,this.var_vrep.simx_opmode_buffer);
                   figure(this.var_figure)
                else
                    figure(this.var_figure)
                end    
                
            
           
            
            InitialObservation = [this.var_pb0x;this.var_pb0y;this.vb0x;this.vb0y;this.thj1_0;this.thj2_0;this.vj1_0;this.vj2_0];
            this.State = InitialObservation;
            
            % (optional) use notifyEnvUpdated to signal that the 
            % environment has been updated (e.g. to update visualization)
            notifyEnvUpdated(this);
        end
    end
    %% Optional Methods (set methods' attributes accordingly)
    methods               
        % Helper methods to create the environment
        function force = getForce(this,action)
            if ~and(all(action >= this.ActionInfo.LowerLimit), ...
                    all(action <= this.ActionInfo.UpperLimit))
                error('Action must be greater or equal  to (%g %g) and lower or equal to (%g %g).',...
                    this.ActionInfo.LowerLimit(1),this.ActionInfo.LowerLimit(2) ,...
                    this.ActionInfo.UpperLimit(1), this.ActionInfo.UpperLimit(2) );
            end
            force = action;           
        end
        
        function updateActionInfo(this)
            this.ActionInfo.LowerLimit = this.var_LowerLimit;
            this.ActionInfo.UpperLimit = this.var_UpperLimit;
        end
           
        % Reward function       
        function Reward = getReward(this)
            if ~this.IsFailure
                X = [(this.xgoal - this.var_xnew);(this.ygoal - this.var_ynew)]; % distanza tra il goal (0,0) e la posizione effettiva della pallina
                Rew = norm(X,2); %calcolo della distanza tra due punti usando la norma due
                Reward = 1/(Rew + this.espT); % il valore del reward è inversamente proporzionale alla distanza tra la pallina e il centro del piatto
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
      
        
        function set.PenaltyForFalling(this,val)
            validateattributes(val,{'numeric'},{'real','finite','scalar'},'','PenaltyForFalling');
            this.PenaltyForFalling = val;
        end
    end
    
    methods (Access = protected)
        % (optional) update visualization everytime the environment is updated 
        % (notifyEnvUpdated is called)
        function envUpdatedCallback(this)
        end
    end
    methods
     function finish = delete(this)
         close(this.var_figure)
         this.var_vrep.simxStopSimulation(this.var_clientID,this.var_vrep.simx_opmode_blocking);
         this.var_vrep.simxFinish(this.var_clientID);  
         this.var_vrep.delete;
         disp('program ended')         
     end
    end
end
