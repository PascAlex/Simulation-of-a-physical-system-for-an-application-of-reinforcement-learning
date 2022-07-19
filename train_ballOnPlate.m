env = BallonPlate;
obsInfo = getObservationInfo(env);
numObservations = prod(obsInfo.Dimension);
actInfo = getActionInfo(env);
numActions = prod(actInfo.Dimension);

% specify the number of outputs for the hidden layers.
hiddenLayerSize = 100; 

observationPath = [
    imageInputLayer([numObservations 1 1],'Normalization','none','Name','observation')
    fullyConnectedLayer(hiddenLayerSize,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(hiddenLayerSize,'Name','fc2')
    additionLayer(2,'Name','add')
    reluLayer('Name','relu2')
    fullyConnectedLayer(hiddenLayerSize,'Name','fc3')
    reluLayer('Name','relu3')
    fullyConnectedLayer(1,'Name','fc4')];
actionPath = [
    imageInputLayer([numActions 1 1],'Normalization','none','Name','action')
    fullyConnectedLayer(hiddenLayerSize,'Name','fc5')];

% create the layerGraph
criticNetwork = layerGraph(observationPath);
criticNetwork = addLayers(criticNetwork,actionPath);

% connect actionPath to obervationPath
criticNetwork = connectLayers(criticNetwork,'fc5','add/in2');

criticOptions = rlRepresentationOptions('LearnRate',1e-03,'GradientThreshold',1);

critic = rlRepresentation(criticNetwork,obsInfo,actInfo,'Observation',{'observation'},'Action',{'action'},criticOptions);


actorNetwork = [
    imageInputLayer([numObservations 1 1],'Normalization','none','Name','observation')
    fullyConnectedLayer(hiddenLayerSize,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(hiddenLayerSize,'Name','fc2')
    reluLayer('Name','relu2')
    fullyConnectedLayer(hiddenLayerSize,'Name','fc3')
    reluLayer('Name','relu3')
    fullyConnectedLayer(numActions,'Name','fc4')
    tanhLayer('Name','tanh1')];

actorOptions = rlRepresentationOptions('LearnRate',1e-04,'GradientThreshold',1);

actor = rlRepresentation(actorNetwork,obsInfo,actInfo,'Observation',{'observation'},'Action',{'tanh1'},actorOptions);


agentOptions = rlDDPGAgentOptions(...
    'SampleTime',env.Ts  ,...
    'TargetSmoothFactor',1e-3,...
    'ExperienceBufferLength',1e6 ,...
    'DiscountFactor',0.99,...
    'MiniBatchSize',256);
agentOptions.NoiseOptions.Variance = 1e-1;
agentOptions.NoiseOptions.VarianceDecayRate = 1e-6;

agent = rlDDPGAgent(actor,critic,agentOptions);

trainOpts = rlTrainingOptions;

trainOpts.MaxEpisodes                = 1000;
trainOpts.MaxStepsPerEpisode         = 500;
trainOpts.StopTrainingCriteria       = "AverageReward";
trainOpts.StopTrainingValue          = 500;
trainOpts.ScoreAveragingWindowLength = 5;

trainOpts.SaveAgentCriteria  = "EpisodeReward";
trainOpts.SaveAgentValue     = 500;
trainOpts.SaveAgentDirectory = "savedAgents";

trainOpts.Verbose = true;
trainOpts.Plots =   "training-progress";

trainingInfo = train(agent,env,trainOpts);

