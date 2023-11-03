classdef OmronTM12 < RobotBaseClass
    %% Omron TM12
    % This class is based on the Omron TM12. 
    % URL: https://assets.omron.eu/downloads/datasheet/en/v3/i837_collaborative_robots_datasheet_en.pdf
    % 
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'OmronTM12';
    end

    methods (Access = public) 
    %% Constructor 
        function self = OmronTM12(baseTr)
			self.CreateModel();
             if nargin < 1			
				baseTr = eye(4);				
             end	
				self.model.base = self.model.base.T * baseTr;
               
            self.homeQ = zeros(1,self.model.n);
            %self.PlotAndColourRobot();
  
            self.plot(q0,'workspace',workspace,'scale',scale);
        end

%% CreateModel
        function CreateModel(self)       
            link(1) = Link('d',0,	'a',0,  	'alpha',-pi/2,    'offset',0, 	'qlim',[deg2rad(-270),deg2rad(270)]);
            link(2) = Link('d',0.180,	'a',0.6361,	'alpha',0,     'offset',0, 	'qlim',[deg2rad(-180),deg2rad(180)]);
            link(3) = Link('d',-0.1297, 	'a',0.5579,	'alpha',0,    'offset',0, 	'qlim',[deg2rad(-166),deg2rad(166)]);
            link(4) = Link('d',0.106, 	'a',0,'alpha',pi/2,     'offset',0, 	'qlim',[deg2rad(-180),deg2rad(180)]);
            link(5) = Link('d',0.106,	'a',0,  'alpha',-pi/2,    'offset',0, 	'qlim',[deg2rad(-180),deg2rad(180)]);
            link(6) = Link('d',0.11315,       'a',0,  'alpha',0,     'offset',0, 	'qlim',[deg2rad(-270),deg2rad(270)]);
%add gripper???
            self.model = SerialLink(link,'name',self.name);

            workspace = [-2 2 -2 2 -2 2];
            scale = 0.6;
            q0 = zeros(size(self.links));

        end     
   end
 
end