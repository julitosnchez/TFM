classdef Cyl_modelling < handle
    % Description of the class
    % Properties:
        % - cloud of the hole
        % - Center detected
    
    properties (Access = private)
        cloud 
        center 
    end
    
    methods   
        function setcloud(obj,cloudIn)
            obj.cloud = cloudIn;
        end
        
        function originalCloud = getcloud(obj)
            originalCloud = obj.cloud;
        end
        
        function center = getcenter(obj)
            center = obj.center;
        end
        
        function setcenter(obj,center)
            obj.center = center;
        end
 
        function callbackCloud(objeto, msg,ptCloud) 
            % Reading the pointcloud from ROS and creating pointcloud
            % object for MATLAB
            ptCloud = pointCloud(readXYZ(ptCloud));            
            objeto.cloud = ptCloud;
            
            %objeto.applyAlgorithm
        end
        
        function callbackCenter(objeto, msg,center)           
            objeto.center = center;
        end
        
        function applyAlgorithm(obj)
            
            nube = obj.cloud;
            centers= obj.center;
            center1 = [centers.X;centers.Y;centers.Z]
           
            pcshow(nube, 'MarkerSize', 25)
              xlabel('X')
              ylabel('Y')
              zlabel('Z')
            hold on 


            NP=input('\ \n Introduce population number: \n');
            if isempty(NP)
                NP=100;   
                fprintf(1,'\n \t Population 100 by default. \n');
            end

            NP=round(NP);
            fprintf(1,'\n Population size: %i \n',NP);

            % Genetic algorithm upper iterations limit
            iter_max=input('\ \n Introduzce the maximum number of iterations: \ \');
            iter_max=round(iter_max);
            if isempty(iter_max)
                iter_max= 200;   
                fprintf(1,'\n \t The default iterations are %d \n',iter_max);
            end

            %--------------------------------------------------------------------------
            % Different options for the GL algorithm can be selected via keyboard:
            %  - DE Core Options:
            %    1) Random Mutation, with Thresholding and Discarding (Default). 
            %    2) Basic version, Random Mutation, without Thresholding, Discarding.
            %    3) Mutation from Best candidate, with Thresholding and Discarding.
            %    4) Random Mutation, with Thresholding and Discarding, NP is
            %    drastically reduced (tracking) after convergence.
            version_de=input('\ \n Introduce the DE version that you want to apply: \n 1) Random Mutation, with Thresholding and Discarding. \n 2) Basic version, Random Mutation, without Thresholding, Discarding. \n 3) Mutation from Best candidate, with Thresholding and Discarding. \n 4) Random Mutation, with Thresholding and Discarding, NP reduced (tracking) after convergence. \n');
            if isempty(version_de)
                version_de=1;   
                fprintf(1,'\n \t Option 1 by default. \n');
            end

            Points=nube.Count;

            if Points > 5

            fprintf(1,'\n Hole points considered: %i \n',Points);    
            xyz=nube.Location;
            xyz=reshape(xyz, [1 size(xyz,1)*size(xyz,2)]);
            xyz(isnan(xyz))=[];
            sz=size(xyz,2);

            for j=1:1:sz/3
                   x=xyz(j);
                   y=xyz(j+sz/3);
                   z=xyz(j+2*sz/3);
                   mat(ceil(j),:) = [x y z];
            end


            ptCloud=pointCloud(mat);

            %--------------------------------------------------------------------------
            %Initialization parameters:
            D=7;                        % DE Number of Chromosomes. 
            F=0.8;                     % Differential variations factor (mutation)
            CR=0.5;                     % Crossover constant

            % Population parameter boundaries,[x,y,z,u,v,w]: xyz center point, uvw axis vector
            %vector_min=[-1 -1 -1];
            vector_min=[-0.4 -0.4 0.6];
            %vector_max=[1 1 1 ];
            vector_max=[0.4 0.4 1];
            min=[center1(1)-0.05, center1(2)-0.05, ptCloud.ZLimits(1)-0.01, vector_min(1), vector_min(2), vector_min(3),0.0125]; %minimum an maximum
            max=[center1(1)+0.05, center1(2)+0.05, ptCloud.ZLimits(1)+0.01, vector_max(1), vector_max(2), vector_max(3),0.03];

            N_SIMULATIONS=1;
            Solution.best_estimate=zeros(N_SIMULATIONS,D);
            Solution.error=zeros(N_SIMULATIONS,1);

            for simul=1:N_SIMULATIONS

            % The initial population is randomly generated.
            population=initiate_pop(min,max,NP,D);

            %--------------------------------------------------------------------------

            % The robot motion simulation starts. In a single step, the robot tries to
            % locate itself. After convergence, robot motion is allowed until an 'f'
            % is introduced in dir_disp. In this case, the GL module ends its
            % execution.

            fprintf(1,'\n Simulation: %d/%d ',simul,N_SIMULATIONS);
            tic
            % The DE-based GL filter is called
            [bestmem,error,population]=alg_genet2(ptCloud,version_de,population,iter_max,max,min,NP,D,F,CR);
            toc

            vector=[bestmem(5) bestmem(6) bestmem(7)];
            center1=[bestmem(2) bestmem(3) bestmem(4)];
            radius=bestmem(8);

            avg=ev(ptCloud,vector,center1,radius);

            fprintf(1,'\n Estimated center by the GL filter (x, y, z) %f, %f, %f\n',bestmem(2),bestmem(3),bestmem(4));
            fprintf(1,'\n Estimated axis vector by the GL filter (u, v, w) %f, %f, %f\n',bestmem(5),bestmem(6),bestmem(7));
            u=[0 0 1];
            angle_deg = atan2d(norm(cross(u,vector)),dot(u,vector));
            fprintf(1,'\n Angle with sensor: %f deg \n',angle_deg);
            fprintf(1,'\n Radius %f cms\n',bestmem(8)*100);
            fprintf(1,'\n Error Radius: %f cms\n',abs(bestmem(8)*100-1.75));
            fprintf(1,'\n Avg. Distance from cloud to cylinder: %f cms \n',avg*100);

            % The best solution and the error are returned.
            Solution.best_estimate(simul,:)=bestmem(2:(D+1));
            Solution.error(simul)=error;

            end

            %--------------------------------------------------------------------------
            % Representation of results
            height=ptCloud.ZLimits(2)-ptCloud.ZLimits(1);
            params=[Solution.best_estimate(1),Solution.best_estimate(2),Solution.best_estimate(3),Solution.best_estimate(1)+height*Solution.best_estimate(4),Solution.best_estimate(2)+height*Solution.best_estimate(5),Solution.best_estimate(3)+height*Solution.best_estimate(6),Solution.best_estimate(7)];
            model = cylinderModel(params);
            %quiver3(Solution.best_estimate(2),Solution.best_estimate(3),Solution.best_estimate(4),Solution.best_estimate(5),Solution.best_estimate(6),Solution.best_estimate(7))
            if avg*100<3
                plot(model)
            else
            end    
            else 
            fprintf(1,'\n More points needed to analyze the hole');    
            end

        end
        
    end
end

