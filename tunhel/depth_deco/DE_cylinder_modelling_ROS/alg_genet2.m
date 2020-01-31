function[bestmem,error,population]=alg_genet2(ptCloud,version_de,population,iter_max,max,min,NP,D,F,CR)
%--------------------------------------------------------------------------
%   Function: alg_genet
%--------------------------------------------------------------------------
% -> Description: DE-based filter that is executed to obtain the cylinder model from the holepoint cloud. The core engine is the DE algorithm and the
% cost value is generated by fitness.m.
%--------------------------------------------------------------------------
% -> Inputs:
%       -version_de: Type of DE version, chosen via keyboard.
%       -population: population set. Matrix with dimensions NP*(D+1) where
%       each row contains a candidate with an associated cost value (first
%       column).
%       -iter_max: Maximum number of iterations.
%       -max: Vector of 6 elements that corresponds to the maximum values
%       of the coordinates, both center and vector
%       -mapmin: Vector of 6 elements that corresponds to the minimum values
%       of the coordinates, both center and vector
%       -NP: Population size.
%       -D: Number of chromosomes (DE parameter).
%       -F: Differential variations factor. Mutation coefficient (DE).
%       -CR: Crossover constant (DE).

% -> Outputs:
%       -bestmem: Vector of D+1 elements containing the solution of the
%       global localization filter (robot's location) and its cost value in
%       the first element.
%       -error: Cost value of the best candidate.
%       -population: Population set after convergence or maximum iter.
%--------------------------------------------------------------------------

%Initilization of variables
trial=zeros(1,D);
bestmem=zeros(1,D+1);
pob_aux=zeros(NP,D+1);
count_a=0; % Counters to check convergence
count_b=0;
count_c=0;
count=1;
imp_count=1;
error_max=100000;
error=10000;
dif_errores=100000;
%--------------------------------------------------------------------------   

%The cost value is computed for the initial population
for i=1:NP
       population(i,1)=fitness2(ptCloud,population(i,:)); 
end
%--------------------------------------------------------------------------
while (count<=iter_max)%&&(error~=error_max)%&&(error_max>error+10)%3*NP/2) %&&(error>NS/3))%%&&(count_c<30)&&((count_a<100)||(count_b<100)))
     %( (count<=iter_max)&&(error>150)&&(count_c<30)&&((count_a<100)||(count_b<100)))
    for i=1:NP

        % Three random vectors are selected for mutation.
        a=random('unid',NP);
        while((a==i)||(a==0))
            a=random('unid',NP);
        end
        b=random('unid',NP);
        while((b==i)||(b==a)||(b==0))
            b=random('unid',NP);
        end
        c=random('unid',NP);
        while((c==i)||(c==a)||(c==b)||(c==0))
            c=random('unid',NP);
        end
        
        % Mutation and crossover. Different mutation options
        for k=2:(D+1)
            cross_rand=random('unid',100);
            if(cross_rand < (100*CR))
                if (version_de==3)
                    if population(a,1)<population(b,1)
                        if population(a,1)<population(c,1)
                            trial(1,(k-1))=population(a,k)+ F*(population(b,k)-population(c,k));
                        else
                            trial(1,(k-1))=population(c,k)+ F*(population(a,k)-population(b,k));
                        end
                    else if population(b,1)<population(c,1)
                            trial(1,(k-1))=population(b,k)+ F*(population(c,k)-population(a,k));
                        else
                            trial(1,(k-1))=population(c,k)+ F*(population(a,k)-population(b,k));
                        end
                    end
                else
                    trial(1,(k-1))=population(c,k)+ F*(population(a,k)-population(b,k));
                end
            else trial(1,(k-1))=population(i,k);
            end
            % We check that we are still in the map limits.
            if (trial(1,(k-1))<min(k-1))
                trial(1,(k-1))=min(k-1);
            end
            if (trial(1,(k-1))>max(k-1))
                trial(1,(k-1))=max(k-1);
            end
        end

        % Selection
        % The cost value is computed for the trial vector.
        error_trial=fitness2(ptCloud,trial);
        
        
        % If the error of the trial vector is better, the current
        % population member will be replaced by the trial vector.
        % Threshodlding and Discarding are tools to avoid optimization in
        % the noise band.
        if version_de==2 % Without Thresholding
            if(error_trial<population(i,1)) 
                for j=2:(D+1)
                    pob_aux(i,j)=trial(1,j-1); % Auxiliar pop is used to avoid changes of pop until the whole next population is created
                end
                pob_aux(i,1)=error_trial;
            else
                for j=1:(D+1)
                    pob_aux(i,j)=population(i,j);
                end
            end
        else         % With thresgolding. The improvement must be bigger than 2%.
            if(error_trial<population(i,1)*1.00) % Thresholding band Thau= 0.02
                for j=2:(D+1)
                    pob_aux(i,j)=trial(1,j-1);
                end
                pob_aux(i,1)=error_trial;
                
            else
                for j=1:(D+1) 
                    pob_aux(i,j)=population(i,j); 
                end
            end
        end
    end
    
    % Population for the next generation
    population=pob_aux;
    
    % Discarding: The worst elements are replaced by other ones closer to
    % the best candidates.
    if version_de~=2
        pob_orden=sortrows(population,1);
%         for i=1:(double(int8(NP/20)))
%             disc=random('unid',NP/2);
%             while disc==0
%                 disc=random('unid',NP/2);
%             end
%             pob_orden(NP+1-i,:)=pob_orden(disc,:);
%         end
        population=pob_orden;
    end 
    
    % We register the best and the worst candidates and the sum of errors.   
    error_aux=population(1,1);
    for j=1:(D+1) 
        bestmem(1,j)=population(1,j);
    end
    
    if ((abs(error_aux-error))<1)  % Convergence criterium
        count_b=count_b+1;
    else
        count_b=0;
    end
    error=error_aux;
%     error_max_aux=max(population(:,1));
%     if ((abs(error_max-error_max_aux))<1)   % Convergence criterium
%         count_a=count_a+1;
%     else
%         count_a=0;
%     end
%     error_max=error_max_aux;
%     dif_errores_aux=error_max-error;
%     if (abs(dif_errores_aux-dif_errores)<1)   % Convergence criterium
%         count_c=count_c+1;
%     else
%         count_c=0;
%     end    
%     dif_errores=dif_errores_aux;
%     error_global=sum(population(:,1)); 
    
    % When we are close to convergence, the search area is decreased.
    %if error-error_max<1
    %    F=0.05;
    %end
    % When convergence, NP is drastically reduced
    %if error-error_max==0
    %    NP=10;
    %end
    
    if imp_count==200     
        %fprintf(1,'\n It: %f Best %f Worst: %f Global: %f \n Position: x y z %f %f %f\n Vector: u v w %f %f %f',count,error,error_max,error_global,population(1,2),population(1,3),population(1,4), population(1,5),population(1,6),population(1,7));
        fprintf(1,'\n It: %f Best %f \n Position: x y z %f %f %f\n Vector: u v w %f %f %f \n Radius: %f',count,error,population(1,2),population(1,3),population(1,4), population(1,5),population(1,6),population(1,7),population(1,8));
        
        imp_count=0;
             
        
%         % To display the map and the population set every x iterations.
%         figure(3)
%         imagesc(map','CDataMapping','scaled')
%         %set(gcf,'Color','white');
%         %set(gcf,'BackingStore','off');
%         set(gca,'DataAspectRatio',[1 1 1]);
%         colormap('gray')
%         hold on  
%         plot(population(:,2),population(:,3),'r.','MarkerSize',5);
%         drawnow
        
    end
    imp_count=imp_count+1;    
    
    count=count+1;
end

end


