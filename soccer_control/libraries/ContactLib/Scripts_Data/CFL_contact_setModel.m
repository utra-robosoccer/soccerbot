function CFL_contact_setModel(mdlname,contactModel)
% Select contact force model in all contact force blocks in a model from
% Simscape Multibody Contact Force Library. Pass the name of the model as
% an argument to this function.

% Copyright 2014-2019 The MathWorks, Inc.

CF_bpth=find_system(mdlname,'RegExp','on','LookUnderMasks','on','FollowLinks','on','force_law','.*');
CF_bpth_box2box=find_system(mdlname,'RegExp','on','LookUnderMasks','on','FollowLinks','on','force_law_box2box','.*');
CF_bpth_box2belt=find_system(mdlname,'RegExp','on','LookUnderMasks','on','FollowLinks','on','force_law_box2belt','.*');
CF_bpth_sph2belt=find_system(mdlname,'RegExp','on','LookUnderMasks','on','FollowLinks','on','force_law_sph2bel','.*');

if(~isempty(CF_bpth))
    
    % Check if force is part of a composite force
    % In that case, adjust the setting in the top level mask
    for i=1:length(CF_bpth)
        within_composite_force = 0;
        
        if(~isempty(CF_bpth_box2box))
            for j=1:length(CF_bpth_box2box)
                if (strfind(char(CF_bpth(i)),char(CF_bpth_box2box(j))))
                    within_composite_force = 1;
                end
            end
        end
        
        if(~isempty(CF_bpth_box2belt))
            for k=1:length(CF_bpth_box2belt)
                if (strfind(char(CF_bpth(i)),char(CF_bpth_box2belt(k))))
                    within_composite_force = 1;
                end
            end
        end
        
        if(~isempty(CF_bpth_sph2belt))
            for k=1:length(CF_bpth_sph2belt)
                if (strfind(char(CF_bpth(i)),char(CF_bpth_sph2belt(k))))
                    within_composite_force = 1;
                end
            end
        end
        
        if(within_composite_force==0)
            set_param(char(CF_bpth(i)),'force_law',contactModel);
            %disp(['Force ' char(CF_bpth(i)) ' not in composite force']);
            %else
            %    disp(['Force ' char(CF_bpth(i)) ' WITHIN composite force']);
        end
    end
end

if(~isempty(CF_bpth_box2box))
    for j=1:length(CF_bpth_box2box)
        set_param(char(CF_bpth_box2box(j)),'force_law_box2box',contactModel)
    end
end

if(~isempty(CF_bpth_box2belt))
    for j=1:length(CF_bpth_box2belt)
        set_param(char(CF_bpth_box2belt(j)),'force_law_box2belt',contactModel)
    end
end

if(~isempty(CF_bpth_sph2belt))
    for j=1:length(CF_bpth_sph2belt)
        set_param(char(CF_bpth_sph2belt(j)),'force_law_sph2bel',contactModel)
    end
end

end
