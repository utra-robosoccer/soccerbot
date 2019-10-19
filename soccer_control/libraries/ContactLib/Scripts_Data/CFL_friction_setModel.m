function CFL_friction_setModel(mdlname,frictionModel)
% Turn friction on or off in all contact force blocks in a model
% from Simscape Multibody Contact Force Library. Pass the name of the model
% as an argument to this function

% Copyright 2014-2017 The MathWorks, Inc.

CF_bpth=find_system(mdlname,'RegExp','on','LookUnderMasks','on','FollowLinks','on','friction_law','.*');
CF_bpth_box2box=find_system(mdlname,'RegExp','on','LookUnderMasks','on','FollowLinks','on','friction_law_box2box','.*');
CF_bpth_box2belt=find_system(mdlname,'RegExp','on','LookUnderMasks','on','FollowLinks','on','friction_law_box2belt','.*');

if(~isempty(CF_bpth))
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
        
        if(within_composite_force==0)
            set_param(char(CF_bpth(i)),'friction_law',frictionModel);
            %disp(['Force ' char(CF_bpth(i)) ' not in composite force']);
            %else
            %    disp(['Force ' char(CF_bpth(i)) ' WITHIN composite force']);
        end
    end
end

if(~isempty(CF_bpth_box2box))
    for j=1:length(CF_bpth_box2box)
        set_param(char(CF_bpth_box2box(j)),'friction_law_box2box',frictionModel)
    end
end

if(~isempty(CF_bpth_box2belt))
    for j=1:length(CF_bpth_box2belt)
        set_param(char(CF_bpth_box2belt(j)),'friction_law_box2belt',frictionModel)
    end
end

end
