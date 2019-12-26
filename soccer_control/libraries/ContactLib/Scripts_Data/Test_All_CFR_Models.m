% Copyright 2014-2019 The MathWorks, Inc.

clear CFR_Res

cd([CFR_HomeDir '/Scripts_Data']);

mdlnames = {'Collision_01_Ball_Infinite_Plane',...
    'Collision_02_Disk_Finite_Plane_Fixed',...
    'Collision_03_Disk_Finite_Plane_Spin',...
    'Collision_04_Disks_in_Box',...
    'Collision_05_Disk_in_Ring',...
    'Collision_06_Catapult',...
    'Collision_07_Ball_Finite_Plane_Float',...
    'Collision_08_Compare_Forces',...
    'Friction_01_Box_on_Ramp_Constraint',...
    'Friction_02_Box_on_Ramp',...
    'Friction_03_Double_Pendulum_Constraint',...
    'Friction_04_Disk_Rolling_on_Ramp',...
    'Friction_05_Beam_on_Wheel',...
    'Friction_06_Disk_on_Disk',...
    'Friction_07_Floating_Disks',...
    'Friction_08_Disks_and_Ring',...
    'Friction_09_Ring_on_Disk_Float',...
    'Friction_10_Ball_on_Wheel',...
    'sm_ball_bearing_testrig',...
    'Cam_Follower',...
    'Cam_Follower_Constraint',...
    'Geneva_Drive',...
    'Spinning_Boxes',...
    'Belts_01_Two_Belts',...
    'Coll3D_01_Ball_Plane_Fixed',...
    'Coll3D_02_Ball_Plane_Spin',...
    'Coll3D_03_Balls_in_Box',...
    'Coll3D_04_Ball_in_Tube_Fixed',...
    'Coll3D_05_Ball_Peg_Board',...
    'Coll3D_06_Ball_in_Ball',...
    'Coll3D_07_Balls_and_Sliding_Tube',...
    'Coll3D_08_Ball_in_Spinning_Cone',...
    'Frict3D_01_Box_on_Table',...
    'Frict3D_02_Ball_on_Table',...
    'Frict3D_03_Board_on_Balls',...
    'Frict3D_04_Ball_on_Ball',...
    'Frict3D_05_Tube_on_Balls',...
    'Frict3D_06_Ball_on_Balls',...
    'Frict3D_07_Ball_in_Ball',...
    'Frict3D_08_Ball_on_Driven_Tube',...
    'Robot_2_Whl',...
    'Caster_4_Whl',...
    'Gripper_2Belts',...
    'sm_tread_drive',...
    };

CFR_Res{1,1} = '#';
CFR_Res{1,2} = 'Model';
CFR_Res{1,3} = 'Stop Time';
CFR_Res{1,4} = '# Steps';
CFR_Res{1,5} = 'Sim Time';
CFR_Res{1,6} = '# Steps';
CFR_Res{1,7} = 'Sim Time';
CFR_Res{1,8} = '# Steps';
CFR_Res{1,9} = 'Sim Time';
CFR_Res{1,10} = '# Steps';
CFR_Res{1,11} = 'Sim Time';


force_law_list = {'force_law','force_law_box2box','force_law_box2belt'};
friction_law_list = {'friction_law','friction_law_box2box','friction_law_box2belt'};
force_variants = {'Linear','Nonlinear'};
friction_variants = {'None','Stick-Slip Continuous'};

for mdl_i = 36:length(mdlnames)
    mdl = char(mdlnames{mdl_i});
    open_system(mdl);
    if(strcmp(mdl,'sm_ball_bearing_testrig'))
        set_param([bdroot '/Ball Bearing'],'popup_constraints','Forces')
        sm_ball_bearing_testrig_param
    end
    disp(['TESTING ' mdl ' ...']);
    
    for frclaw_i=1:length(force_law_list)
        block_pths = find_system(bdroot,'RegExp','on','LookUnderMasks','all',char(force_law_list(frclaw_i)),'.*');
        if(~isempty(block_pths))
            for i=1:length(block_pths)
                set_param(char(block_pths(i)),char(force_law_list(frclaw_i)),'Linear');
            end
        end
    end
    
    for frilaw_i=1:length(friction_law_list)
        block_pths = find_system(bdroot,'RegExp','on','LookUnderMasks','all',char(friction_law_list(frilaw_i)),'.*');
        if(~isempty(block_pths))
            for i=1:length(block_pths)
                set_param(char(block_pths(i)),char(friction_law_list(frilaw_i)),'None');
            end
        end
    end
    
    sim(mdl);
    CFR_Res{mdl_i+1,1} = mdl_i;
    CFR_Res{mdl_i+1,2} = mdl;
    CFR_Res{mdl_i+1,3} = sprintf('%5.2f',str2double(get_param(bdroot,'StopTime')));
    CFR_Res{mdl_i+1,4} = num2str(length(tout));
    CFR_Res{mdl_i+1,5} = sprintf('%5.3f',Elapsed_Sim_Time);
    
    for frclaw_i=1:length(force_law_list)
        block_pths = find_system(bdroot,'RegExp','on','LookUnderMasks','all',char(force_law_list(frclaw_i)),'.*');
        if(~isempty(block_pths))
            for i=1:length(block_pths)
                set_param(char(block_pths(i)),char(force_law_list(frclaw_i)),'Linear');
            end
        end
    end
    
    for frilaw_i=1:length(friction_law_list)
        block_pths = find_system(bdroot,'RegExp','on','LookUnderMasks','all',char(friction_law_list(frilaw_i)),'.*');
        if(~isempty(block_pths))
            for i=1:length(block_pths)
                set_param(char(block_pths(i)),char(friction_law_list(frilaw_i)),'Stick-Slip Continuous');
            end
        end
    end
    
    sim(mdl);
    CFR_Res{mdl_i+1,6} = num2str(length(tout));
    CFR_Res{mdl_i+1,7} = sprintf('%5.3f',Elapsed_Sim_Time);
    
    for frclaw_i=1:length(force_law_list)
        block_pths = find_system(bdroot,'RegExp','on','LookUnderMasks','all',char(force_law_list(frclaw_i)),'.*');
        if(~isempty(block_pths))
            for i=1:length(block_pths)
                set_param(char(block_pths(i)),char(force_law_list(frclaw_i)),'Nonlinear');
            end
        end
    end
    
    for frilaw_i=1:length(friction_law_list)
        block_pths = find_system(bdroot,'RegExp','on','LookUnderMasks','all',char(friction_law_list(frilaw_i)),'.*');
        if(~isempty(block_pths))
            for i=1:length(block_pths)
                set_param(char(block_pths(i)),char(friction_law_list(frilaw_i)),'None');
            end
        end
    end
    
    sim(mdl);
    CFR_Res{mdl_i+1,8} = num2str(length(tout));
    CFR_Res{mdl_i+1,9} = sprintf('%5.3f',Elapsed_Sim_Time);
    
    for frclaw_i=1:length(force_law_list)
        block_pths = find_system(bdroot,'RegExp','on','LookUnderMasks','all',char(force_law_list(frclaw_i)),'.*');
        if(~isempty(block_pths))
            for i=1:length(block_pths)
                set_param(char(block_pths(i)),char(force_law_list(frclaw_i)),'Nonlinear');
            end
        end
    end
    
    for frilaw_i=1:length(friction_law_list)
        block_pths = find_system(bdroot,'RegExp','on','LookUnderMasks','all',char(friction_law_list(frilaw_i)),'.*');
        if(~isempty(block_pths))
            for i=1:length(block_pths)
                set_param(char(block_pths(i)),char(friction_law_list(frilaw_i)),'Stick-Slip Continuous');
            end
        end
    end
    
    sim(mdl);
    CFR_Res{mdl_i+1,10} = num2str(length(tout));
    CFR_Res{mdl_i+1,11} = sprintf('%5.3f',Elapsed_Sim_Time);
    
    bdclose(mdl);
    disp(['FINISHED TESTING ' mdl]);
end

cd([CFR_HomeDir '/Examples/2D/Mini_Golf']);
startup_Mini_Golf
Mini_Golf_Run_Course
bdclose

rows_CFR_Res = size(CFR_Res,1);

for i=1:length(MGT.Steps)
    CFR_Res{rows_CFR_Res+i,1} = rows_CFR_Res+i-1;
    CFR_Res{rows_CFR_Res+i,2} = ['Mini Golf Hole ' num2str(i)];
    CFR_Res{rows_CFR_Res+i,3} = sprintf('%5.2f',MGT.StopTime(i));
    CFR_Res{rows_CFR_Res+i,4} = num2str(MGT.Steps(i));
    CFR_Res{rows_CFR_Res+i,5} = sprintf('%5.3f',MGT.SimTime(i));
    CFR_Res{rows_CFR_Res+i,6} = 'n/a';
    CFR_Res{rows_CFR_Res+i,7} = 'n/a';
    CFR_Res{rows_CFR_Res+i,8} = 'n/a';
    CFR_Res{rows_CFR_Res+i,9} = 'n/a';
    CFR_Res{rows_CFR_Res+i,10} = 'n/a';
    CFR_Res{rows_CFR_Res+i,11} = 'n/a';
end
cd([CFR_HomeDir '/Scripts_Data']);

xlswrite('CFR_Results.xlsx',{'Contact Forces Tests'},'Results','B1');
xlswrite('CFR_Results.xlsx',{version},'Results','C1');
xlswrite('CFR_Results.xlsx',{datestr(now)},'Results','F1');
CFL_verstr = strsplit(CFL_ver);
xlswrite('CFR_Results.xlsx',{['CFL ' CFL_verstr{6}]},'Results','H1');
xlswrite('CFR_Results.xlsx',CFR_Res,'Results','A3')
