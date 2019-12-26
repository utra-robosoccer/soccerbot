%% Script to define contact forces for many spheres in a single sphere
% Settings for model
mdl  = 'sm_spheres_in_sphere';
subsys_name = 'Spheres in Sphere';
num_sphs = 5;

%% Create model
ssc_new(mdl);
set_param(mdl,'ModelBrowserVisibility','off');
set_param(mdl,'SimMechanicsRigidlyBoundBlock','none');
set_param(mdl,'Solver','ode15s','RelTol','1e-3','AbsTol','1e-4');

% Save parameter values in model workspace so it can be saved with all
% necessary data to run
mdlWks = get_param(mdl,'ModelWorkspace');
assignin(mdlWks,'inner_sphere_rad',0.05);
assignin(mdlWks,'kcon_inner_inner',1e4);
assignin(mdlWks,'bcon_inner_inner',1e2);
assignin(mdlWks,'kcon_inner_outer',1e4);
assignin(mdlWks,'bcon_inner_outer',1e2);
assignin(mdlWks,'muk_inner_inner',0.5);
assignin(mdlWks,'mus_inner_inner',0.7);
assignin(mdlWks,'vth_inner_inner',1e-2);
assignin(mdlWks,'muk_inner_outer',0.5)
assignin(mdlWks,'mus_inner_outer',0.7);
assignin(mdlWks,'vth_inner_outer',1e-2);

outer_sphere_rad_in = 0.15;
outer_sphere_rad_out = 0.02+outer_sphere_rad_in;

% Geometry for outer sphere
avec = 89:-1:-89;
outer_sphere_xs = [...
    0 outer_sphere_rad_out;...
    0 outer_sphere_rad_in;...
    [cosd(avec)*outer_sphere_rad_in]' [sind(avec)*outer_sphere_rad_in]';...
    0 -outer_sphere_rad_in;...
    0 -outer_sphere_rad_out;...
    flipud([[cosd(avec)*outer_sphere_rad_out]' [sind(avec)*outer_sphere_rad_out]'])];

assignin(mdlWks,'outer_sphere_rad_in',outer_sphere_rad_in);   % inner radius
assignin(mdlWks,'outer_sphere_rad_out',outer_sphere_rad_out);  % outer radius
assignin(mdlWks,'outer_sphere_xs',outer_sphere_xs);

% Color and opacity for spheres
assignin(mdlWks,'clr_inner_sphere',[0.2 0.4 0.6]);
assignin(mdlWks,'clr_outer_sphere',[0.6 0.6 0.6]);

% List of blocks with path to location within libraries
% Required for add_block() command
s2sp = 'Contact_Forces_Lib/3D/Sphere to Sphere Force';
sisp = 'Contact_Forces_Lib/3D/Sphere in Sphere Force';
conn = 'nesl_utility/Connection Port';
rtfm = 'sm_lib/Frames and Transforms/Rigid Transform';
wdfm = 'sm_lib/Frames and Transforms/World Frame';
rvjt = 'sm_lib/Joints/Revolute Joint';
sdjt = 'sm_lib/Joints/6-DOF Joint';
slid = 'sm_lib/Body Elements/Solid';
mcfg = 'sm_lib/Utilities/Mechanism Configuration';
scfg = 'nesl_utility/Solver Configuration';

% Adjust block references based on MATLAB version
if (strncmp(version('-release'),'2012',4) || strncmp(version('-release'),'2013a',5))
    mcfg = 'sm_lib/Utilities/Mechanism  Configuration';
    rtfm = 'sm_lib/Frames and  Transforms/Rigid  Transform';
    wdfm = 'sm_lib/Frames and  Transforms/World Frame';
end

% Block diagram constants
blk_or = [100 155 140 195];         % Start position and size for square block
blk_or_conn = [100 155 130 175];    % Start position and size for connection port
blk_or_conf = [100 180 130 165];    % Start position and size for contact force
blk_xo = [80 0 80 0];               % Standard block diagram offset in x
blk_yo = [0 80 0 80];               % Standard block diagram offset in y
conn_size = [0 0 30 20];            % Size of connection port block
conf_size = [0 0 80 25];            % Size of contact force block
solid_size = [0 0 40 40];            % Size of contact force block
subsys_position = [230 154 330 196];% Position of subsystem in top level diagram

%% Clean out model

% Delete lines
linehan = find_system(mdl,'FindAll','on','type','line');
if ~isempty(linehan)
    delete(linehan)
end

% Delete blocks
blkhan = find_system(mdl,'Type','Block');
if ~isempty(blkhan)
    delete_block(blkhan)
end

%% Add and connect required blocks
wdfm_h = add_block(wdfm,[mdl '/World Frame'],'Position',blk_or,...
    'Orientation','Right');
add_block(mcfg,[mdl '/Mechanism Configuration'],...
    'Position',blk_or + blk_yo,'Orientation','Right');
add_line(mdl,'World Frame/RConn1',...
    'Mechanism Configuration/RConn1');
add_block(scfg,[mdl '/Solver Configuration'],...
    'Position',blk_or - blk_yo,'Orientation','Right');
add_line(mdl,'World Frame/RConn1',...
    'Solver Configuration/RConn1');

%% Add and clean subsystem
sub_h = add_block('simulink/Ports & Subsystems/Subsystem',[mdl '/' subsys_name],'Position',subsys_position);

% Delete lines
linehan = find_system(sub_h,'LookUnderMasks','all','FollowLinks','on','FindAll','on','type','line');
if ~isempty(linehan)
    delete(linehan)
end

% Delete blocks
inport_h = find_system(sub_h,'BlockType','Inport');
outport_h = find_system(sub_h,'BlockType','Outport');
delete_block(inport_h);
delete_block(outport_h);

% Store path to subsystem
subs_path = [bdroot '/' get_param(sub_h,'Name')];

%% Add solids, joints, and connection port

% Add connection port
conn_h = add_block(conn,...
    [bdroot '/' get_param(sub_h,'Name') '/Ref'],...
    'Position',blk_or_conn-blk_xo,...
    'Orientation','Right','Name','Ref');

% Add solids and joints
solid_pos = blk_or;  % Position of first inner sphere
for insph_i = 1:num_sphs
    
    % Offset solid blocks vertically in diagram (except first one)
    if(insph_i>1)
        solid_pos(insph_i,:) = solid_pos(insph_i-1,:) + blk_yo*(num_sphs-insph_i+1+1);
    end
    
    % Add solid blocks for inner spheres
    insph_h(insph_i) = add_block(slid,...
        [bdroot '/' get_param(sub_h,'Name') ['/InSph' num2str(insph_i)]],...
        'Position',solid_pos(insph_i,:),...
        'Orientation','Right','Name',['InSph' num2str(insph_i)]);
    
    % Add 6-DOF Joints for inner spheres
    insphjt_h(insph_i) = add_block(sdjt,...
        [bdroot '/' get_param(sub_h,'Name') ['/InSph Joint ' num2str(insph_i)]],...
        'Position',solid_pos(insph_i,:)+ blk_yo,...
        'Orientation','Right','Name',['InSph Joint' num2str(insph_i)]);
    
    % Connect solids to joints
    add_line(subs_path,[get_param(insph_h(insph_i),'Name') '/RConn1'],...
        [get_param(insphjt_h(insph_i),'Name') '/RConn1'],'autorouting','on');
    
    % Connect joints to connection port
    add_line(subs_path,[get_param(insphjt_h(insph_i),'Name') '/LConn1'],...
        [get_param(conn_h,'Name') '/RConn1'],'autorouting','on');
    
    % Set parameters for inner spheres
    set_param(insph_h(insph_i),...
        'GeometryShape','Sphere',...
        'SphereRadius','inner_sphere_rad',...
        'GraphicDiffuseColor','clr_inner_sphere',...
        'Density','1000');
    
    % Set initial position of inner spheres
    set_param(insphjt_h(insph_i),...
        'PxPositionTargetSpecify','on',...
        'PxPositionTargetValue',...
        ['(outer_sphere_rad_in-inner_sphere_rad)*cosd(360*(' num2str(insph_i-1) '/' num2str(num_sphs) '))'],...
        'PyPositionTargetSpecify','on',...
        'PyPositionTargetValue',...
        ['(outer_sphere_rad_in-inner_sphere_rad)*sind(360*(' num2str(insph_i-1) '/' num2str(num_sphs) '))']);
    
end


%%  Add Outer Sphere

outsph_h = add_block(slid,...
    [bdroot '/' get_param(sub_h,'Name') '/OutSph'],...
    'Position',blk_or+[1 0 1 0]*600,...
    'Orientation','Left');

set_param(outsph_h,...
    'GeometryShape','GeneralSolidOfRevolution',...
    'RevolutionCrossSection','outer_sphere_xs',...
    'RevolutionExtent','Full',...
    'GraphicDiffuseColor','clr_outer_sphere',...
    'Density','1000',...
    'GraphicOpacity','0.4');

% Connect solid for outer sphere to connection port
add_line(subs_path,[get_param(outsph_h,'Name') '/RConn1'],...
    [get_param(conn_h,'Name') '/RConn1'],'autorouting','on');

%% Add internal contact forces

% Loop over inner spheres
for insph_i = 1:num_sphs
    
    % Do for all spheres except the last one
    if(insph_i<num_sphs)
        
        % For all spheres with a higher index
        for insph_j = insph_i+1:num_sphs

            % Determine position for contact force block (inner-inner)
            % relative to solid block
            s2sp_pos = solid_pos(insph_i,:) - solid_size + conf_size...
                +(insph_j-insph_i)*blk_yo/2+[1 0 1 0]*200;
            
            % Add contact force between current sphere and other spheres
            % Sphere to Sphere
            s2sp_h = add_block(s2sp,...
                [bdroot '/' get_param(sub_h,'Name') '/Sph2Sph ' num2str(insph_i) ' ' num2str(insph_j)],...
                'Position',s2sp_pos,...
                'Orientation','Right');
            
            % Connect contact force to current sphere
            add_line(subs_path,[get_param(s2sp_h,'Name') '/LConn1'],...
                [get_param(insph_h(insph_i),'Name') '/RConn1'],'autorouting','on');
            
            % Connect contact force to other spheres
            add_line(subs_path,[get_param(s2sp_h,'Name') '/RConn1'],...
                [get_param(insph_h(insph_j),'Name') '/RConn1'],'autorouting','on');
            
            % Assign MATLAB variables to contact force parameters
            set_param(s2sp_h,...
                'sphB_rad','inner_sphere_rad',...
                'sphF_rad','inner_sphere_rad',...
                'k_contact','kcon_inner_inner',...
                'b_contact','bcon_inner_inner',...
                'mu_kinetic','muk_inner_inner',...
                'mu_static','mus_inner_inner',...
                'v_thr','vth_inner_inner');
        end
    end
    
    % Determine position for contact force block (inner-outer)
    % relative to solid block
    sisp_pos = solid_pos(insph_i,:) - solid_size + conf_size...
        +[1 0 1 0]*400;
    
    % Add contact force block (inner-outer, Sphere in Sphere)
    sisp_h = add_block(sisp,...
        [bdroot '/' get_param(sub_h,'Name') '/SphinSph ' num2str(insph_i)],...
        'Position',sisp_pos,...
        'Orientation','Right');
    
    % Connect contact force to current sphere
    add_line(subs_path,[get_param(sisp_h,'Name') '/LConn1'],...
        [get_param(insph_h(insph_i),'Name') '/RConn1'],'autorouting','on');
    
    % Connect contact force to outer sphere
    add_line(subs_path,[get_param(sisp_h,'Name') '/RConn1'],...
        [get_param(outsph_h,'Name') '/RConn1'],'autorouting','on');
    
    % Assign MATLAB variables to contact force parameters
    set_param(sisp_h,...
        'sphB_rad','outer_sphere_rad_in',...
        'sphF_rad','inner_sphere_rad',...
        'k_contact','kcon_inner_outer',...
        'b_contact','bcon_inner_outer',...
        'mu_kinetic','muk_inner_outer',...
        'mu_static','mus_inner_outer',...
        'v_thr','vth_inner_outer');
    
end

%% Connect subsystem to world
add_line(mdl,[subsys_name '/LConn1'],...
        [get_param(wdfm_h,'Name') '/RConn1'],'autorouting','on');

