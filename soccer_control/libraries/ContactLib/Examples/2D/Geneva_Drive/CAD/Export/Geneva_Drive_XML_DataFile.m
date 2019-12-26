% Simscape(TM) Multibody(TM) version: 4.8

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.



%============= Transform =============%

%Initialize the Transform structure array by filling in null values.
Transform(6).translation = [0.0 0.0 0.0];
Transform(6).angle = 0.0;
Transform(6).axis = [0.0 0.0 0.0];
Transform(6).ID = '';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
Transform(1).translation = [0 99.999999999999943 1.0000000000000009];  % mm
Transform(1).angle = 0;  % rad
Transform(1).axis = [0 0 0];
Transform(1).ID = 'UCS[Drive_Wheel*:*Default:SMLINK_Rotator_Pin]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
Transform(2).translation = [100 0 0];  % mm
Transform(2).angle = 0;  % rad
Transform(2).axis = [0 0 0];
Transform(2).ID = 'B[Base-1:-:Geneva-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
Transform(3).translation = [99.999999999999261 -2.4158453015843406e-13 -1.4210854715202004e-14];  % mm
Transform(3).angle = 0;  % rad
Transform(3).axis = [0 0 0];
Transform(3).ID = 'F[Base-1:-:Geneva-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
Transform(4).translation = [-99.999999999999986 0 57.42640687119286];  % mm
Transform(4).angle = 3.1415926535897931;  % rad
Transform(4).axis = [1 0 0];
Transform(4).ID = 'B[Base-1:-:Drive_Wheel-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
Transform(5).translation = [-100.00000000000026 2.1316282072803006e-14 57.426406871192782];  % mm
Transform(5).angle = 3.1415926535897931;  % rad
Transform(5).axis = [-1 5.9163189700902024e-33 -6.6609835156154749e-17];
Transform(5).ID = 'F[Base-1:-:Drive_Wheel-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
Transform(6).translation = [-30.994181557875464 40 -14.83110487348719];  % mm
Transform(6).angle = 1.5707963267948966;  % rad
Transform(6).axis = [-1 0 0];
Transform(6).ID = 'RootGround[Base-1]';


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
Solid(3).mass = 0.0;
Solid(3).CoM = [0.0 0.0 0.0];
Solid(3).MoI = [0.0 0.0 0.0];
Solid(3).PoI = [0.0 0.0 0.0];
Solid(3).color = [0.0 0.0 0.0];
Solid(3).opacity = 0.0;
Solid(3).ID = '';

%Inertia Type - Custom
%Visual Properties - Simple
Solid(1).mass = 1.1459440199505517;  % kg
Solid(1).CoM = [-103.32572655678197 -3.3257265567819814 0.56035000006051983];  % mm
Solid(1).MoI = [5063.1142307875289 5063.1142307875298 10008.151794093599];  % kg*mm^2
Solid(1).PoI = [46.956086417341879 46.956086417341901 41.94861289102812];  % kg*mm^2
Solid(1).color = [0.20000000000000001 0.40000000000000002 1];
Solid(1).opacity = 1;
Solid(1).ID = 'Drive_Wheel*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
Solid(2).mass = 0.24105441419713447;  % kg
Solid(2).CoM = [100.00000000000023 1.1564451014440063e-12 5.0000000000000009];  % mm
Solid(2).MoI = [803.62795532691052 803.62795532691246 1603.261580672231];  % kg*mm^2
Solid(2).PoI = [0 0 0];  % kg*mm^2
Solid(2).color = [1 0.40000000000000002 0];
Solid(2).opacity = 1;
Solid(2).ID = 'Geneva*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
Solid(3).mass = 5.6121441650895836;  % kg
Solid(3).CoM = [0.22415415704528135 0 -24.782168314450626];  % mm
Solid(3).MoI = [57246.923822174664 131168.16755878026 187513.90774897448];  % kg*mm^2
Solid(3).PoI = [0 -24.879239071091789 0];  % kg*mm^2
Solid(3).color = [0.82745098039215681 0.82745098039215681 0.81960784313725488];
Solid(3).opacity = 1;
Solid(3).ID = 'Base*:*Default';


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the RevoluteJoint structure array by filling in null values.
RevoluteJoint(2).Rz.Pos = 0.0;
RevoluteJoint(2).ID = '';

RevoluteJoint(1).Rz.Pos = 90.000000000003197;  % deg
RevoluteJoint(1).ID = '[Base-1:-:Geneva-1]';

RevoluteJoint(2).Rz.Pos = 106.26478005487978;  % deg
RevoluteJoint(2).ID = '[Base-1:-:Drive_Wheel-1]';

