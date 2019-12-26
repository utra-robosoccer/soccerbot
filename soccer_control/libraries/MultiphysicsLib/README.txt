Simscape Multibody Multiphysics Library.

Run startup_sm_ssci.m to get started

This file contains example models showing how to extend Simscape Multibody models
by adding physical effects spanning multiple physical domains modeled in Simscape. 

Connecting the models using Simscape Physical Signals ensures a lossless transfer 
of power between physical networks. This submission contains a library that contains 
general interface blocks (rotational, translational), and example models showing 
how to use them to model multidomain physical systems.

You need to ensure that your use of these interfaces is physically valid.  Connecting
a 3D mechanical model to a 1D physical systems requires that you follow a few basic
rules:

1. Never add inertia directly to the node on the Simscape side of the interface.
  
   All masses in Simscape models live in an implicit inertial reference frame. A Simscape mechanical 
   circuit interfaced to a Simscape Multibody machine in general moves in an accelerated frame. A simulation 
   with such a circuit does not include the pseudoforces acting on the Simscape mass and inertia elements 
   as experienced in such a noninertial frame and thus violates Newton's second law of mechanics.

2. If you must model inertia in the Simscape network, connect it to the interface element 
   via a spring and damper connected in parallel.  Be aware that a Simscape circuit does not model 
   the motion of such bodies along or about axes orthogonal to the coupled primitive axis chosen 
   in the interfaced Joint.

3. Quantities sensed in Simscape (like translation at a node) may be offset from comparable quantities
   measured in Simscape Multibody.  This is because the initial position of the Simscape Multibody joint,
   which is determined during the assembly process, is not automatically conveyed to the Simscape network.
   You must either use MATLAB variables to synchronize the setting of the initial position or feed
   the position from Simscape Multibody to the Simscape network.  The examples in this submission
   show how to do that.

Copyright 2013-2017 The MathWorks, Inc.

#########  Release History  #########  
v 2.3 (R2017a)	July 2017	Fixed mistake in library (Interfaces/Translational Simscape Multibody).
                            Changed checkbox from torque to force.
                            Added example sm_ssci_01_slider_crank.slx                            

v 2.2 (R2017a)	May  2017	Initial release (version number set to match File Exchange)
                            Includes general 3D-1D interface blocks as well as abstract multiphysics
                            blocks connecting hydraulic, electrical, and mechanical effects to
                            multibody systems.  5 basic examples and one CAD workflow example.

