clear
close all
clc

%% Parameters
% lenghts [m]
d1 = 0.333;
d3=0.316;
d5=0.384;
a4=0.0825;
a5=-0.0825;
a7=0.088;


%% Denavit-Hartenberg
L1 = Link([0,d1,  0,  0], 'modified');

L2 = Link([0,0, 0,  -pi/2], 'modified');

L3 = Link([0,d3, 0,  pi/2], 'modified');

L4 = Link([0,0,  a4,  pi/2], 'modified');

L5 = Link([0,d5,  a5,  -pi/2], 'modified');

L6 = Link([0,0,  0,  pi/2], 'modified');

L7 = Link([0,0,  a7,  pi/2], 'modified');

PANDA = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name','PANDA'); % real robot

m1=3.4525; m2=3.4821; m3=4.0562; m4=3.4822; m5=2.1633; 
m6=2.3466; 
m7=0.31290;

r1=transpose([0; -0.03; 0.12]); 
r2=transpose([3.0000e-04; 0.059; 0.042]);
r3=transpose([0; 0.03; 0.13]);
r4=transpose([0; 0.067; 0.034]);
r5=transpose([1.0000e-04; 0.021; 0.076]);
r6=transpose([0; 6.0000e-04; 4.0000e-04]);
r7=transpose([0; 0; 0.02]);


   I1=[0.0747 0.0085 0;
      0.0085 0.0574 0;
      0 0 0.0239];
  
  I2=[0.0390 -0.0086 -0.0037;
    -0.0086 0.0279 -6.1633e-05;
    -0.0037 -6.1633e-05 0.0199];

  I3=[0.006052050623697 0.000000262383560 0.000001120384479;
     0.000000262383560 0.005990028254028 -0.001308542301422;
    0.000001120384479 -0.001308542301422 0.001861529721327];
 
  I4=[0.006052050623697 -0.000000262507583 -0.000001120888863;
      -0.000000262507583 0.005990028254028 -0.001308542301422;
     -0.000001120888863 -0.001308542301422 0.001861529721327];

  I5=[0.005775526977146 -0.000000448127278 0.000000782342032;
      -0.000000448127278 0.005348473437925 0.001819965983941;
      0.000000782342032 0.001819965983941 0.002181233531810];

  I6=[0.001882302441080 0.000000003150206 -0.000000072256604;
      0.000000003150206 0.001889339660303 -0.000012066987492;
     -0.000000072256604 -0.000012066987492 0.002133520179065];

   I7=[0.0003390625 0 0;
      0 0.0003390625 0;
      0 0 0.000528125];


%% Set link mass, lenght and inertia

PANDA.links(1).m = m1;
PANDA.links(2).m = m2;
PANDA.links(3).m = m3;
PANDA.links(4).m = m4;
PANDA.links(5).m = m5;
PANDA.links(6).m = m6;
PANDA.links(7).m = m7;

% PANDAmodel.links(1).m = m1;
% PANDAmodel.links(2).m = m2;
% PANDAmodel.links(3).m = m3;
% PANDAmodel.links(4).m = m4;
% PANDAmodel.links(5).m = m5;
% PANDAmodel.links(6).m = m6;
% PANDAmodel.links(7).m = m7;

PANDA.links(1).r = r1;
PANDA.links(2).r = r2;
PANDA.links(3).r = r3;
PANDA.links(4).r = r4;
PANDA.links(5).r = r5;
PANDA.links(6).r = r6;
PANDA.links(7).r = r7;

% PANDAmodel.links(1).r = r1;
% PANDAmodel.links(2).r = r2;
% PANDAmodel.links(3).r = r3;
% PANDAmodel.links(4).r = r4;
% PANDAmodel.links(5).r = r5;
% PANDAmodel.links(6).r = r6;
% PANDAmodel.links(7).r = r7;

PANDA.links(1).I = I1; 
PANDA.links(2).I = I2;
PANDA.links(3).I = I3; 
PANDA.links(4).I = I4;
PANDA.links(5).I = I5; 
PANDA.links(6).I = I6; 
PANDA.links(7).I = I7; 


% PANDAmodel.links(1).I = I1; 
% PANDAmodel.links(2).I = I2;
% PANDAmodel.links(3).I = I3; 
% PANDAmodel.links(4).I = I4;
% PANDAmodel.links(5).I = I5; 
% PANDAmodel.links(6).I = I6; 
% PANDAmodel.links(7).I = I7; 


%% Plot the robot
q0 = [0 0 0 0 0 0 0];
% KUKA.plot(q0,'floorlevel',0)

syms q1 q2 q3 q4 q5 q6 q7 real
q = [q1 q2 q3 q4 q5 q6 q7];


%% Code generator

save('robot','PANDA')

% save('robotmodel','PANDAsymp_model')

