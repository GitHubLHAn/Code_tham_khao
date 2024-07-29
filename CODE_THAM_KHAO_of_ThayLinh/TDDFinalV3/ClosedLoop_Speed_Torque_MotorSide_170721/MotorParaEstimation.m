Ia_rated = 15;          %Ampere
If_rated = 1;			%Ampere
Speed_rated = 1800.0;	%rpm
Power_rated = 2200;		%W
Va_rated = 220;         %V

Speed_max = 2000;       %rpm
kPhiConst = Power_rated/(Ia_rated*Speed_rated/9.55)
Torque_rated = Ia_rated * kPhiConst
%Amature resistance estimation
Ra = (Va_rated - (kPhiConst*Speed_rated/9.55))/Ia_rated