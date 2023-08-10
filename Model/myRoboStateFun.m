function states = myRoboStateFun(states,inputs)
% StepSize
dt = 0.1;
% States at k+1
states = states + [inputs(1)*cos(states(3)); inputs(1)*sin(states(3)); inputs(2)] * dt;
end