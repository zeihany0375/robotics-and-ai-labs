function r = rewardfun(state,action,madeit,collided)

x = state(1)/10;
y = state(2)/10;
omega = state(6);

r = 0.06*exp(-3*y) + 0.05*exp(-8*x^2) - 0.14 - 0.001*omega^2 - ...
    - 0.01*sum(action.^2) + 5*madeit - 2*collided;
