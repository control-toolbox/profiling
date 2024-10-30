ocp = Model()                                   # empty optimal control problem

time!(ocp, t0=0, tf=1)                          # initial and final times
state!(ocp, 2)                                  # dimension of the state
control!(ocp, 1)                                # dimension of the control

constraint!(ocp, :initial; val=[ -1, 0 ])       # initial condition
constraint!(ocp, :final;   val=[  0, 0 ])       # final condition

dynamics!(ocp, (x, u) -> [ x[2], u ])           # dynamics of the double integrator

objective!(ocp, :lagrange, (x, u) -> 0.5u^2)    # cost in Lagrange form