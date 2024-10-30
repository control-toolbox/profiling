using OptimalControl
using NLPModelsIpopt
using NLPModels

t0 = 0      # initial time
r0 = 1      # initial altitude
v0 = 0      # initial speed
m0 = 1      # initial mass
vmax = 0.1  # maximal authorized speed
mf = 0.6    # final mass to target

ocp = @def begin # definition of the optimal control problem

    tf ∈ R, variable
    t ∈ [t0, tf], time
    x = (r, v, m) ∈ R³, state
    u ∈ R, control

    x(t0) == [ r0, v0, m0 ]
    m(tf) == mf,         (1)
    0 ≤ u(t) ≤ 1
    r(t) ≥ r0
    0 ≤ v(t) ≤ vmax

    ẋ(t) == F0(x(t)) + u(t) * F1(x(t))

    r(tf) → max

end;

# Dynamics
const Cd = 310
const Tmax = 3.5
const β = 500
const b = 2

F0(x) = begin
    r, v, m = x
    D = Cd * v^2 * exp(-β*(r - 1)) # Drag force
    return [ v, -D/m - 1/r^2, 0 ]
end

F1(x) = begin
    r, v, m = x
    return [ 0, Tmax/m, -b*Tmax ]
end

# classical usage: solve the problem
sol = solve(ocp; grid_size=100)

# advanced usage: get the NLP model and play with it
docp, nlp = direct_transcription(ocp)

nlp_sol = ipopt(nlp; print_level=5, mu_strategy="adaptive", tol=1e-8, sb="yes")

model = nlp
x = nlp_sol.solution
y = nlp_sol.multipliers

obj(model, x)
cons(model, x)
grad(model, x)
jac(model, x)
hess(model, x, y)