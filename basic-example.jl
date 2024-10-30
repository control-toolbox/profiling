using OptimalControl
using NLPModelsIpopt

ocp = @def begin
    t ∈ [0, 1], time
    x ∈ R², state
    u ∈ R, control
    x(0) == [ -1, 0 ]
    x(1) == [ 0, 0 ]
    ẋ(t) == [ x₂(t), u(t) ]
    ∫( 0.5u(t)^2 ) → min
end

# classical usage: solve the problem
sol = solve(ocp; grid_size=100)

# advanced usage: get the NLP model, the solution and play with it
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