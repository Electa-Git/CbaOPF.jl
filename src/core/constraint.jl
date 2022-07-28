function constraint_inertia_limit(pm::_PM.AbstractDCPModel, n::Int, gen_inertia, inertia_limit)
    pg    = _PM.var(pm, n, :pg)

    print(inertia_limit)

    JuMP.@constraint(pm.model, sum([pg[i] * gen_inertia[(n,i)] for (i, gen) in _PM.ref(pm, n, :gen)]) >= inertia_limit)
end