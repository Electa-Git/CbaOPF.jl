function constraint_inertia_limit(pm::_PM.AbstractDCPModel, n::Int, gen_inertia, inertia_limit)
    pg    = _PM.var(pm, n, :pg)

    print(inertia_limit)

    JuMP.@constraint(pm.model, sum([pg[g] * iner  for (g, iner) in gen_inertia]) >= inertia_limit)
end