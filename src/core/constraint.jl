function constraint_inertia_limit(pm::_PM.AbstractDCPModel, n::Int, gen_inertia, conv_inertia, inertia_limit)
    pg    = _PM.var(pm, n, :pg)
    pconv_grid_ac = _PM.var(pm, n,  :pconv_tf_fr)

    JuMP.@constraint(pm.model, sum([pg[g] * iner_g  for (g, iner_g) in gen_inertia]) - sum([pconv_grid_ac[c] * iner_c  for (c, iner_c) in conv_inertia])  >= inertia_limit) #
end