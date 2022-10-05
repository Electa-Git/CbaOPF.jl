function constraint_inertia_limit(pm::_PM.AbstractDCPModel, n::Int, generator_properties, inertia_limit)
    alpha_g = _PM.var(pm, n, :alpha_g)

    JuMP.@constraint(pm.model, sum([properties["inertia"] * properties["rating"] * alpha_g[g] / 0.9 for (g, properties) in generator_properties])  >= inertia_limit) #
    #JuMP.@constraint(pm.model, sum([pg[g] * iner_g  for (g, iner_g) in gen_inertia]) - sum([pconv_grid_ac[c] * iner_c  for (c, iner_c) in conv_inertia])  >= inertia_limit) #
end

function constraint_inertia_limit(pm::_PM.AbstractNFAModel, n::Int, generator_properties, inertia_limit)
    pg = _PM.var(pm, n, :pg)
    if !isempty(generator_properties)
        JuMP.@constraint(pm.model, sum([properties["inertia"] * pg[g] / 0.9 for (g, properties) in generator_properties])  >= inertia_limit)
    end #
    #JuMP.@constraint(pm.model, sum([pg[g] * iner_g  for (g, iner_g) in gen_inertia]) - sum([pconv_grid_ac[c] * iner_c  for (c, iner_c) in conv_inertia])  >= inertia_limit) #
end