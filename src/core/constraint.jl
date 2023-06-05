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

function constraint_active_conv_setpoint(pm::_PM.AbstractPowerModel, n::Int, i, pconv)
    pconv_var = _PM.var(pm, n, :pconv_tf_fr, i)

    if pconv >= 0
        JuMP.@constraint(pm.model, pconv_var >= - pconv - pconv/10)
        JuMP.@constraint(pm.model, pconv_var <= + pconv + pconv/10)
    else
        JuMP.@constraint(pm.model, pconv_var >= + pconv + pconv/10)
        JuMP.@constraint(pm.model, pconv_var <= - pconv - pconv/10)
    end
end

# Calucating the net storage injection
function constraint_net_storage_injection(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    ps = _PM.var(pm, nw, :ps, i)
    ps_ch = _PM.var(pm, nw, :ps_ch, i)
    ps_dch = _PM.var(pm, nw, :ps_dch, i)

    JuMP.@constraint(pm.model, ps == ps_dch - ps_ch)
end

# Initial state of charge of storage
function constraint_storage_initial_state(pm::_PM.AbstractPowerModel, n::Int, i::Int, energy, charge_eff, discharge_eff, time_elapsed)
    es = _PM.var(pm, n, :es, i)
    ps_ch = _PM.var(pm, n, :ps_ch, i)
    ps_dch = _PM.var(pm, n, :ps_dch, i)

    JuMP.@constraint(pm.model, es - energy == time_elapsed*(charge_eff*ps_ch - ps_dch/discharge_eff))
end

# State of charge of storage
function constraint_storage_state(pm::_PM.AbstractPowerModel, n::Int, i::Int, charge_eff, discharge_eff, time_elapsed)
    es = _PM.var(pm, n, :es, i)
    es_t_1 = _PM.var(pm, n-1, :es, i)
    ps_ch = _PM.var(pm, n, :ps_ch, i)
    ps_dch = _PM.var(pm, n, :ps_dch, i)

    JuMP.@constraint(pm.model, es - es_t_1 == time_elapsed*(charge_eff*ps_ch - ps_dch/discharge_eff))
end

function constraint_power_balance(pm::_PM.AbstractAPLossLessModels, n::Int, i::Int, bus_arcs, bus_gens, bus_loads, bus_storage)
    p    = _PM.var(pm, n, :p)
    pg   = _PM.var(pm, n,   :pg)
    ps   = _PM.var(pm, n,   :ps)
    pflex = _PM.var(pm, n, :pflex)


    cstr_p = JuMP.@constraint(pm.model,
        sum(p[a] for a in bus_arcs)
        ==
        sum(pg[g] for g in bus_gens)
        + sum(ps[s] for s in bus_storage) 
        - sum(pflex[d] for d in bus_loads)
    )

    if _IM.report_duals(pm)
        _PM.sol(pm, n, :bus, i)[:lam_kcl_r] = cstr_p
    end
end

function constraint_frequency(pm::_PM.AbstractPowerModel, n::Int, generator_properties, gcont, ΔT, f0, fmin)
    ΔPg = _PM.var(pm, 1, :pg)[gcont]
    alpha_g = _PM.var(pm, n, :alpha_g)
    print(gcont, "\n")
    cf = JuMP.@constraint(pm.model, sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (f0 - fmin)) >= f0 * ΔPg *  ΔT)
    print(cf, "\n")
end

function constraint_generator_status(pm::_PM.AbstractPowerModel, n::Int, i::Int)
    alpha_n = _PM.var(pm, n, :alpha_g, i)
    alpha_n_1 = _PM.var(pm, n-1, :alpha_g, i)

    JuMP.@constraint(pm.model, alpha_n == alpha_n_1)
end