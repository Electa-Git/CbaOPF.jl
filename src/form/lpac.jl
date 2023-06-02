function constraint_power_balance_ac(pm::_PM.AbstractLPACModel, n::Int, i::Int, bus_arcs, bus_arcs_pst, bus_convs_ac, bus_arcs_sw, bus_gens, bus_storage, bus_loads, bus_gs, bus_bs)
    phi = _PM.var(pm, n, :phi, i)
    p    = _PM.var(pm, n, :p)
    q    = _PM.var(pm, n, :q)
    ppst    = _PM.var(pm, n,    :ppst)
    qpst    = _PM.var(pm, n,    :qpst)
    pg   = _PM.var(pm, n,   :pg)
    qg   = _PM.var(pm, n,   :qg)
    pconv_grid_ac = _PM.var(pm, n,  :pconv_tf_fr)
    qconv_grid_ac = _PM.var(pm, n,  :qconv_tf_fr)
    pflex = _PM.var(pm, n, :pflex)
    qflex = _PM.var(pm, n, :qflex)


    cstr_p = JuMP.@constraint(pm.model, 
    sum(p[a] for a in bus_arcs)
    + sum(ppst[a] for a in bus_arcs_pst)
    + sum(pconv_grid_ac[c] for c in bus_convs_ac)
    ==
    sum(pg[g] for g in bus_gens)
    - sum(pflex[d] for d in bus_loads)
    - sum(gs for (i,gs) in bus_gs)*(1.0 + 2*phi))

    cstr_q = JuMP.@constraint(pm.model, 
    sum(q[a] for a in bus_arcs)
    + sum(qpst[a] for a in bus_arcs_pst)
    + sum(qconv_grid_ac[c] for c in bus_convs_ac)
    ==
    sum(qg[g] for g in bus_gens)
    - sum(qflex[d] for d in bus_loads)
    + sum(bs for (i,bs) in bus_bs)*(1.0 + 2*phi))
end

function constraint_total_flexible_demand(pm::_PM.AbstractLPACModel, n::Int, i, pd, pf_angle)
    pflex       = _PM.var(pm, n, :pflex, i)
    qflex       = _PM.var(pm, n, :qflex, i)
    pcurt       = _PM.var(pm, n, :pcurt, i)
    pred        = _PM.var(pm, n, :pred, i)

    # Active power demand is the reference demand `pd` plus the contributions from all the demand flexibility decision variables
    JuMP.@constraint(pm.model, pflex == pd - pcurt - pred)

    # Reactive power demand is given by the active power demand and the power factor angle of the load
    JuMP.@constraint(pm.model, qflex == tan(pf_angle) * pflex)
end

# To Do: PST constraints for LPAC.
function constraint_ohms_y_from_pst(pm::_PM.AbstractLPACModel, n::Int, i::Int, f_bus, t_bus, f_idx, t_idx, g, b, g_fr, b_fr)
    alpha = _PM.var(pm, n,  :psta, i)
    p_fr  = _PM.var(pm, n,  :ppst, f_idx)
    q_fr  = _PM.var(pm, n,  :qpst, f_idx)

    phi_fr = _PM.var(pm, n, :phi, f_bus)
    phi_to = _PM.var(pm, n, :phi, t_bus)
    va_fr = _PM.var(pm, n, :va, f_bus)
    va_to = _PM.var(pm, n, :va, t_bus)
    cs = _PM.var(pm, n, :cs_pst, (f_bus, t_bus))

    JuMP.@constraint(pm.model, p_fr ==  g * (1.0 + 2*phi_fr) - g * (cs + phi_fr + phi_to) - b * (va_fr - va_to - alpha))
    JuMP.@constraint(pm.model, q_fr == -b * (1.0 + 2*phi_fr) + b * (cs + phi_fr + phi_to) - g * (va_fr - va_to - alpha))
end

function constraint_ohms_y_to_pst(pm::_PM.AbstractLPACModel, n::Int, i::Int, f_bus, t_bus, f_idx, t_idx, g, b, g_fr, b_fr)
    alpha = _PM.var(pm, n,  :psta, i)
    p_to  = _PM.var(pm, n,  :ppst, t_idx)
    q_to  = _PM.var(pm, n,  :qpst, t_idx)

    phi_fr = _PM.var(pm, n, :phi, f_bus)
    phi_to = _PM.var(pm, n, :phi, t_bus)
    va_fr = _PM.var(pm, n, :va, f_bus)
    va_to = _PM.var(pm, n, :va, t_bus)
    cs = _PM.var(pm, n, :cs_pst, (f_bus, t_bus))

    JuMP.@constraint(pm.model, p_to ==  g * (1.0 + 2 * phi_to) - g * (cs + phi_fr + phi_to) -b * (va_to - va_fr + alpha))
    JuMP.@constraint(pm.model, q_to == -b * (1.0 + 2 * phi_to) + b * (cs + phi_fr + phi_to) -g * (va_to - va_fr + alpha))
end



