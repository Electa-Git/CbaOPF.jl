##############################################
# PST Constraints
function constraint_ohms_y_from_pst(pm::_PM.AbstractDCPModel, n::Int, i::Int, f_bus, t_bus, f_idx, t_idx, g, b, g_fr, b_fr)
    alpha = _PM.var(pm, n,  :psta, i)
    p_fr  = _PM.var(pm, n,  :ppst, f_idx)
    vm = 1
    va_fr = _PM.var(pm, n, :va, f_bus)
    va_to = _PM.var(pm, n, :va, t_bus)

    JuMP.@constraint(pm.model, p_fr ==   -b * vm * (va_fr - va_to - alpha))
end

function constraint_ohms_y_to_pst(pm::_PM.AbstractDCPModel, n::Int, i::Int, f_bus, t_bus, f_idx, t_idx, g, b, g_to, b_to)
    alpha = _PM.var(pm, n,  :psta, i)
    p_to  = _PM.var(pm, n,  :ppst, t_idx)
    vm = 1
    va_fr = _PM.var(pm, n, :va, f_bus)
    va_to = _PM.var(pm, n, :va, t_bus)

    JuMP.@constraint(pm.model, p_to ==  -b * vm * (va_to - va_fr + alpha))
    end

function constraint_limits_pst(pm::_PM.AbstractDCPModel, i::Int; nw::Int=_PM.nw_id_default)
    pst = _PM.ref(pm, nw, :pst, i)
    srated = pst["rateA"]
    angmin = pst["angmin"]
    angmax = pst["angmax"]

    f_bus = pst["fbus"]
    t_bus = pst["tbus"]
    f_idx = (i, f_bus, t_bus)
    t_idx = (i, t_bus, f_bus)

    alpha = _PM.var(pm, nw,  :psta, i)
    p_fr  = _PM.var(pm, nw,  :ppst, f_idx)
    p_to  = _PM.var(pm, nw,  :ppst, t_idx)

    JuMP.@constraint(pm.model, -srated <= p_fr <= srated)
    JuMP.@constraint(pm.model, -srated <= p_to <= srated)
    JuMP.@constraint(pm.model, alpha <= angmax)
    JuMP.@constraint(pm.model, alpha >= angmin)
end

function constraint_power_balance_ac(pm::_PM.AbstractDCPModel, n::Int, i::Int, bus_arcs, bus_arcs_pst, bus_convs_ac, bus_arcs_sw, bus_gens, bus_storage, bus_loads, bus_gs, bus_bs)
    p    = _PM.var(pm, n, :p)
    ppst    = _PM.var(pm, n,    :ppst)
    pg   = _PM.var(pm, n,   :pg)
    pconv_grid_ac = _PM.var(pm, n,  :pconv_tf_fr)
    pflex = _PM.var(pm, n, :pflex)


    cstr_p = JuMP.@constraint(pm.model,
        sum(p[a] for a in bus_arcs)
        + sum(ppst[a] for a in bus_arcs_pst)
        + sum(pconv_grid_ac[c] for c in bus_convs_ac)
        ==
        sum(pg[g] for g in bus_gens)
        - sum(pflex[d] for d in bus_loads)
        - sum(gs for (i,gs) in bus_gs)*1^2
    )
end

function constraint_total_flexible_demand(pm::_PM.AbstractDCPModel, n::Int, i, pd, pf_angle)
    pflex       = _PM.var(pm, n, :pflex, i)
    pcurt       = _PM.var(pm, n, :pcurt, i)
    pred        = _PM.var(pm, n, :pred, i)

    # Active power demand is the reference demand `pd` plus the contributions from all the demand flexibility decision variables
    JuMP.@constraint(pm.model, pflex == pd - pcurt - pred)
end