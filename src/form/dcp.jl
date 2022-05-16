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

function constraint_power_balance_ac(pm::_PM.AbstractDCPModel, n::Int, i::Int, bus_arcs, bus_arcs_pst, bus_convs_ac, bus_arcs_sw, bus_gens, bus_storage, bus_pd, bus_qd, bus_gs, bus_bs)
    p    = _PM.get(_PM.var(pm, n),    :p, Dict()); _PM._check_var_keys(p, bus_arcs, "active power", "branch")
    q    = _PM.get(_PM.var(pm, n),    :q, Dict()); _PM._check_var_keys(q, bus_arcs, "reactive power", "branch")
    ppst    = _PM.get(_PM.var(pm, n),    :ppst, Dict()); _PM.check_var_keys(p, bus_arcs_pst, "active power", "pst")
    pg   = _PM.get(_PM.var(pm, n),   :pg, Dict()); _PM._check_var_keys(pg, bus_gens, "active power", "generator")
    qg   = _PM.get(_PM.var(pm, n),   :qg, Dict()); _PM._check_var_keys(qg, bus_gens, "reactive power", "generator")
    ps   = _PM.get(_PM.var(pm, n),   :ps, Dict()); _PM._check_var_keys(ps, bus_storage, "active power", "storage")
    qs   = _PM.get(_PM.var(pm, n),   :qs, Dict()); _PM._check_var_keys(qs, bus_storage, "reactive power", "storage")
    psw  = _PM.get(_PM.var(pm, n),  :psw, Dict()); _PM._check_var_keys(psw, bus_arcs_sw, "active power", "switch")
    qsw  =_PM. get(_PM.var(pm, n),  :qsw, Dict()); _PM._check_var_keys(qsw, bus_arcs_sw, "reactive power", "switch")
    pconv_grid_ac = _PM.var(pm, n,  :pconv_tf_fr)


    cstr_p = JuMP.@constraint(pm.model,
        sum(p[a] for a in bus_arcs)
        + sum(ppst[a] for a in bus_arcs_pst)
        + sum(pconv_grid_ac[c] for c in bus_convs_ac)
        + sum(psw[a_sw] for a_sw in bus_arcs_sw)
        ==
        sum(pg[g] for g in bus_gens)
        - sum(ps[s] for s in bus_storage)
        - sum(pd for (i,pd) in bus_pd)
        - sum(gs for (i,gs) in bus_gs)*1^2
    )
end