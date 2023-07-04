function constraint_power_balance_ac(pm::_PM.AbstractWModels, n::Int, i::Int, bus_arcs, bus_arcs_pst, bus_convs_ac, bus_arcs_sw, bus_gens, bus_storage, bus_loads, bus_gs, bus_bs)
    w   = _PM.var(pm, n, :w, i)
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
        - sum(gs for (i,gs) in bus_gs)*w^2
    )
    cstr_q = JuMP.@constraint(pm.model,
        sum(q[a] for a in bus_arcs)
        + sum(qpst[a] for a in bus_arcs_pst)
        + sum(qconv_grid_ac[c] for c in bus_convs_ac)
        ==
        sum(qg[g] for g in bus_gens)
        - sum(qflex[d] for d in bus_loads)
        + sum(bs for (i,bs) in bus_bs)*w^2
    )

    if _IM.report_duals(pm)
        _PM.sol(pm, n, :bus, i)[:lam_kcl_r] = cstr_p
        _PM.sol(pm, n, :bus, i)[:lam_kcl_i] = cstr_q
    end
end

function constraint_total_flexible_demand(pm::_PM.AbstractWModels, n::Int, i, pd, pf_angle)
    pflex       = _PM.var(pm, n, :pflex, i)
    qflex       = _PM.var(pm, n, :qflex, i)
    pcurt       = _PM.var(pm, n, :pcurt, i)
    pred        = _PM.var(pm, n, :pred, i)

    # Active power demand is the reference demand `pd` plus the contributions from all the demand flexibility decision variables
    JuMP.@constraint(pm.model, pflex == pd - pcurt - pred)

    # Reactive power demand is given by the active power demand and the power factor angle of the load
    JuMP.@constraint(pm.model, qflex == tan(pf_angle) * pflex)
end