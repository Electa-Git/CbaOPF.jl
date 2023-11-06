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
    srated = pst["rate_a"]
    angmin = pst["angmin"]
    angmax = pst["angmax"]

    f_bus = pst["f_bus"]
    t_bus = pst["t_bus"]
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

    if _IM.report_duals(pm)
        _PM.sol(pm, n, :bus, i)[:lam_kcl_r] = cstr_p
    end
end

function constraint_total_flexible_demand(pm::_PM.AbstractDCPModel, n::Int, i, pd, pf_angle)
    pflex       = _PM.var(pm, n, :pflex, i)
    pcurt       = _PM.var(pm, n, :pcurt, i)
    pred        = _PM.var(pm, n, :pred, i)

    # Active power demand is the reference demand `pd` plus the contributions from all the demand flexibility decision variables
    JuMP.@constraint(pm.model, pflex == pd - pcurt - pred)
end


function constraint_fixed_xb_flows(pm::_PM.AbstractDCPModel, n::Int, xb_lines, xb_convs, flow, slack)
    p    = _PM.var(pm, n, :p)
    pconv = _PM.var(pm, n,  :pconv_tf_fr)

    # converter behaves like a load: 
    # pconv > 0 is an export
    # pconv < 0 is an import

    # flow > 0 means export
    # flow < 0 means import
    
    if flow > 0  # in case of import
        JuMP.@constraint(pm.model, sum(p[a] for a in xb_lines) - sum(pconv[c] for c in xb_convs) <= (1 + slack) * flow)
        JuMP.@constraint(pm.model,  (1 - slack) * flow <= sum(p[a] for a in xb_lines) - sum(pconv[c] for c in xb_convs))
    else
        JuMP.@constraint(pm.model, sum(p[a] for a in xb_lines) - sum(pconv[c] for c in xb_convs) >= (1 + slack) * flow)
        JuMP.@constraint(pm.model,  (1 - slack) * flow >= sum(p[a] for a in xb_lines) - sum(pconv[c] for c in xb_convs))
    end
end


function constraint_gen_redispatch(pm::_PM.AbstractDCPModel, n::Int, i, pg_ref)
    pg       = _PM.var(pm, n, :pg, i)
    dpg_up   = _PM.var(pm, n, :dpg_up, i)
    dpg_down = _PM.var(pm, n, :dpg_down, i)

    # Starting from the reference dispatch pg_ref, the new dispatch point is pg == pg_ref + dpg_up - dpg_down
    JuMP.@constraint(pm.model, pg == pg_ref + dpg_up - dpg_down)
end


function constraint_generator_on_off(pm::_PM.AbstractDCPModel, n::Int, i, pmax, pmin, status)
    pg = _PM.var(pm, n, :pg, i)
    alpha_g = _PM.var(pm, n, :alpha_g, i)

    JuMP.@constraint(pm.model,  pg <= pmax * alpha_g)
    JuMP.@constraint(pm.model,  pg >= pmin * alpha_g)
    JuMP.@constraint(pm.model,  alpha_g >= status)
end

function constraint_generator_on_off(pm::_PM.AbstractDCPModel, n::Int, nw_ref, i, pmax, pmin, status)
    pg = _PM.var(pm, n, :pg, i)
    alpha_g = _PM.var(pm, nw_ref, :alpha_g, i)

    JuMP.@constraint(pm.model,  pg <= pmax * alpha_g)
    JuMP.@constraint(pm.model,  pg >= pmin * alpha_g)
    JuMP.@constraint(pm.model,  alpha_g >= status)
end

function constraint_variable_branch_capacity_from(pm::_PM.AbstractDCPModel, n::Int, f_idx, pmax)
    l,i,j = f_idx
    p_fr = _PM.var(pm, n, :p, (l,i,j))
    delta_cap = _PM.var(pm, n, :delta_cap, l)

    JuMP.@constraint(pm.model, p_fr <=  (pmax + delta_cap))
end


function constraint_variable_branch_capacity_to(pm::_PM.AbstractDCPModel, n::Int, t_idx, pmax)
    l,i,j = t_idx
    p_to = _PM.var(pm, n, :p, (l,i,j))
    delta_cap = _PM.var(pm, n, :delta_cap, l)

    JuMP.@constraint(pm.model, p_to <=  (pmax + delta_cap))
end

function constraint_dc_branch_contingency(pm::_PM.AbstractPowerModel, n::Int, f_idx, t_idx)
    p_fr = _PM.var(pm, n, :p_dcgrid, f_idx)
    p_to = _PM.var(pm, n, :p_dcgrid, t_idx)

    JuMP.@constraint(pm.model, p_fr == 0)
    JuMP.@constraint(pm.model, p_to == 0)
end

function constraint_dc_conv_contingency(pm::_PM.AbstractPowerModel, n::Int, i)
    p_conv = _PM.var(pm, n, :pconv_tf_fr, i)
    p_conv_in = _PM.var(pm, n, :pconv_in, i)
    p_conv_abs = _PM.var(pm, n, :pconv_in_abs, i)
    p_dc = _PM.var(pm, n, :pconv_dc, i)

    JuMP.@constraint(pm.model, p_conv == 0)
    JuMP.@constraint(pm.model, p_conv_in == 0)
    JuMP.@constraint(pm.model, p_conv_abs == 0)
    JuMP.@constraint(pm.model, p_dc == 0)
end