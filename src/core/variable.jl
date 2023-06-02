# PST variables
function variable_pst(pm; kwargs...)
    variable_active_pst_flow(pm, kwargs...)
    variable_reactive_pst_flow(pm, kwargs...)
    variable_pst_angle(pm, kwargs...)
    variable_pst_cosine(pm, kwargs...)
end

"variable: `p[l,i,j]` for `(l,i,j)` in `arcs`"
function variable_active_pst_flow(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    p = _PM.var(pm, nw)[:ppst] = JuMP.@variable(pm.model,
        [(l,i,j) in _PM.ref(pm, nw, :arcs_pst)], base_name="$(nw)_ppst",
        start = 0
    )

    if bounded
        for arc in _PM.ref(pm, nw, :arcs_pst)
            l,i,j = arc
                JuMP.set_lower_bound(p[arc], -_PM.ref(pm, nw, :pst, l)["rate_a"])
                JuMP.set_upper_bound(p[arc], _PM.ref(pm, nw, :pst, l)["rate_a"])
        end
    end

    report && _PM.sol_component_value_edge(pm, nw, :pst, :pf, :pt, _PM.ref(pm, nw, :arcs_from_pst), _PM.ref(pm, nw, :arcs_to_pst), p)
end

"variable: `q[l,i,j]` for `(l,i,j)` in `arcs`"
function variable_reactive_pst_flow(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    q = _PM.var(pm, nw)[:qpst] = JuMP.@variable(pm.model,
        [(l,i,j) in _PM.ref(pm, nw, :arcs_pst)], base_name="$(nw)_qpst",
        start = 0
    )

    if bounded
        for arc in _PM.ref(pm, nw, :arcs_pst)
            l,i,j = arc
                JuMP.set_lower_bound(q[arc], -_PM.ref(pm, nw, :pst, l)["rate_a"])
                JuMP.set_upper_bound(q[arc], _PM.ref(pm, nw, :pst, l)["rate_a"])
        end
    end

    report && _PM.sol_component_value_edge(pm, nw, :pst, :qf, :qt, _PM.ref(pm, nw, :arcs_from_pst), _PM.ref(pm, nw, :arcs_to_pst), q)
end

"variable: `t[i]` for `i` in `bus`es"
function variable_pst_angle(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    alpha = _PM.var(pm, nw)[:psta] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :pst)], base_name="$(nw)_psta",
        start = 0
    )
    if bounded
        for (i, pst) in _PM.ref(pm, nw, :pst)
            JuMP.set_lower_bound(alpha[i], pst["angmin"])
            JuMP.set_upper_bound(alpha[i], pst["angmax"])
        end
    end
    report && _PM.sol_component_value(pm, nw, :pst, :alpha, _PM.ids(pm, nw, :pst), alpha)
end

function variable_pst_cosine(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
end

function variable_pst_cosine(pm::_PM.LPACCPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    cs_pst = _PM.var(pm, nw)[:cs_pst] = JuMP.@variable(pm.model,
        [bp in _PM.ids(pm, nw, :buspairs_pst)], base_name="$(nw)_cs",
        start = _PM.comp_start_value(_PM.ref(pm, nw, :buspairs_pst, bp), "cs_start", 1.0)
    )

    if bounded
        for (bp, buspair) in _PM.ref(pm, nw, :buspairs_pst)
            angmin = buspair["angmin"]
            angmax = buspair["angmax"]
            if angmin >= 0
                cos_max = cos(angmin)
                cos_min = cos(angmax)
            end
            if angmax <= 0
                cos_max = cos(angmax)
                cos_min = cos(angmin)
            end
            if angmin < 0 && angmax > 0
                cos_max = 1.0
                cos_min = min(cos(angmin), cos(angmax))
            end

            JuMP.set_lower_bound(cs_pst[bp], cos_min)
            JuMP.set_upper_bound(cs_pst[bp], cos_max)
        end
    end

    report && _PM.sol_component_value_buspair(pm, nw, :buspairs_pst, :cs_pst, _PM.ids(pm, nw, :buspairs_pst), cs_pst)
end





function variable_flexible_demand(pm::_PM.AbstractPowerModel; kwargs...)
    variable_total_flex_demand(pm; kwargs...)
    variable_demand_reduction(pm; kwargs...)
    variable_demand_curtailment(pm; kwargs...)
end


function variable_total_flex_demand(pm::_PM.AbstractPowerModel; kwargs...)
    variable_total_flex_demand_active(pm; kwargs...)
    variable_total_flex_demand_reactive(pm; kwargs...)
end

"Variable for the actual (flexible) real load demand at each load point and each time step"
function variable_total_flex_demand_active(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    pflex = _PM.var(pm, nw)[:pflex] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :load)], base_name="$(nw)_pflex",
        lower_bound = min(0, _PM.ref(pm, nw, :load, i, "pd")),
        upper_bound = max(0, _PM.ref(pm, nw, :load, i, "pd")),
        start = _PM.comp_start_value(_PM.ref(pm, nw, :load, i), "pd")
    )
    report && _PM.sol_component_value(pm, nw, :load, :pflex, _PM.ids(pm, nw, :load), pflex)
end

function variable_total_flex_demand_reactive(pm::_PM.AbstractActivePowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
end

"Variable for the actual (flexible) reactive load demand at each load point and each time step"
function variable_total_flex_demand_reactive(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    qflex = _PM.var(pm, nw)[:qflex] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :load)], base_name="$(nw)_qflex",
        start = _PM.comp_start_value(_PM.ref(pm, nw, :load, i), "qd")
    )
    report && _PM.sol_component_value(pm, nw, :load, :qflex, _PM.ids(pm, nw, :load), qflex)
end


"Variable for the power not consumed (voluntary load reduction) at each flex load point and each time step"
function variable_demand_reduction(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    pred = _PM.var(pm, nw)[:pred] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :flex_load)], base_name="$(nw)_pred",
        lower_bound = 0,
        upper_bound = max(0, _PM.ref(pm, nw, :load, i, "pd")) * _PM.ref(pm, nw, :flex_load, i, "pred_rel_max"),
        start = 0
    )
    if report
        _PM.sol_component_value(pm, nw, :load, :pred, _PM.ids(pm, nw, :flex_load), pred)
    end
end

"Variable for load curtailment (i.e. involuntary demand reduction) at each load point and each time step"
function variable_demand_curtailment(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    pcurt = _PM.var(pm, nw)[:pcurt] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :load)], base_name="$(nw)_pcurt",
        lower_bound = 0,
        upper_bound = max(0, _PM.ref(pm, nw, :load, i, "pd")),
        start = 0
    )
    report && _PM.sol_component_value(pm, nw, :load, :pcurt, _PM.ids(pm, nw, :load), pcurt)
end

"Collect redispatch variables"
function variable_gen_redispatch(pm; kwargs...)
    variable_redispacth_up(pm, kwargs...)
    variable_redispacth_down(pm, kwargs...)
end

"Variable for upwards generator redispatch each time step"
function variable_redispacth_up(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    dpg_up = _PM.var(pm, nw)[:dpg_up] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :gen)], base_name="$(nw)_dpg_up",
        lower_bound = 0,
        upper_bound = 2 * max(0, _PM.ref(pm, nw, :gen, i, "pmax")),
        start = 0
    )
    report && _PM.sol_component_value(pm, nw, :gen, :dpg_up, _PM.ids(pm, nw, :gen), dpg_up)
end

"Variable for upwards generator redispatch each time step"
function variable_redispacth_down(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    dpg_down = _PM.var(pm, nw)[:dpg_down] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :gen)], base_name="$(nw)_dpg_down",
        lower_bound = 0,
        upper_bound = 2 * max(0, _PM.ref(pm, nw, :gen, i, "pmax")),
        start = 0
    )
    report && _PM.sol_component_value(pm, nw, :gen, :dpg_down, _PM.ids(pm, nw, :gen), dpg_down)
end

"Variable for upwards generator redispatch each time step"
function variable_generator_state(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    alpha_g = _PM.var(pm, nw)[:alpha_g] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :gen)], base_name="$(nw)_alpha_g",
        binary = true,
        start = 0
    )
    report && _PM.sol_component_value(pm, nw, :gen, :alpha_g, _PM.ids(pm, nw, :gen), alpha_g)
end

"Variable for the variable NTC capacity"
function variable_branch_capacity(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    delta_cap = _PM.var(pm, nw)[:delta_cap] = JuMP.@variable(pm.model,
        [b in _PM.ids(pm, nw, :branch)], base_name="$(nw)_delta_cap",
        start = 0
    )

    if bounded
        for (b, branch) in _PM.ref(pm, nw, :branch)
            JuMP.set_lower_bound(delta_cap[b], 0)
            JuMP.set_upper_bound(delta_cap[b], branch["delta_cap_max"])
        end
    end

    report && _PM.sol_component_value(pm, nw, :branch, :delta_cap, _PM.ids(pm, nw, :branch), delta_cap)
end

function variable_storage(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default)
    variable_storage_charging_power(pm, nw = nw)
    variable_storage_discharging_power(pm, nw = nw)
    variable_storage_net_power(pm, nw = nw)
    variable_storage_energy_content(pm, nw = nw)
end


"Variable for storage charging"
function variable_storage_charging_power(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    ps_ch = _PM.var(pm, nw)[:ps_ch] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :storage_simple)], base_name="$(nw)_ps_ch",
        lower_bound = 0,
        upper_bound = max(0, _PM.ref(pm, nw, :storage_simple, i, "charge_rating")),
        start = 0
    )
    report && _PM.sol_component_value(pm, nw, :storage_simple, :ps_ch, _PM.ids(pm, nw, :storage_simple), ps_ch)
end

"Variable for storage discharging"
function variable_storage_discharging_power(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    ps_dch = _PM.var(pm, nw)[:ps_dch] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :storage_simple)], base_name="$(nw)_ps_dch",
        lower_bound = 0,
        upper_bound = max(0, _PM.ref(pm, nw, :storage_simple, i, "discharge_rating")),
        start = 0
    )
    report && _PM.sol_component_value(pm, nw, :storage_simple, :ps_dch, _PM.ids(pm, nw, :storage_simple), ps_dch)
end

"Variable for net storage injection"
function variable_storage_net_power(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    ps = _PM.var(pm, nw)[:ps] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :storage_simple)], base_name="$(nw)_ps",
        lower_bound = -max(0, _PM.ref(pm, nw, :storage_simple, i, "charge_rating")),
        upper_bound =  max(0, _PM.ref(pm, nw, :storage_simple, i, "discharge_rating")),
        start = 0
    )
    report && _PM.sol_component_value(pm, nw, :storage_simple, :ps, _PM.ids(pm, nw, :storage_simple), ps)
end


"Variable storage energy content"
function variable_storage_energy_content(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    es = _PM.var(pm, nw)[:es] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :storage_simple)], base_name="$(nw)_es",
    lower_bound = 0,
    upper_bound = max(0, _PM.ref(pm, nw, :storage_simple, i, "energy_rating")),
    start = _PM.ref(pm, nw, :storage_simple, i, "energy")
    )

    report && _PM.sol_component_value(pm, nw, :storage_simple, :es, _PM.ids(pm, nw, :storage_simple), es)
end