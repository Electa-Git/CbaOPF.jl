# PST variables
function variable_pst(pm; kwargs...)
    variable_active_pst_flow(pm, kwargs...)
    variable_reactive_pst_flow(pm, kwargs...)
    variable_pst_angle(pm, kwargs...)
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