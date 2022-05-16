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
                JuMP.set_lower_bound(p[arc], -_PM.ref(pm, nw, :pst, l)["rateA"])
                JuMP.set_upper_bound(p[arc], _PM.ref(pm, nw, :pst, l)["rateA"])
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
                JuMP.set_lower_bound(q[arc], -_PM.ref(pm, nw, :pst, l)["rateA"])
                JuMP.set_upper_bound(q[arc], _PM.ref(pm, nw, :pst, l)["rateA"])
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
    _FP.variable_total_flex_demand(pm; kwargs...)
    _FP.variable_demand_reduction(pm; kwargs...)
    _FP.variable_demand_curtailment(pm; kwargs...)
end