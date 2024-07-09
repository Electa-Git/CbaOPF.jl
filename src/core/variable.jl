# PST variables
function variable_pst(pm::_PM.AbstractPowerModel; kwargs...)
    variable_active_pst_flow(pm; kwargs...)
    variable_reactive_pst_flow(pm; kwargs...)
    variable_pst_angle(pm; kwargs...)
    variable_pst_cosine(pm; kwargs...)
end

"variable: `p[l,i,j]` for `(l,i,j)` in `pst_arcs`"
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

"variable: `q[l,i,j]` for `(l,i,j)` in `pst_arcs`"
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

"variable: psta[i] for PSTs"
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
        start = _PM.comp_start_value(_PM.ref(pm, nw, :load, i), "qd"),
        lower_bound = -max(0, _PM.ref(pm, nw, :load, i, "qd")),
        upper_bound =  max(0, _PM.ref(pm, nw, :load, i, "qd")),
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
    variable_redispatch_up(pm; kwargs...)
    variable_redispatch_down(pm; kwargs...)
end

"Variable for upwards generator redispatch each time step"
function variable_redispatch_up(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    dpg_up = _PM.var(pm, nw)[:dpg_up] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :gen)], base_name="$(nw)_dpg_up",
        lower_bound = 0,
        upper_bound = 2 * max(0, _PM.ref(pm, nw, :gen, i, "pmax")),
        start = 0
    )
    report && _PM.sol_component_value(pm, nw, :gen, :dpg_up, _PM.ids(pm, nw, :gen), dpg_up)
end

"Variable for upwards generator redispatch each time step"
function variable_redispatch_down(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    dpg_down = _PM.var(pm, nw)[:dpg_down] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :gen)], base_name="$(nw)_dpg_down",
        lower_bound = 0,
        upper_bound = 2 * max(0, _PM.ref(pm, nw, :gen, i, "pmax")),
        start = 0
    )
    report && _PM.sol_component_value(pm, nw, :gen, :dpg_down, _PM.ids(pm, nw, :gen), dpg_down)
end

function variable_generator_states(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, uc = false, res_on = false, all_on = false)
    variable_generator_state(pm, nw = nw; res_on = res_on, all_on = all_on)
    if uc == true
        if haskey(pm.setting, "relax_uc_binaries") && pm.setting["relax_uc_binaries"] == true
            variable_generator_state_mut_relax(pm, nw = nw)
            variable_generator_state_mdt_relax(pm, nw = nw)
        else
            variable_generator_state_mut(pm, nw = nw)
            variable_generator_state_mdt(pm, nw = nw)
        end
    end
end



"Variable for generator state"
function variable_generator_state(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true, res_on = false, all_on = false)
    alpha_g = _PM.var(pm, nw)[:alpha_g] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :gen)], base_name="$(nw)_alpha_g",
        binary = true,
        start = 0,
        upper_bound = 1,
        lower_bound = 0,
    )
    
    for (g, gen) in _PM.ref(pm, nw, :gen)
        if res_on == true && (gen["type"] == "Wind" || gen["type"] == "Solar")
            JuMP.set_lower_bound(alpha_g[g], 1)
            JuMP.set_upper_bound(alpha_g[g], 1)
        elseif gen["pmax"] == 0
            JuMP.set_lower_bound(alpha_g[g], 0)
            JuMP.set_upper_bound(alpha_g[g], 0)
        # elseif gen["mut"] == 1 && gen["mdt"] == 1 && gen["cost"][2] == 0.0 && gen["start_up_cost"] == 0.0 # generators without any auxiliary cost & start-up cost can always be online
        #     JuMP.set_lower_bound(alpha_g[g], 1)
        #     JuMP.set_upper_bound(alpha_g[g], 1)
        end
        if all_on == true
            JuMP.set_lower_bound(alpha_g[g], 1)
            JuMP.set_upper_bound(alpha_g[g], 1)
        end
    end
    report && _PM.sol_component_value(pm, nw, :gen, :alpha_g, _PM.ids(pm, nw, :gen), alpha_g)
end

"Variable for minimum up-time"
function variable_generator_state_mut(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    beta_g = _PM.var(pm, nw)[:beta_g] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :gen)], base_name="$(nw)_beta_g",
        binary = true,
        start = 0,
        lower_bound = 0,
        upper_bound = 1
    )
    report && _PM.sol_component_value(pm, nw, :gen, :beta_g, _PM.ids(pm, nw, :gen), beta_g)
end

"Variable for minimum down-time"
function variable_generator_state_mdt(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    gamma_g = _PM.var(pm, nw)[:gamma_g] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :gen)], base_name="$(nw)_gamma_g",
        binary = true,
        start = 0,
        lower_bound = 0,
        upper_bound = 1
    )
    report && _PM.sol_component_value(pm, nw, :gen, :gamma_g, _PM.ids(pm, nw, :gen), gamma_g)
end

"Variable for minimum up-time relaxed"
function variable_generator_state_mut_relax(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    beta_g = _PM.var(pm, nw)[:beta_g] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :gen)], base_name="$(nw)_beta_g",
        binary = false,
        lower_bound = 0,
        upper_bound = 1,
        start = 0
    )
    report && _PM.sol_component_value(pm, nw, :gen, :beta_g, _PM.ids(pm, nw, :gen), beta_g)
end

"Variable for minimum down-time relaxed"
function variable_generator_state_mdt_relax(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    gamma_g = _PM.var(pm, nw)[:gamma_g] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :gen)], base_name="$(nw)_gamma_g",
        binary = false,
        lower_bound = 0,
        upper_bound = 1,
        start = 0
    )
    report && _PM.sol_component_value(pm, nw, :gen, :gamma_g, _PM.ids(pm, nw, :gen), gamma_g)
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

"Variable storage on/off state"
function variable_storage_on_off(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    alpha_s = _PM.var(pm, nw)[:alpha_s] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :storage)], base_name="$(nw)_alpha_s",
    binary = true
    )

    report && _PM.sol_component_value(pm, nw, :storage, :alpha_s, _PM.ids(pm, nw, :storage), alpha_s)
end

function variable_hvdc_contribution(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default)
    variable_converter_inertia(pm, nw = nw)
    variable_converter_inertia_abs(pm, nw = nw)
    variable_total_hvdc_inertia(pm, nw = nw)
    variable_total_hvdc_inertia_tie_line(pm, nw = nw)
end

"Variable to model the power change of HVDC converter to provide inertia"
function variable_converter_inertia(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    Δpconv = _PM.var(pm, nw)[:pconv_in] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :convdc)], base_name="$(nw)_pconv_in",
    start = _PM.comp_start_value(_PM.ref(pm, nw, :convdc, i), "P_g", 0.0)
    )

    if bounded
        for (c, convdc) in _PM.ref(pm, nw, :convdc)
            JuMP.set_lower_bound(Δpconv[c],  -2 * convdc["Pacrated"])
            JuMP.set_upper_bound(Δpconv[c],   2 * convdc["Pacrated"])
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :convdc, :pconv_in, _PM.ids(pm, nw, :convdc), Δpconv)
end

"Variable to model the converter ramp rate"
function variable_converter_droop(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    rdc = _PM.var(pm, nw)[:rdc] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :convdc)], base_name="$(nw)_rdc",
    start = 0
    )

    if bounded
        for (c, convdc) in _PM.ref(pm, nw, :convdc)
            JuMP.set_lower_bound(rdc[c],  -convdc["rmax"])
            JuMP.set_upper_bound(rdc[c],   convdc["rmax"])
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :convdc, :rdc, _PM.ids(pm, nw, :convdc), rdc)
end

"Variable to represent absolute value HVDC converter inertia provision for the objective"
function variable_converter_inertia_abs(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    Δpconv_abs = _PM.var(pm, nw)[:pconv_in_abs] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :convdc)], base_name="$(nw)_pconv_in_abs",
    start = _PM.comp_start_value(_PM.ref(pm, nw, :convdc, i), "P_g", 1.0)
    )

    if bounded
        for (c, convdc) in _PM.ref(pm, nw, :convdc)
            JuMP.set_lower_bound(Δpconv_abs[c],  0)
            JuMP.set_upper_bound(Δpconv_abs[c],  2 * convdc["Pacrated"])
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :convdc, :pconv_in_abs, _PM.ids(pm, nw, :convdc), Δpconv_abs)
end

function variable_gen_contribution(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default)
    variable_gen_droop(pm, nw = nw)
    variable_gen_droop_abs(pm, nw = nw)
    # variable_total_gen_droop(pm, nw = nw)
    # variable_total_gen_droop_tie_line(pm, nw = nw)
end

function variable_gen_droop(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    pg_droop = _PM.var(pm, nw)[:pg_droop] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :gen)], base_name="$(nw)_pg_droop",
    start = 0
    )

    if bounded
        for (g, gen) in _PM.ref(pm, nw, :gen)
            if gen["fcr_contribution"] == true
                JuMP.set_lower_bound(pg_droop[g],  - 2 * gen["pmax"])
                JuMP.set_upper_bound(pg_droop[g],    2 * gen["pmax"])
            else
                JuMP.set_lower_bound(pg_droop[g],  0)
                JuMP.set_upper_bound(pg_droop[g],  0)
            end
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :gen, :pg_droop, _PM.ids(pm, nw, :gen), pg_droop)
end

function variable_gen_droop_abs(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    Δpg_abs = _PM.var(pm, nw)[:pg_droop_abs] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :gen)], base_name="$(nw)_pg_droop_abs",
    start = 0
    )

    if bounded
        for (g, gen) in _PM.ref(pm, nw, :gen)
            JuMP.set_lower_bound(Δpg_abs[g],  0)
            JuMP.set_upper_bound(Δpg_abs[g],  2 * gen["pmax"])
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :gen, :pg_droop_abs, _PM.ids(pm, nw, :gen), Δpg_abs)
end

function variable_storage_contribution(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default)
    variable_storage_droop(pm, nw = nw)
    variable_storage_droop_abs(pm, nw = nw)
end

function variable_storage_droop(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    ps_droop = _PM.var(pm, nw)[:ps_droop] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :storage)], base_name="$(nw)_ps_droop",
    start = 0
    )

    if bounded
        for (s, storage) in _PM.ref(pm, nw, :storage)
            if storage["fcr_contribution"] == true
                JuMP.set_lower_bound(ps_droop[s],  - 2 * storage["discharge_rating"])
                JuMP.set_upper_bound(ps_droop[s],    2 * storage["discharge_rating"])
            else
                JuMP.set_lower_bound(ps_droop[s],  0)
                JuMP.set_upper_bound(ps_droop[s],  0)
            end
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :storage, :ps_droop, _PM.ids(pm, nw, :storage), ps_droop)
end

function variable_storage_droop_abs(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    Δps_abs = _PM.var(pm, nw)[:ps_droop_abs] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :storage)], base_name="$(nw)_ps_droop_abs",
    start = 0
    )

    if bounded
        for (s, storage) in _PM.ref(pm, nw, :storage)
            JuMP.set_lower_bound(Δps_abs[s],  0)
            JuMP.set_upper_bound(Δps_abs[s],  2 * storage["discharge_rating"])
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :storage, :ps_droop_abs, _PM.ids(pm, nw, :storage), Δps_abs)
end



function variable_inertia(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default)
    variable_inertia_gen(pm, nw = nw)
    variable_inertia_tie_line(pm, nw = nw)
end


"Variable to inspect total inertia"
function variable_inertia_gen(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    htot = _PM.var(pm, nw)[:htot] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :zones)], base_name="$(nw)_htot",
    start = 0.0
    )

    if bounded
        for (z, zone) in _PM.ref(pm, nw, :zones)
            JuMP.set_lower_bound(htot[z],   0.0)
            JuMP.set_upper_bound(htot[z],   1e6)
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :zones, :htot, _PM.ids(pm, nw, :zones), htot)
end

"Variable to inspect total inertia"
function variable_inertia_tie_line(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    htot_area = _PM.var(pm, nw)[:htot_area] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :areas)], base_name="$(nw)_htot_area",
    start = 0.0
    )

    if bounded
        for (a, area) in _PM.ref(pm, nw, :areas)
            JuMP.set_lower_bound(htot_area[a],   0.0)
            JuMP.set_upper_bound(htot_area[a],   1e6)
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :areas, :htot, _PM.ids(pm, nw, :areas), htot_area)
end

"Variable to inspect total inertia"
function variable_total_hvdc_inertia(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    dc_contr = _PM.var(pm, nw)[:dc_contr] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :zones)], base_name="$(nw)_dc_contr",
    start = 0.0
    )

    if bounded
        for (z, zone) in _PM.ref(pm, nw, :zones)
            JuMP.set_lower_bound(dc_contr[z], -1e6)
            JuMP.set_upper_bound(dc_contr[z],  1e6)
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :zones, :dc_contr, _PM.ids(pm, nw, :zones), dc_contr)
end

"Variable to inspect total inertia"
function variable_total_hvdc_inertia_tie_line(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    dc_contr_area = _PM.var(pm, nw)[:dc_contr_area] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :areas)], base_name="$(nw)_dc_contr_area",
    start = 0.0
    )

    if bounded
        for (a, area) in _PM.ref(pm, nw, :areas)
            JuMP.set_lower_bound(dc_contr_area[a], -1e6)
            JuMP.set_upper_bound(dc_contr_area[a],  1e6)
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :areas, :dc_contr, _PM.ids(pm, nw, :areas), dc_contr_area)
end



# "Variable for generator state"
# function variable_generator_droop(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
#     rg = _PM.var(pm, nw)[:rg] = JuMP.@variable(pm.model,
#         [i in _PM.ids(pm, nw, :gen)], base_name="$(nw)_rg",
#         start = 0
#     )

#     if bounded
#         for (g, gen) in _PM.ref(pm, nw, :gen)
#             JuMP.set_lower_bound(rg[g], 0)
#             JuMP.set_upper_bound(rg[g], gen["ramp_rate"] / (3600))
#         end
#     end

#     report && _PM.sol_component_value(pm, nw, :gen, :rg, _PM.ids(pm, nw, :gen), rg)
# end

function variable_contingencies(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default)
    variable_generator_contingency(pm, nw = nw)
    variable_generator_contingency_indicator(pm, nw = nw)
    variable_converter_contingency(pm, nw = nw)
    variable_converter_contingency_indicator(pm, nw = nw)
    variable_tieline_contingency(pm, nw = nw)
    variable_tieline_contingency_indicator(pm, nw = nw)
    variable_storage_contingency(pm, nw = nw)
    variable_storage_contingency_indicator(pm, nw = nw)
end

function variable_generator_contingency(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    δPg = _PM.var(pm, nw)[:gen_cont] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :zones)], base_name="$(nw)_gen_cont",
    start = 0.0
    )

    if bounded
        pg_max = maximum([gen["pmax"] for (g, gen) in _PM.ref(pm, nw, :gen)])
        for (z, zone) in _PM.ref(pm, nw, :zones)
            JuMP.set_lower_bound(δPg[z], 0)
            JuMP.set_upper_bound(δPg[z],  pg_max)
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :contingency, :gen_cont, _PM.ids(pm, nw, :zones), δPg)
end

function variable_generator_contingency_indicator(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    delta_g = _PM.var(pm, nw)[:delta_g] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :gen)], base_name="$(nw)_delta_g",
        binary = true,
        start = 0,
        lower_bound = 0,
        upper_bound = 1
    )
    report && _PM.sol_component_value(pm, nw, :gen, :delta_g, _PM.ids(pm, nw, :gen), delta_g)
end

function variable_converter_contingency(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    δPc_plus = _PM.var(pm, nw)[:conv_cont_plus] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :zones)], base_name="$(nw)_conv_cont_plus",
    start = 0.0
    )

    δPc_minus = _PM.var(pm, nw)[:conv_cont_minus] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :zones)], base_name="$(nw)_conv_cont_minus",
    start = 0.0
    )

    if bounded
        pc_max = maximum([conv["Pacrated"] for (c, conv) in _PM.ref(pm, nw, :convdc)])
        for (z, zone) in _PM.ref(pm, nw, :zones)
            JuMP.set_lower_bound(δPc_plus[z], -pc_max)
            JuMP.set_lower_bound(δPc_minus[z], -pc_max)
            JuMP.set_upper_bound(δPc_plus[z],  pc_max)
            JuMP.set_upper_bound(δPc_minus[z],  0)
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :contingency, :conv_cont_plus, _PM.ids(pm, nw, :zones), δPc_plus)
    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :contingency, :conv_cont_minus, _PM.ids(pm, nw, :zones), δPc_minus)
end

function variable_converter_contingency_indicator(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    delta_c_plus = _PM.var(pm, nw)[:delta_c_plus] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :convdc)], base_name="$(nw)_delta_c_plus",
        binary = true,
        start = 0,
        lower_bound = 0,
        upper_bound = 1
    )

    delta_c_minus = _PM.var(pm, nw)[:delta_c_minus] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :convdc)], base_name="$(nw)_delta_c_minus",
    binary = true,
    start = 0,
    lower_bound = 0,
    upper_bound = 1
    )

    report && _PM.sol_component_value(pm, nw, :convdc, :delta_c_plus, _PM.ids(pm, nw, :convdc), delta_c_plus)
    report && _PM.sol_component_value(pm, nw, :convdc, :delta_c_minus, _PM.ids(pm, nw, :convdc), delta_c_minus)
end

function variable_storage_contingency(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    δPs = _PM.var(pm, nw)[:storage_cont] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :zones)], base_name="$(nw)_storage_cont",
    start = 0.0
    )

    if bounded
        if !isempty([storage["thermal_rating"] for (s, storage) in _PM.ref(pm, nw, :storage)])
            ps_max = maximum([storage["thermal_rating"] for (s, storage) in _PM.ref(pm, nw, :storage)])
        else
            ps_max = 0
        end
        for (z, zone) in _PM.ref(pm, nw, :zones)
            JuMP.set_lower_bound(δPs[z], 0)
            JuMP.set_upper_bound(δPs[z],  ps_max)
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :contingency, :storage_cont, _PM.ids(pm, nw, :zones), δPs)
end

function variable_storage_contingency_indicator(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    delta_s = _PM.var(pm, nw)[:delta_s] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :storage)], base_name="$(nw)_delta_s",
        binary = true,
        start = 0,
        lower_bound = 0,
        upper_bound = 1
    )
    report && _PM.sol_component_value(pm, nw, :storage, :delta_s, _PM.ids(pm, nw, :storage), delta_s)
end

function variable_tieline_contingency(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    δPl_plus = _PM.var(pm, nw)[:tieline_cont_plus] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :areas)], base_name="$(nw)_tieline_cont_plus",
    start = 0.0
    )

    δPl_minus = _PM.var(pm, nw)[:tieline_cont_minus] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :areas)], base_name="$(nw)_tieline_cont_minus",
    start = 0.0
    )

    if bounded
        pl_max = maximum([branch["rate_a"] for (b, branch) in _PM.ref(pm, nw, :branch)])
        for (a, area) in _PM.ref(pm, nw, :areas)
            JuMP.set_lower_bound(δPl_plus[a], 0)
            JuMP.set_lower_bound(δPl_minus[a], -pl_max)
            JuMP.set_upper_bound(δPl_plus[a],  pl_max)
            JuMP.set_upper_bound(δPl_minus[a],  0)
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :contingency_l, :tieline_cont_plus, _PM.ids(pm, nw, :areas), δPl_plus)
    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :contingency_l, :tieline_cont_minus, _PM.ids(pm, nw, :areas), δPl_minus)
end

function variable_tieline_contingency_indicator(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    delta_l_plus = _PM.var(pm, nw)[:delta_l_plus] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :tie_lines)], base_name="$(nw)_delta_l_plus",
        binary = true,
        start = 0,
        lower_bound = 0,
        upper_bound = 1
    )

    delta_l_minus = _PM.var(pm, nw)[:delta_l_minus] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :tie_lines)], base_name="$(nw)_delta_l_minus",
        binary = true,
        start = 0,
        lower_bound = 0,
        upper_bound = 1
    )

    report && _PM.sol_component_value(pm, nw, :tie_lines, :delta_l_plus, _PM.ids(pm, nw, :tie_lines), delta_l_plus)
    report && _PM.sol_component_value(pm, nw, :tie_lines, :delta_l_minus, _PM.ids(pm, nw, :tie_lines), delta_l_minus)
end