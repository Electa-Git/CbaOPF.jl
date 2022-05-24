###############################################
# Constraint templates
function constraint_ohms_y_from_pst(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    pst = _PM.ref(pm, nw, :pst, i)
    f_bus = pst["f_bus"]
    t_bus = pst["t_bus"]
    f_idx = (i, f_bus, t_bus)
    t_idx = (i, t_bus, f_bus)

    g, b = _PM.calc_branch_y(pst)
    g_fr = pst["g_fr"]
    b_fr = pst["b_fr"]

    constraint_ohms_y_from_pst(pm, nw, i, f_bus, t_bus, f_idx, t_idx, g, b, g_fr, b_fr)
end

function constraint_ohms_y_to_pst(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    pst = _PM.ref(pm, nw, :pst, i)
    f_bus = pst["f_bus"]
    t_bus = pst["t_bus"]
    f_idx = (i, f_bus, t_bus)
    t_idx = (i, t_bus, f_bus)

    g, b = _PM.calc_branch_y(pst)
    g_to = pst["g_to"]
    b_to = pst["b_to"]

    constraint_ohms_y_to_pst(pm, nw, i, f_bus, t_bus, f_idx, t_idx, g, b, g_to, b_to)
end

function constraint_power_balance_ac(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    bus = _PM.ref(pm, nw, :bus, i)
    bus_arcs = _PM.ref(pm, nw, :bus_arcs, i)
    bus_arcs_pst = _PM.ref(pm, nw, :bus_arcs_pst, i)
    bus_arcs_sw = _PM.ref(pm, nw, :bus_arcs_sw, i)
    bus_gens = _PM.ref(pm, nw, :bus_gens, i)
    bus_loads = _PM.ref(pm, nw, :bus_loads, i)
    bus_shunts = _PM.ref(pm, nw, :bus_shunts, i)
    bus_storage = _PM.ref(pm, nw, :bus_storage, i)
    bus_convs_ac = _PM.ref(pm, nw, :bus_convs_ac, i)

    bus_pd = Dict(k => _PM.ref(pm, nw, :load, k, "pd") for k in bus_loads)
    bus_qd = Dict(k => _PM.ref(pm, nw, :load, k, "qd") for k in bus_loads)

    bus_gs = Dict(k => _PM.ref(pm, nw, :shunt, k, "gs") for k in bus_shunts)
    bus_bs = Dict(k => _PM.ref(pm, nw, :shunt, k, "bs") for k in bus_shunts)

    constraint_power_balance_ac(pm, nw, i, bus_arcs, bus_arcs_pst, bus_convs_ac, bus_arcs_sw, bus_gens, bus_storage, bus_loads, bus_gs, bus_bs)
end


function constraint_total_flexible_demand(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    load     = _PM.ref(pm, nw, :load, i)
    pd       = load["pd"]
    pf_angle = _FP.get(load, "pf_angle", 0.0) # Power factor angle, in radians
    constraint_total_flexible_demand(pm, nw, i, pd, pf_angle)
end


function constraint_fixed_xb_flows(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    xb_line_dict = _PM.ref(pm, nw, :borders, i, "xb_lines")
    arcs_xb_lines = []
    for (k, line) in xb_line_dict
        if line["direction"] == "from"
            push!(arcs_xb_lines, (line["index"],  line["f_bus"], line["t_bus"] ))
        else
            push!(arcs_xb_lines, (line["index"],  line["t_bus"], line["f_bus"] ))
        end
    end
    print(arcs_xb_lines,"\n")
    
    flow = _PM.ref(pm, nw, :borders, i, "flow")

    constraint_fixed_xb_flows(pm, nw, arcs_xb_lines, flow)
end