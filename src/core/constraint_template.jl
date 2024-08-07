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

function constraint_total_fixed_demand(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    load     = _PM.ref(pm, nw, :load, i)
    pd       = load["pd"]
    pf_angle = _FP.get(load, "pf_angle", 0.0) # Power factor angle, in radians
    constraint_total_fixed_demand(pm, nw, i, pd, pf_angle)
end

function constraint_fixed_xb_flows(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    xb_line_dict = _PM.ref(pm, nw, :borders, i, "xb_lines")
    xb_conv_dict = _PM.ref(pm, nw, :borders, i, "xb_convs")
    slack = _PM.ref(pm, nw, :borders, i, "slack")
    arcs_xb_lines = []
    xb_convs = []
    for (k, line) in xb_line_dict
        if haskey(line, "br_status") && line["br_status"] == 1
            if line["direction"] == "from"
                push!(arcs_xb_lines, (line["index"],  line["f_bus"], line["t_bus"] ))
            else
                push!(arcs_xb_lines, (line["index"],  line["t_bus"], line["f_bus"] ))
            end
        end
    end

    for (c, conv) in xb_conv_dict
        if haskey(conv, "index") && conv["status"] == 1
            push!(xb_convs, conv["index"])
        end
    end
    
    flow = _PM.ref(pm, nw, :borders, i, "flow")

    constraint_fixed_xb_flows(pm, nw, arcs_xb_lines, xb_convs, flow, slack)
end


function constraint_gen_redispatch(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    gen     = _PM.ref(pm, nw, :gen, i)

    if haskey(gen, "pg")
        pg_ref  = gen["pg"]
        constraint_gen_redispatch(pm, nw, i, pg_ref)
    end
end


function constraint_inertia_limit(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    inertia_limit = _PM.ref(pm, nw, :inertia_limit, i)["limit"]

    generator_properties = Dict()
    for (g, gen) in _PM.ref(pm, nw, :gen)
        if haskey(gen, "zone") && gen["zone"] == i && haskey(gen, "inertia_constants")
            push!(generator_properties, g => Dict("inertia" => gen["inertia_constants"], "rating" => gen["pmax"]))
        end
    end

    constraint_inertia_limit(pm, nw, generator_properties, inertia_limit)

    # conv_inertia = Dict()
    # for (c, conv) in _PM.ref(pm, nw, :convdc)
    #     if haskey(conv, "zone") && conv["zone"] == i
    #         push!(conv_inertia, c => conv["inertia_constants"])
    #     end
    # end
end

function constraint_generator_on_off(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default, use_status = true, second_stage = false)
    gen     = _PM.ref(pm, nw, :gen, i)
    pmax = gen["pmax"]
    pmin = gen["pmin"]
    if use_status == true
        status = gen["dispatch_status"]
    else
        status = 0
    end

    if second_stage == false
        constraint_generator_on_off(pm, nw, i, pmax, pmin, status)
    else
        nw_ref = get_reference_network_id(pm, nw)
        constraint_generator_on_off(pm, nw, nw_ref, i, pmax, pmin, status)
    end
end

function constraint_generator_status(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    constraint_generator_status(pm, nw, i)
end

function constraint_generator_status_uc(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    previous_hour_network = get_previous_hour_network_id(pm, nw)

    constraint_generator_status_uc(pm, nw, i, previous_hour_network)
end

function constraint_generator_status_cont(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    reference_network_idx = get_reference_network_id(pm, nw)
    constraint_generator_status_cont(pm, nw, i, reference_network_idx)
end


function constraint_active_conv_setpoint(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    conv = _PM.ref(pm, nw, :convdc, i)
    
    constraint_active_conv_setpoint(pm, nw, conv["index"], conv["P_g"])
end

function constraint_variable_branch_capacity_from(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    branch = _PM.ref(pm, nw, :branch, i)
    f_bus = branch["f_bus"]
    t_bus = branch["t_bus"]
    f_idx = (i, f_bus, t_bus)

    pmax = branch["rate_a"]

    constraint_variable_branch_capacity_from(pm, nw, f_idx, pmax)
end

function constraint_variable_branch_capacity_to(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    branch = _PM.ref(pm, nw, :branch, i)
    f_bus = branch["f_bus"]
    t_bus = branch["t_bus"]
    t_idx = (i, t_bus, f_bus)

    pmax = branch["rate_a"]

    constraint_variable_branch_capacity_to(pm, nw, t_idx, pmax)
end


function constraint_branch_capacity(pm::_PM.AbstractAPLossLessModels, i::Int; nw::Int=_PM.nw_id_default)
    n = nw
    n_1 = n - 1

    constraint_branch_capacity(pm, i, n, n_1)
end


# Constraint template for the energy content, e.g. state of charge of storage
function constraint_storage_state(pm::_PM.AbstractPowerModel, i::Int; nw::Int=nw_id_default)
    storage = _PM.ref(pm, nw, :storage_simple, i)

    if haskey(_PM.ref(pm, nw), :time_elapsed)
        time_elapsed = _PM.ref(pm, nw, :time_elapsed)
    else
        Memento.warn(_LOGGER, "network data should specify time_elapsed, using 1.0 as a default")
        time_elapsed = 1.0
    end

    constraint_storage_state(pm, nw, i, storage["charge_efficiency"], storage["discharge_efficiency"], time_elapsed)
end

# Constraint template for the energy content, e.g. state of charge of storage, in the first hour
function constraint_storage_initial_state(pm::_PM.AbstractPowerModel, i::Int; nw::Int=nw_id_default)
    storage = _PM.ref(pm, nw, :storage_simple, i)

    if haskey(_PM.ref(pm, nw), :time_elapsed)
        time_elapsed = _PM.ref(pm, nw, :time_elapsed)
    else
        Memento.warn(_LOGGER, "network data should specify time_elapsed, using 1.0 as a default")
        time_elapsed = 1.0
    end

    constraint_storage_initial_state(pm, nw, i, storage["energy"], storage["charge_efficiency"], storage["discharge_efficiency"], time_elapsed)
end

# Constraint to set the on/off state of storage devices for the interatia and FCR model
function constraint_storage_on_off(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    storage     = _PM.ref(pm, nw, :storage, i)
    charge_rating = storage["charge_rating"]
    discharge_rating = storage["discharge_rating"]

    nw_ref = get_reference_network_id(pm, nw)
    constraint_storage_on_off(pm, nw_ref, i, charge_rating, discharge_rating)
end

# Constraint template for AC nodes power balance
function constraint_power_balance(pm::_PM.AbstractAPLossLessModels, i::Int; nw::Int=_PM.nw_id_default)
    bus_arcs = _PM.ref(pm, nw, :bus_arcs, i)
    bus_gens = _PM.ref(pm, nw, :bus_gens, i)
    bus_loads = _PM.ref(pm, nw, :bus_loads, i)
    bus_storage = _PM.ref(pm, nw, :bus_storage, i)

    constraint_power_balance(pm, nw, i, bus_arcs, bus_gens, bus_loads, bus_storage)
end


function constraint_frequency(pm::_PM.AbstractPowerModel; nw::Int = _PM.nw_id_default, hvdc_contribution = false)
    reference_network_idx = get_reference_network_id(pm, nw)

    if !isnothing(_PM.ref(pm, nw, :contingency)["gen_id"])
        gcont = _PM.ref(pm, nw, :contingency)["gen_id"]
        generator_properties = Dict()
        zone_convs = Dict()
        for (g, gen) in _PM.ref(pm, nw, :gen)
            if haskey(gen, "zone")
                zone = gen["zone"]
                if !haskey(generator_properties, zone)
                    generator_properties[zone] = Dict()
                end
                if g != gcont
                    push!(generator_properties[zone], g => Dict("inertia" => gen["inertia_constants"], "rating" => gen["pmax"]))
                else
                    push!(generator_properties[zone], g => Dict("inertia" => 0, "rating" => gen["pmax"]))
                end
            end
        end

        for (c, conv) in _PM.ref(pm, nw, :convdc)
            if haskey(conv, "zone")
                zone = conv["zone"] 
                if !haskey(zone_convs, zone)
                    zone_convs[zone] = Dict()
                end
                push!(zone_convs[zone], c => Dict("t_hvdc" => conv["t_hvdc"]))
            end
        end

        frequency_parameters = _PM.ref(pm, nw, :frequency_parameters)
        ΔT = frequency_parameters["t_fcr"]
        fmin = frequency_parameters["fmin"]
        fmax = frequency_parameters["fmax"]
        f0 = frequency_parameters["f0"]
        if haskey(frequency_parameters, "fdb")
            fdb = frequency_parameters["fdb"]
        else
            fdb = 0
        end

        zones = [i for i in _PM.ids(pm, nw, :zones)]
        for i in zones
            zone = _PM.ref(pm, nw, :zones, i)["zone"]
            if haskey(generator_properties, zone)
                g_properties = generator_properties[zone]
            else
                g_properties = Dict()
            end
            if haskey(zone_convs, zone)
                z_convs = zone_convs[zone]
            else
                z_convs = Dict()
            end

            if _PM.ref(pm, nw, :gen, gcont)["zone"] == zone
                constraint_frequency(pm, nw, reference_network_idx, g_properties, gcont, ΔT, f0, fmin, fmax, fdb, z_convs, hvdc_contribution, i)
            else
                constraint_frequency(pm, nw, reference_network_idx, g_properties, ΔT, f0, fmin, fmax, fdb, z_convs, hvdc_contribution, i)
            end
        end
    end
end


function constraint_frequency_tie_line(pm::_PM.AbstractPowerModel; nw::Int = _PM.nw_id_default, hvdc_contribution = false)
    reference_network_idx = get_reference_network_id(pm, nw)

    if !isnothing(_PM.ref(pm, nw, :contingency)["branch_id"])
        bcont = _PM.ref(pm, nw, :contingency)["branch_id"]
        generator_properties = Dict()
        zone_convs = Dict()
        for (g, gen) in _PM.ref(pm, nw, :gen)
            if haskey(gen, "area")
                area = gen["area"]
                if !haskey(generator_properties, area)
                    generator_properties[area] = Dict()
                end
                push!(generator_properties[area], g => Dict("inertia" => gen["inertia_constants"], "rating" => gen["pmax"]))
            end
        end

        for (c, conv) in _PM.ref(pm, nw, :convdc)
            if haskey(conv, "area")
                area = conv["area"] 
                if !haskey(zone_convs, area)
                    zone_convs[area] = Dict()
                end
                push!(zone_convs[area], c => Dict("t_hvdc" => conv["t_hvdc"]))
            end
        end

        frequency_parameters = _PM.ref(pm, nw, :frequency_parameters)
        ΔT = frequency_parameters["t_fcr"]
        fmin = frequency_parameters["fmin"]
        fmax = frequency_parameters["fmax"]
        f0 = frequency_parameters["f0"]
        if haskey(frequency_parameters, "fdb")
            fdb = frequency_parameters["fdb"]
        else
            fdb = 0
        end


        tie_line = _PM.ref(pm, nw, :tie_lines)[bcont]
        br_id = tie_line["br_idx"]
        f_idx = _PM.ref(pm, nw, :branch)[br_id]["f_bus"]
        t_idx = _PM.ref(pm, nw, :branch)[br_id]["t_bus"]
        area_fr = tie_line["area_fr"]
        area_to = tie_line["area_to"]
        areas = [area_fr area_to]
        for i in areas
            if i == area_fr
                br_idx = (br_id, f_idx, t_idx)
            else
                br_idx = (br_id, t_idx, f_idx)
            end
            if haskey(generator_properties, i)
                g_properties = generator_properties[i]
            else
                g_properties = Dict()
            end
            if haskey(zone_convs, i)
                z_convs = zone_convs[i]
            else
                z_convs = Dict()
            end

            constraint_frequency_tie_line(pm, nw, reference_network_idx, g_properties, ΔT, f0, fmin, fmax, fdb, z_convs, hvdc_contribution, br_idx, i)
        end
    end
end

function constraint_frequency_converter(pm::_PM.AbstractPowerModel; nw::Int = _PM.nw_id_default, hvdc_contribution = false)
    reference_network_idx = get_reference_network_id(pm, nw)

    if !isnothing(_PM.ref(pm, nw, :contingency)["conv_id"])
        ccont = _PM.ref(pm, nw, :contingency)["conv_id"]
        generator_properties = Dict()
        zone_convs = Dict()
        for (g, gen) in _PM.ref(pm, nw, :gen)
            if haskey(gen, "zone")
                zone = gen["zone"]
                if !haskey(generator_properties, zone)
                    generator_properties[zone] = Dict()
                end
                push!(generator_properties[zone], g => Dict("inertia" => gen["inertia_constants"], "rating" => gen["pmax"]))
            end
        end

        for (c, conv) in _PM.ref(pm, nw, :convdc)
            if haskey(conv, "zone")
                zone = conv["zone"] 
                if !haskey(zone_convs, zone)
                    zone_convs[zone] = Dict()
                end
                if c != ccont
                    push!(zone_convs[zone], c => Dict("t_hvdc" => conv["t_hvdc"]))
                end
            end
        end

        frequency_parameters = _PM.ref(pm, nw, :frequency_parameters)
        ΔT = frequency_parameters["t_fcr"]
        fmin = frequency_parameters["fmin"]
        fmax = frequency_parameters["fmax"]
        f0 = frequency_parameters["f0"]
        if haskey(frequency_parameters, "fdb")
            fdb = frequency_parameters["fdb"]
        else
            fdb = 0
        end

        zones = [i for i in _PM.ids(pm, nw, :zones)]
        for i in zones
            zone = _PM.ref(pm, nw, :zones, i)["zone"]
            if haskey(generator_properties, zone)
                g_properties = generator_properties[zone]
            else
                g_properties = Dict()
            end
            if haskey(zone_convs, zone)
                z_convs = zone_convs[zone]
            else
                z_convs = Dict()
            end

            if _PM.ref(pm, nw, :convdc, ccont)["zone"] == zone
                constraint_frequency_converter(pm, nw, reference_network_idx , g_properties, ccont, ΔT, f0, fmin, fmax, fdb, z_convs, hvdc_contribution, i)
            else
                constraint_frequency_converter(pm, nw, reference_network_idx , g_properties, ΔT, f0, fmin, fmax, fdb, z_convs, hvdc_contribution, i)
            end
        end
    end
end

function constraint_dc_branch_contingency(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    branch =_PM.ref(pm, nw, :branchdc, i)
    f_bus = branch["fbusdc"]
    t_bus = branch["tbusdc"]
    f_idx = (i, f_bus, t_bus)
    t_idx = (i, t_bus, f_bus)

    constraint_dc_branch_contingency(pm, nw, f_idx, t_idx)
end

function constraint_dc_conv_contingency(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    constraint_dc_conv_contingency(pm, nw, i)
end

function constraint_converter_power_balance(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    reference_network_idx = get_reference_network_id(pm, nw)

    constraint_converter_power_balance(pm, i, nw, reference_network_idx)
end

function constraint_converter_contribution_absolute(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    constraint_converter_contribution_absolute(pm, i, nw)
end

function get_reference_network_id(pm::_PM.AbstractPowerModel, nw::Int)
    number_of_contingencies = pm.ref[:it][:pm][:number_of_contingencies]
    if mod(nw, number_of_contingencies) == 0
        hour_id = Int(nw - number_of_contingencies + 1)
    else
        hour_id = Int(nw - mod(nw, number_of_contingencies) + 1)
    end

    reference_network_idx = hour_id

    return reference_network_idx 
end

function get_previous_hour_network_id(pm::_PM.AbstractPowerModel, nw::Int)
    number_of_contingencies = pm.ref[:it][:pm][:number_of_contingencies]

    if number_of_contingencies == 0
        previous_hour_id = Int((nw - 1))
        previous_hour_network = pm.ref[:it][:pm][:hour_ids][previous_hour_id]
    else
        previous_hour_id = Int((nw - 1) / number_of_contingencies)
        previous_hour_network = pm.ref[:it][:pm][:hour_ids][previous_hour_id]
    end

    return previous_hour_network
end

function contstraint_unit_commitment(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    constraint_generator_decisions(pm, i, nw)
    constraint_minimum_up_time(pm, i, nw)
    constraint_minimum_down_time(pm, i, nw)
end

function constraint_generator_decisions(pm::_PM.AbstractPowerModel, i::Int, nw::Int = _PM.nw_id_default)
    if nw == 1
        constraint_initial_generator_decisions(pm, i, nw)
    else
        previous_hour_network = get_previous_hour_network_id(pm, nw)
        constraint_generator_decisions(pm, i, nw, previous_hour_network)
        constraint_generator_ramping(pm, i, nw, previous_hour_network)
    end
end

function constraint_minimum_up_time(pm::_PM.AbstractPowerModel, i::Int, nw::Int = _PM.nw_id_default)
    gen =_PM.ref(pm, nw, :gen, i)
    mut = gen["mut"]
    if pm.ref[:it][:pm][:number_of_contingencies] !== 0
        interval = pm.ref[:it][:pm][:number_of_contingencies]
    else
        interval = 1
    end
    h_start = max(1, (nw + interval - (mut * interval))) 
    τ = h_start : interval : nw

    return constraint_minimum_up_time(pm, i, nw, τ)
end

function constraint_minimum_down_time(pm::_PM.AbstractPowerModel, i::Int, nw::Int = _PM.nw_id_default)
    gen =_PM.ref(pm, nw, :gen, i)
    mdt = gen["mdt"]
    if pm.ref[:it][:pm][:number_of_contingencies] !== 0
        interval = pm.ref[:it][:pm][:number_of_contingencies]
    else
        interval = 1
    end
    h_start = max(1, (nw + interval - (mdt * interval))) 
    τ = h_start : interval : nw

    return constraint_minimum_down_time(pm, i, nw, τ)
end

function constraint_generator_ramping(pm::_PM.AbstractPowerModel, i::Int, nw::Int, previous_hour_network)
    gen = _PM.ref(pm, nw, :gen, i)
    Δt = _PM.ref(pm, nw, :frequency_parameters)["uc_time_interval"]
    pmax = gen["pmax"]
    pmin = gen["pmin"]
    ΔPg_up = gen["ramp_rate"] * Δt * pmax
    ΔPg_down = gen["ramp_rate"] * Δt * pmax

    return constraint_generator_ramping(pm, i, nw, previous_hour_network, ΔPg_up, ΔPg_down, pmin)
end

function constraint_unit_commitment_reserves(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    return constraint_unit_commitment_reserves(pm, i, nw)
end

function constraint_frequency_droop(pm::_PM.AbstractPowerModel; nw::Int = _PM.nw_id_default, hvdc_contribution = false)
    reference_network_idx = get_reference_network_id(pm, nw)

    if !isnothing(_PM.ref(pm, nw, :contingency)["gen_id"])
        gcont = _PM.ref(pm, nw, :contingency)["gen_id"]
        generator_properties = Dict()
        zone_convs = Dict()
        for (g, gen) in _PM.ref(pm, nw, :gen)
            if haskey(gen, "zone")
                zone = gen["zone"]
                if !haskey(generator_properties, zone)
                    generator_properties[zone] = Dict()
                end
                if g != gcont
                    push!(generator_properties[zone], g => Dict("inertia" => gen["inertia_constants"], "rating" => gen["pmax"]))
                else
                    push!(generator_properties[zone], g => Dict("inertia" => 0, "rating" => gen["pmax"]))
                end
            end
        end

        for (c, conv) in _PM.ref(pm, nw, :convdc)
            if haskey(conv, "zone")
                zone = conv["zone"] 
                if !haskey(zone_convs, zone)
                    zone_convs[zone] = Dict()
                end
                push!(zone_convs[zone], c => Dict("t_hvdc" => conv["t_hvdc"]))
            end
        end

        frequency_parameters = _PM.ref(pm, nw, :frequency_parameters)
        ΔTin = frequency_parameters["t_fcr"]
        ΔTdroop = frequency_parameters["t_fcrd"]
        fmin = frequency_parameters["fmin"]
        fmax = frequency_parameters["fmax"]
        f0 = frequency_parameters["f0"]
        Δfss = frequency_parameters["delta_fss"]
        if haskey(frequency_parameters, "fdb")
            fdb = frequency_parameters["fdb"]
        else
            fdb = 0
        end

        zones = [i for i in _PM.ids(pm, nw, :zones)]
        for i in zones
            zone = _PM.ref(pm, nw, :zones, i)["zone"]
            if haskey(generator_properties, zone)
                g_properties = generator_properties[zone]
            else
                g_properties = Dict()
            end
            if haskey(zone_convs, zone)
                z_convs = zone_convs[zone]
            else
                z_convs = Dict()
            end
            ΔPg = maximum([gen["rating"] for (g, gen) in generator_properties[zone]])
            # ΔPg = _PM.ref(pm, nw, :gen, gcont)["pmax"]
            if _PM.ref(pm, nw, :gen, gcont)["zone"] == zone
                constraint_frequency_droop(pm, nw, reference_network_idx, g_properties, ΔPg, gcont, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, z_convs, hvdc_contribution, i)
            else
                constraint_frequency_droop(pm, nw, reference_network_idx, g_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, z_convs, hvdc_contribution, i)
            end
        end
    end
end

function constraint_frequency_tie_line_droop(pm::_PM.AbstractPowerModel; nw::Int = _PM.nw_id_default, hvdc_contribution = false)
    reference_network_idx = get_reference_network_id(pm, nw)

    if !isnothing(_PM.ref(pm, nw, :contingency)["branch_id"])
        bcont = _PM.ref(pm, nw, :contingency)["branch_id"]
        generator_properties = Dict()
        zone_convs = Dict()
        for (g, gen) in _PM.ref(pm, nw, :gen)
            if haskey(gen, "area")
                area = gen["area"]
                if !haskey(generator_properties, area)
                    generator_properties[area] = Dict()
                end
                push!(generator_properties[area], g => Dict("inertia" => gen["inertia_constants"], "rating" => gen["pmax"]))
            end
        end

        for (c, conv) in _PM.ref(pm, nw, :convdc)
            if haskey(conv, "area")
                area = conv["area"] 
                if !haskey(zone_convs, area)
                    zone_convs[area] = Dict()
                end
                push!(zone_convs[area], c => Dict("t_hvdc" => conv["t_hvdc"]))
            end
        end

        frequency_parameters = _PM.ref(pm, nw, :frequency_parameters)
        ΔTin = frequency_parameters["t_fcr"]
        ΔTdroop = frequency_parameters["t_fcrd"]
        fmin = frequency_parameters["fmin"]
        fmax = frequency_parameters["fmax"]
        f0 = frequency_parameters["f0"]
        Δfss = frequency_parameters["delta_fss"]
        if haskey(frequency_parameters, "fdb")
            fdb = frequency_parameters["fdb"]
        else
            fdb = 0
        end

        tie_line = _PM.ref(pm, nw, :tie_lines)[bcont]
        br_id = tie_line["br_idx"]
        f_idx = _PM.ref(pm, nw, :branch)[br_id]["f_bus"]
        t_idx = _PM.ref(pm, nw, :branch)[br_id]["t_bus"]
        area_fr = tie_line["area_fr"]
        area_to = tie_line["area_to"]
        areas = [area_fr area_to]
        for i in areas
            if i == area_fr
                br_idx = (br_id, f_idx, t_idx)
            else
                br_idx = (br_id, t_idx, f_idx)
            end
            if haskey(generator_properties, i)
                g_properties = generator_properties[i]
            else
                g_properties = Dict()
            end
            if haskey(zone_convs, i)
                z_convs = zone_convs[i]
            else
                z_convs = Dict()
            end

            constraint_frequency_tie_line_droop(pm, nw, reference_network_idx, g_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, z_convs, hvdc_contribution, br_idx, i)
        end
    end
end

function constraint_frequency_converter_droop(pm::_PM.AbstractPowerModel; nw::Int = _PM.nw_id_default, hvdc_contribution = false)
    reference_network_idx = get_reference_network_id(pm, nw)

    if !isnothing(_PM.ref(pm, nw, :contingency)["conv_id"])
        ccont = _PM.ref(pm, nw, :contingency)["conv_id"]
        generator_properties = Dict()
        zone_convs = Dict()
        for (g, gen) in _PM.ref(pm, nw, :gen)
            if haskey(gen, "zone")
                zone = gen["zone"]
                if !haskey(generator_properties, zone)
                    generator_properties[zone] = Dict()
                end
                push!(generator_properties[zone], g => Dict("inertia" => gen["inertia_constants"], "rating" => gen["pmax"]))
            end
        end

        for (c, conv) in _PM.ref(pm, nw, :convdc)
            if haskey(conv, "zone")
                zone = conv["zone"] 
                if !haskey(zone_convs, zone)
                    zone_convs[zone] = Dict()
                end
                if c != ccont
                    push!(zone_convs[zone], c => Dict("t_hvdc" => conv["t_hvdc"]))
                end
            end
        end

        frequency_parameters = _PM.ref(pm, nw, :frequency_parameters)
        ΔTin = frequency_parameters["t_fcr"]
        ΔTdroop = frequency_parameters["t_fcrd"]
        fmin = frequency_parameters["fmin"]
        fmax = frequency_parameters["fmax"]
        f0 = frequency_parameters["f0"]
        Δfss = frequency_parameters["delta_fss"]
        if haskey(frequency_parameters, "fdb")
            fdb = frequency_parameters["fdb"]
        else
            fdb = 0
        end

        zones = [i for i in _PM.ids(pm, nw, :zones)]
        for i in zones
            zone = _PM.ref(pm, nw, :zones, i)["zone"]
            if haskey(generator_properties, zone)
                g_properties = generator_properties[zone]
            else
                g_properties = Dict()
            end
            if haskey(zone_convs, zone)
                z_convs = zone_convs[zone]
            else
                z_convs = Dict()
            end

            if _PM.ref(pm, nw, :convdc, ccont)["zone"] == zone
                constraint_frequency_converter_droop(pm, nw, reference_network_idx , g_properties, ccont, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, z_convs, hvdc_contribution, i)
            else
                constraint_frequency_converter_droop(pm, nw, reference_network_idx , g_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, z_convs, hvdc_contribution, i)
            end
        end
    end
end

function constraint_gen_droop_power_balance(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    reference_network_idx = get_reference_network_id(pm, nw)

    constraint_gen_droop_power_balance(pm, i, nw, reference_network_idx)
end


function constraint_generator_droop(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    gen = _PM.ref(pm, nw, :gen, i)
    ramp_rate = gen["ramp_rate_per_s"]

    ΔTin = _PM.ref(pm, nw, :frequency_parameters)["t_fcr"]
    ΔTdroop = _PM.ref(pm, nw, :frequency_parameters)["t_fcrd"]

    return constraint_generator_droop(pm, i, nw, ramp_rate, ΔTin, ΔTdroop)
end

function constraint_generator_droop_absolute(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    constraint_generator_droop_absolute(pm, i, nw)
end


function constraint_generator_contingency(pm::_PM.AbstractPowerModel; nw::Int = _PM.nw_id_default)
    reference_network_idx = get_reference_network_id(pm, nw)

    reference_network_idx = get_reference_network_id(pm, nw)
    if haskey(_PM.ref(pm, nw), :excluded_zones)
        excluded_zones = _PM.ref(pm, nw, :excluded_zones)
    else
        excluded_zones = []
    end
    zones = [i for i in _PM.ids(pm, nw, :zones) if !any(i .== excluded_zones)]
    for zone in zones
        for (g, gen) in _PM.ref(pm, nw, :gen)
            if haskey(gen, "zone") && _PM.ref(pm, nw, :zones, zone)["zone"] == gen["zone"]
                constraint_generator_contingency(pm, g, reference_network_idx, zone)
            end
        end
    end 
end

function constraint_generator_contingency_indicator(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    reference_network_idx = get_reference_network_id(pm, nw)

    bigM = 2 * maximum([gen["pmax"] for (g, gen) in _PM.ref(pm, nw, :gen)])
    gen = _PM.ref(pm, nw, :gen, i)
    gen_zone = gen["zone"]
    if haskey(_PM.ref(pm, nw), :excluded_zones)
        excluded_zones = _PM.ref(pm, nw, :excluded_zones)
    else
        excluded_zones = []
    end
    zones = [i for i in _PM.ids(pm, nw, :zones) if !any(i .== excluded_zones)]
    for z in zones
        zone = _PM.ref(pm, nw, :zones, z)["zone"]
        if gen_zone == zone
            constraint_generator_contingency_indicator(pm, i, reference_network_idx, bigM, z)
        end
    end
end

function constraint_select_generator_contingency(pm::_PM.AbstractPowerModel; nw::Int = _PM.nw_id_default)
    reference_network_idx = get_reference_network_id(pm, nw)
    if haskey(_PM.ref(pm, nw), :excluded_zones)
        excluded_zones = _PM.ref(pm, nw, :excluded_zones)
    else
        excluded_zones = []
    end
    
    zones = [i for i in _PM.ids(pm, nw, :zones) if !any(i .== excluded_zones)]
    for zone in zones
        zone_gens = []
        for (g, gen) in _PM.ref(pm, nw, :gen)
            if haskey(gen, "zone") && _PM.ref(pm, nw, :zones, zone)["zone"] == gen["zone"]
                append!(zone_gens, g)
            end
        end
        constraint_select_generator_contingency(pm, reference_network_idx, zone_gens)
    end 
end

function constraint_converter_contingency(pm::_PM.AbstractPowerModel; nw::Int = _PM.nw_id_default)
    reference_network_idx = get_reference_network_id(pm, nw)
    if haskey(_PM.ref(pm, nw), :excluded_zones)
        excluded_zones = _PM.ref(pm, nw, :excluded_zones)
    else
        excluded_zones = []
    end
    zones = [i for i in _PM.ids(pm, nw, :zones) if !any(i .== excluded_zones)]
    for zone in zones
        for (c, conv) in _PM.ref(pm, nw, :convdc)
            if haskey(conv, "zone") && _PM.ref(pm, nw, :zones, zone)["zone"] == conv["zone"]
                constraint_converter_contingency(pm, c, reference_network_idx, zone)
            end
        end
    end
end

function constraint_converter_contingency_indicator(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    reference_network_idx = get_reference_network_id(pm, nw)
    bigM = 2 * maximum([conv["Pacrated"] for (c, conv) in _PM.ref(pm, nw, :convdc)])

    conv = _PM.ref(pm, nw, :convdc, i)
    conv_zone = conv["zone"]
    if haskey(_PM.ref(pm, nw), :excluded_zones)
        excluded_zones = _PM.ref(pm, nw, :excluded_zones)
    else
        excluded_zones = []
    end
    zones = [i for i in _PM.ids(pm, nw, :zones) if !any(i .== excluded_zones)]
    for z in zones
        zone = _PM.ref(pm, nw, :zones, z)["zone"]
        if conv_zone == zone
            constraint_converter_contingency_indicator(pm, i, reference_network_idx, bigM, z)
        end
    end
end

function constraint_select_converter_contingency(pm::_PM.AbstractPowerModel; nw::Int = _PM.nw_id_default)
    reference_network_idx = get_reference_network_id(pm, nw)
    if haskey(_PM.ref(pm, nw), :excluded_zones)
        excluded_zones = _PM.ref(pm, nw, :excluded_zones)
    else
        excluded_zones = []
    end
    zones = [i for i in _PM.ids(pm, nw, :zones) if !any(i .== excluded_zones)]
    for zone in zones
        zone_convs = []
        for (c, conv) in _PM.ref(pm, nw, :convdc)
            if haskey(conv, "zone") && _PM.ref(pm, nw, :zones, zone)["zone"] == conv["zone"]
                append!(zone_convs, c)
            end
        end
        constraint_select_converter_contingency(pm, reference_network_idx, zone_convs)
    end 
end

function constraint_tieline_contingency(pm::_PM.AbstractPowerModel; nw::Int = _PM.nw_id_default)
    reference_network_idx = get_reference_network_id(pm, nw)
    areas = [i for i in _PM.ids(pm, nw, :areas)]
    for area in areas
        for (t, tieline) in _PM.ref(pm, nw, :tie_lines)
            if (haskey(tieline, "area_fr") && _PM.ref(pm, nw, :areas, area)["area"] == tieline["area_fr"])  || (haskey(tieline, "area_to") && _PM.ref(pm, nw, :areas, area)["area"] == tieline["area_to"]) 
                br_idx = tieline["br_idx"]
                fbus = _PM.ref(pm, nw, :branch, br_idx)["f_bus"]
                tbus = _PM.ref(pm, nw, :branch, br_idx)["t_bus"]
                constraint_tieline_contingency(pm, br_idx, fbus, tbus, reference_network_idx, area)
            end
        end
    end
end

function constraint_tieline_contingency_indicator(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    reference_network_idx = get_reference_network_id(pm, nw)
    bigM = 2 * maximum([branch["rate_a"] for (b, branch) in _PM.ref(pm, nw, :branch)])

    tieline = _PM.ref(pm, nw, :tie_lines, i)
    br_idx = tieline["br_idx"]
    fbus = _PM.ref(pm, nw, :branch, br_idx)["f_bus"]
    tbus = _PM.ref(pm, nw, :branch, br_idx)["t_bus"]
    areas = [a for a in _PM.ids(pm, nw, :areas)]
    for a in areas
        area = _PM.ref(pm, nw, :areas, a)
        if (haskey(tieline, "area_fr") && _PM.ref(pm, nw, :areas, a)["area"] == tieline["area_fr"])  || (haskey(tieline, "area_to") && _PM.ref(pm, nw, :areas, a)["area"] == tieline["area_to"]) 
            constraint_tieline_contingency_indicator(pm, i, br_idx, fbus, tbus, reference_network_idx, bigM, a)
        end
    end
end

function constraint_select_tieline_contingency(pm::_PM.AbstractPowerModel; nw::Int = _PM.nw_id_default)
    reference_network_idx = get_reference_network_id(pm, nw)
    areas = [i for i in _PM.ids(pm, nw, :areas)]
    for area in areas
        tielines = []
        for (t, tieline) in _PM.ref(pm, nw, :tie_lines)
            if (haskey(tieline, "area_fr") && _PM.ref(pm, nw, :areas, area)["area"] == tieline["area_fr"]) || (haskey(tieline, "area_to") && _PM.ref(pm, nw, :areas, area)["area"] == tieline["area_to"]) # one constraint should be enough per area: To be checked
                append!(tielines, t)
            end
        end
        if !isempty(tielines)
            println(area, " ", tielines)
            constraint_select_tieline_contingency(pm, reference_network_idx, tielines)
        end
    end 
end

# function constraint_branch_contingency(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
#     branch_id = _PM.ref(pm, nw, :tie_lines, i)["br_idx"]
#     branch = _PM.ref(pm, nw, :branch, branch_id)
#     f_idx = branch["f_bus"]
#     t_idx = branch["t_bus"]
#     br_idx = (branch_id, f_idx, t_idx)

#     constraint_branch_contingency(pm, br_idx, nw)
# end

# function constraint_branch_contingency_indicator(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
#     branch_id = _PM.ref(pm, nw, :tie_lines, i)["br_idx"]
#     branch = _PM.ref(pm, nw, :branch, branch_id)
#     f_idx = branch["f_bus"]
#     t_idx = branch["t_bus"]
#     br_idx = (branch_id, f_idx, t_idx)

#     bigM = 2 * maximum([branch["rate_a"] for (br, branch) in _PM.ref(pm, nw, :branch)])
#     constraint_branch_contingency_indicator(pm, i, nw, br_idx, bigM)
# end

function constraint_frequency_droop_lean(pm::_PM.AbstractPowerModel, zone_id; nw::Int = _PM.nw_id_default, hvdc_contribution = false)
    ref_id = get_reference_network_id(pm, nw)
    generator_properties = Dict()
    zone_convs = Dict()
    storage_properties = Dict()
    zone = _PM.ref(pm, nw, :zones, zone_id)["zone"]

    for (g, gen) in _PM.ref(pm, nw, :gen)
        if haskey(gen, "zone")
            gen_zone = gen["zone"]
            if !haskey(generator_properties, gen_zone)
                generator_properties[gen_zone] = Dict()
            end
        push!(generator_properties[gen_zone], g => Dict("inertia" => gen["inertia_constants"], "rating" => gen["pmax"]))
        end
    end

    for (s, storage) in _PM.ref(pm, nw, :storage)
        if haskey(storage, "zone")
            st_zone = storage["zone"]
            if !haskey(storage_properties, st_zone)
                storage_properties[st_zone] = Dict()
            end
        push!(storage_properties[st_zone], s => Dict("inertia" => storage["inertia_constants"], "rating" => storage["thermal_rating"]))
        end
    end

    for (c, conv) in _PM.ref(pm, nw, :convdc)
        if haskey(conv, "zone")
            conv_zone = conv["zone"] 
            if !haskey(zone_convs, conv_zone)
                zone_convs[conv_zone] = Dict()
            end
            push!(zone_convs[conv_zone], c => Dict("t_hvdc" => conv["t_hvdc"]))
        end
    end

    frequency_parameters = _PM.ref(pm, nw, :frequency_parameters)
    ΔTin = frequency_parameters["t_fcr"]
    ΔTdroop = frequency_parameters["t_fcrd"]
    fmin = frequency_parameters["fmin"]
    fmax = frequency_parameters["fmax"]
    f0 = frequency_parameters["f0"]
    Δfss = frequency_parameters["delta_fss"]
    if haskey(frequency_parameters, "fdb")
        fdb = frequency_parameters["fdb"]
    else
        fdb = 0
    end
    if haskey(_PM.ref(pm, nw), :excluded_zones)
        excluded_zones = _PM.ref(pm, nw, :excluded_zones)
    else
        excluded_zones = []
    end
    zones = [z["zone"] for (i,z) in _PM.ref(pm, nw, :zones) if !any(z["zone"] .== excluded_zones)]
    zone_ids = [i for (i,z) in _PM.ref(pm, nw, :zones) if !any(z["zone"] .== excluded_zones)]
    if any(zone .== zones)
        if haskey(generator_properties, zone)
            g_properties = generator_properties[zone]
        else
            g_properties = Dict()
        end
        if haskey(storage_properties, zone)
            s_properties = storage_properties[zone]
        else
            s_properties = 0
        end
        if haskey(zone_convs, zone)
            z_convs = zone_convs[zone]
        else
            z_convs = Dict()
        end
        constraint_frequency_droop_lean(pm, nw, ref_id, g_properties, s_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, z_convs, hvdc_contribution, zone_id, zone_cont = true)
        for j in setdiff(zone_ids, zone_id)
            zone_ = _PM.ref(pm, nw, :zones, j)["zone"]
            if haskey(generator_properties, zone_)
                g_properties = generator_properties[zone_]
            else
                g_properties = Dict()
            end
            if haskey(storage_properties, zone)
                s_properties = storage_properties[zone_]
            else
                s_properties = 0
            end
            if haskey(zone_convs, zone_)
                z_convs = zone_convs[zone_]
            else
                z_convs = Dict()
            end
            constraint_frequency_droop_lean(pm, nw, ref_id, g_properties, s_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, z_convs, hvdc_contribution, j, zone_cont = false)
        end
    end
end

function constraint_frequency_converter_droop_lean(pm::_PM.AbstractPowerModel, zone_id; nw::Int = _PM.nw_id_default, hvdc_contribution = false)
    ref_id = get_reference_network_id(pm, nw)
    zone = _PM.ref(pm, nw, :zones, zone_id)["zone"]
    generator_properties = Dict()
    storage_properties = Dict()
    zone_convs = Dict()
    for (g, gen) in _PM.ref(pm, nw, :gen)
        if haskey(gen, "zone")
            zone = gen["zone"]
            if !haskey(generator_properties, zone)
                generator_properties[zone] = Dict()
            end
        push!(generator_properties[zone], g => Dict("inertia" => gen["inertia_constants"], "rating" => gen["pmax"]))
        end
    end

    for (s, storage) in _PM.ref(pm, nw, :storage)
        if haskey(storage, "zone")
            st_zone = storage["zone"]
            if !haskey(storage_properties, st_zone)
                storage_properties[st_zone] = Dict()
            end
        push!(storage_properties[st_zone], s => Dict("inertia" => storage["inertia_constants"], "rating" => storage["thermal_rating"]))
        end
    end

    for (c, conv) in _PM.ref(pm, nw, :convdc)
        if haskey(conv, "zone")
            zone = conv["zone"] 
            if !haskey(zone_convs, zone)
                zone_convs[zone] = Dict()
            end
            push!(zone_convs[zone], c => Dict("t_hvdc" => conv["t_hvdc"]))
        end
    end

    frequency_parameters = _PM.ref(pm, nw, :frequency_parameters)
    ΔTin = frequency_parameters["t_fcr"]
    ΔTdroop = frequency_parameters["t_fcrd"]
    fmin = frequency_parameters["fmin"]
    fmax = frequency_parameters["fmax"]
    f0 = frequency_parameters["f0"]
    Δfss = frequency_parameters["delta_fss"]
    if haskey(frequency_parameters, "fdb")
        fdb = frequency_parameters["fdb"]
    else
        fdb = 0
    end
    if haskey(_PM.ref(pm, nw), :excluded_zones)
        excluded_zones = _PM.ref(pm, nw, :excluded_zones)
    else
        excluded_zones = []
    end
    
    zones = [z["zone"] for (i,z) in _PM.ref(pm, nw, :zones) if !any(z["zone"] .== excluded_zones)]
    zone_ids = [i for (i,z) in _PM.ref(pm, nw, :zones) if !any(z["zone"] .== excluded_zones)]
    if any(zone .== zones)
        if haskey(generator_properties, zone)
            g_properties = generator_properties[zone]
        else
            g_properties = Dict()
        end
        if haskey(storage_properties, zone)
            s_properties = storage_properties[zone]
        else
            s_properties = 0
        end
        if haskey(zone_convs, zone)
            z_convs = zone_convs[zone]
        else
            z_convs = Dict()
        end
        
        constraint_frequency_converter_droop_lean(pm, nw, ref_id, g_properties, s_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, z_convs, hvdc_contribution, zone_id, zone_cont = true, direction = "plus")
        constraint_frequency_converter_droop_lean(pm, nw, ref_id, g_properties, s_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, z_convs, hvdc_contribution, zone_id, zone_cont = true, direction = "minus")
        for j in setdiff(zone_ids, zone_id)
            zone_ = _PM.ref(pm, nw, :zones, j)["zone"]
            if haskey(generator_properties, zone_)
                g_properties = generator_properties[zone_]
            else
                g_properties = Dict()
            end
            if haskey(storage_properties, zone_)
                s_properties = storage_properties[zone_]
            else
                s_properties = Dict()
            end
            if haskey(zone_convs, zone_)
                z_convs = zone_convs[zone_]
            else
                z_convs = Dict()
            end
            constraint_frequency_converter_droop_lean(pm, nw, ref_id, g_properties, s_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, z_convs, hvdc_contribution, j, zone_cont = false, direction = "plus")
            constraint_frequency_converter_droop_lean(pm, nw, ref_id, g_properties, s_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, z_convs, hvdc_contribution, j, zone_cont = false, direction = "minus")
        end
    end
end


function constraint_frequency_tieline_droop_lean(pm::_PM.AbstractPowerModel, area_id; nw::Int = _PM.nw_id_default, hvdc_contribution = false)
    ref_id = get_reference_network_id(pm, nw)
    area = _PM.ref(pm, nw, :areas, area_id)["area"]
    generator_properties = Dict()
    storage_properties = Dict()
    area_convs = Dict()
    for (g, gen) in _PM.ref(pm, nw, :gen)
        if haskey(gen, "area")
            area = gen["area"]
            if !haskey(generator_properties, area)
                generator_properties[area] = Dict()
            end
        push!(generator_properties[area], g => Dict("inertia" => gen["inertia_constants"], "rating" => gen["pmax"]))
        end
    end

    for (s, storage) in _PM.ref(pm, nw, :storage)
        if haskey(storage, "area")
            st_area = storage["area"]
            if !haskey(storage_properties, st_area)
                storage_properties[st_area] = Dict()
            end
        push!(storage_properties[st_area], s => Dict("inertia" => storage["inertia_constants"], "rating" => storage["thermal_rating"]))
        end
    end

    for (c, conv) in _PM.ref(pm, nw, :convdc)
        if haskey(conv, "area")
            area = conv["area"] 
            if !haskey(area_convs, area)
                area_convs[area] = Dict()
            end
            push!(area_convs[area], c => Dict("t_hvdc" => conv["t_hvdc"]))
        end
    end

    frequency_parameters = _PM.ref(pm, nw, :frequency_parameters)
    ΔTin = frequency_parameters["t_fcr"]
    ΔTdroop = frequency_parameters["t_fcrd"]
    fmin = frequency_parameters["fmin"]
    fmax = frequency_parameters["fmax"]
    f0 = frequency_parameters["f0"]
    Δfss = frequency_parameters["delta_fss"]
    if haskey(frequency_parameters, "fdb")
        fdb = frequency_parameters["fdb"]
    else
        fdb = 0
    end
    if haskey(_PM.ref(pm, nw), :excluded_areas)
        excluded_areas = _PM.ref(pm, nw, :excluded_areas)
    else
        excluded_areas = []
    end
    
    areas = [a["area"] for (i,a) in _PM.ref(pm, nw, :areas) if !any(a["area"] .== excluded_areas)]
    area_ids = [i for (i,a) in _PM.ref(pm, nw, :areas) if !any(a["area"] .== excluded_areas)]
    if any(area .== areas)
        if haskey(generator_properties, area)
            g_properties = generator_properties[area]
        else
            g_properties = Dict()
        end
        if haskey(storage_properties, area)
            s_properties = storage_properties[area]
        else
            s_properties = Dict()
        end
        if haskey(area_convs, area)
            a_convs = area_convs[area]
        else
            a_convs = Dict()
        end
        
        constraint_frequency_tieline_droop_lean(pm, nw, ref_id, g_properties, s_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, a_convs, hvdc_contribution, area_id, zone_cont = true, direction = "fr")
        constraint_frequency_tieline_droop_lean(pm, nw, ref_id, g_properties, s_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, a_convs, hvdc_contribution, area_id, zone_cont = true, direction = "to")

        for j in setdiff(area_ids, area_id)
            area_ = _PM.ref(pm, nw, :areas, j)["area"]
            if haskey(generator_properties, area_)
                g_properties = generator_properties[area_]
            else
                g_properties = Dict()
            end
            if haskey(storage_properties, area_)
                s_properties = storage_properties[area_]
            else
                s_properties = Dict()
            end
            if haskey(area_convs, area_)
                a_convs = area_convs[area_]
            else
                a_convs = Dict()
            end
            constraint_frequency_tieline_droop_lean(pm, nw, ref_id, g_properties, s_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, a_convs, hvdc_contribution, j, zone_cont = false, direction = "fr")
            constraint_frequency_tieline_droop_lean(pm, nw, ref_id, g_properties, s_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, a_convs, hvdc_contribution, j, zone_cont = false, direction = "to")
        end
    end
end


function constraint_frequency_storage_droop_lean(pm::_PM.AbstractPowerModel, zone_id; nw::Int = _PM.nw_id_default, hvdc_contribution = false)
    ref_id = get_reference_network_id(pm, nw)
    zone = _PM.ref(pm, nw, :zones, zone_id)["zone"]
    generator_properties = Dict()
    storage_properties = Dict()
    zone_convs = Dict()
    for (g, gen) in _PM.ref(pm, nw, :gen)
        if haskey(gen, "zone")
            zone = gen["zone"]
            if !haskey(generator_properties, zone)
                generator_properties[zone] = Dict()
            end
        push!(generator_properties[zone], g => Dict("inertia" => gen["inertia_constants"], "rating" => gen["pmax"]))
        end
    end

    for (s, storage) in _PM.ref(pm, nw, :storage)
        if haskey(storage, "zone")
            st_zone = storage["zone"]
            if !haskey(storage_properties, st_zone)
                storage_properties[st_zone] = Dict()
            end
        push!(storage_properties[st_zone], s => Dict("inertia" => storage["inertia_constants"], "rating" => storage["thermal_rating"]))
        end
    end

    for (c, conv) in _PM.ref(pm, nw, :convdc)
        if haskey(conv, "zone")
            zone = conv["zone"] 
            if !haskey(zone_convs, zone)
                zone_convs[zone] = Dict()
            end
            push!(zone_convs[zone], c => Dict("t_hvdc" => conv["t_hvdc"]))
        end
    end

    frequency_parameters = _PM.ref(pm, nw, :frequency_parameters)
    ΔTin = frequency_parameters["t_fcr"]
    ΔTdroop = frequency_parameters["t_fcrd"]
    fmin = frequency_parameters["fmin"]
    fmax = frequency_parameters["fmax"]
    f0 = frequency_parameters["f0"]
    Δfss = frequency_parameters["delta_fss"]
    if haskey(frequency_parameters, "fdb")
        fdb = frequency_parameters["fdb"]
    else
        fdb = 0
    end
    if haskey(_PM.ref(pm, nw), :excluded_zones)
        excluded_zones = _PM.ref(pm, nw, :excluded_zones)
    else
        excluded_zones = []
    end
    
    zones = [z["zone"] for (i,z) in _PM.ref(pm, nw, :zones) if !any(z["zone"] .== excluded_zones)]
    zone_ids = [i for (i,z) in _PM.ref(pm, nw, :zones) if !any(z["zone"] .== excluded_zones)]
    if any(zone .== zones)
        if haskey(generator_properties, zone)
            g_properties = generator_properties[zone]
        else
            g_properties = Dict()
        end
        if haskey(storage_properties, zone)
            s_properties = storage_properties[zone]
        else
            s_properties = 0
        end
        if haskey(zone_convs, zone)
            z_convs = zone_convs[zone]
        else
            z_convs = Dict()
        end
        
        constraint_frequency_storage_droop_lean(pm, nw, ref_id, g_properties, s_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, z_convs, hvdc_contribution, zone_id, zone_cont = true, direction = "plus")
        constraint_frequency_storage_droop_lean(pm, nw, ref_id, g_properties, s_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, z_convs, hvdc_contribution, zone_id, zone_cont = true, direction = "minus")
        for j in setdiff(zone_ids, zone_id)
            zone_ = _PM.ref(pm, nw, :zones, j)["zone"]
            if haskey(generator_properties, zone_)
                g_properties = generator_properties[zone_]
            else
                g_properties = Dict()
            end
            if haskey(storage_properties, zone_)
                s_properties = storage_properties[zone_]
            else
                s_properties = Dict()
            end
            if haskey(zone_convs, zone_)
                z_convs = zone_convs[zone_]
            else
                z_convs = Dict()
            end
            constraint_frequency_storage_droop_lean(pm, nw, ref_id, g_properties, s_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, z_convs, hvdc_contribution, j, zone_cont = false, direction = "plus")
            constraint_frequency_storage_droop_lean(pm, nw, ref_id, g_properties, s_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, z_convs, hvdc_contribution, j, zone_cont = false, direction = "minus")
        end
    end
end


function constraint_storage_contingency(pm::_PM.AbstractPowerModel; nw::Int = _PM.nw_id_default)
    reference_network_idx = get_reference_network_id(pm, nw)

    reference_network_idx = get_reference_network_id(pm, nw)
    if haskey(_PM.ref(pm, nw), :excluded_zones)
        excluded_zones = _PM.ref(pm, nw, :excluded_zones)
    else
        excluded_zones = []
    end
    zones = [i for i in _PM.ids(pm, nw, :zones) if !any(i .== excluded_zones)]
    for zone in zones
        for (s, storage) in _PM.ref(pm, nw, :storage)
            if haskey(storage, "zone") && _PM.ref(pm, nw, :zones, zone)["zone"] == storage["zone"]
                constraint_storage_contingency(pm, s, reference_network_idx, zone)
            end
        end
    end 
end

function constraint_storage_contingency_indicator(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    reference_network_idx = get_reference_network_id(pm, nw)

    bigM = 2 * maximum([storage["thermal_rating"] for (s, storage) in _PM.ref(pm, nw, :storage)])
    storage = _PM.ref(pm, nw, :storage, i)
    storage_zone = storage["zone"]
    if haskey(_PM.ref(pm, nw), :excluded_zones)
        excluded_zones = _PM.ref(pm, nw, :excluded_zones)
    else
        excluded_zones = []
    end
    zones = [i for i in _PM.ids(pm, nw, :zones) if !any(i .== excluded_zones)]
    for z in zones
        zone = _PM.ref(pm, nw, :zones, z)["zone"]
        if storage_zone == zone
            constraint_storage_contingency_indicator(pm, i, reference_network_idx, bigM, z)
        end
    end
end

function constraint_select_storage_contingency(pm::_PM.AbstractPowerModel; nw::Int = _PM.nw_id_default)
    reference_network_idx = get_reference_network_id(pm, nw)
    if haskey(_PM.ref(pm, nw), :excluded_zones)
        excluded_zones = _PM.ref(pm, nw, :excluded_zones)
    else
        excluded_zones = []
    end
    
    zones = [i for i in _PM.ids(pm, nw, :zones) if !any(i .== excluded_zones)]
    for zone in zones
        zone_storages = []
        for (s, storage) in _PM.ref(pm, nw, :storage)
            if haskey(storage, "zone") && _PM.ref(pm, nw, :zones, zone)["zone"] == storage["zone"]
                append!(zone_storages, s)
            end
        end
        constraint_select_storage_contingency(pm, reference_network_idx, zone_storages)
    end 
end

function constraint_storage_droop(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    storage = _PM.ref(pm, nw, :storage, i)
    ramp_rate = storage["ramp_rate_per_s"]

    ΔTin = _PM.ref(pm, nw, :frequency_parameters)["t_fcr"]
    ΔTdroop = _PM.ref(pm, nw, :frequency_parameters)["t_fcrd"]

    return constraint_storage_droop(pm, i, nw, ramp_rate, ΔTin, ΔTdroop)
end

function constraint_storage_droop_absolute(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    constraint_storage_droop_absolute(pm, i, nw)
end






function constraint_converter_outage(pm::_PM.AbstractPowerModel, i; nw::Int = _PM.nw_id_default)
    reference_network_idx = get_reference_network_id(pm, nw)
    prated = _PM.ref(pm, nw, :convdc, i)["Pacrated"]

    constraint_converter_outage(pm, nw, i, prated, reference_network_idx)
end