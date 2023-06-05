function solve_fsopf(data::Dict{String,Any}, model_type::Type, optimizer; kwargs...)
    return _PM.solve_model(data, model_type, optimizer, build_fsopf; ref_extensions = [_PMACDC.add_ref_dcgrid!, ref_add_flex_load!, ref_add_pst!], kwargs...)
end

""
function build_fsopf(pm::_PM.AbstractPowerModel)
    for (n, networks) in pm.ref[:it][:pm][:nw]
        _PM.variable_bus_voltage(pm; nw = n)
        _PM.variable_gen_power(pm; nw = n)
        _PM.variable_branch_power(pm; nw = n)

        _PMACDC.variable_active_dcbranch_flow(pm; nw = n)
        _PMACDC.variable_dcbranch_current(pm; nw = n)
        _PMACDC.variable_dc_converter(pm; nw = n)
        _PMACDC.variable_dcgrid_voltage_magnitude(pm; nw = n)

        _PM.constraint_model_voltage(pm; nw = n)
        _PMACDC.constraint_voltage_dc(pm; nw = n)

        variable_flexible_demand(pm; nw = n)
        variable_generator_state(pm; nw = n)
        variable_pst(pm; nw = n)
    end

    network_ids = sort(collect(_PM.nw_ids(pm)))
    n1 = network_ids[1]
    first_stage_model!(pm, n1)

    for n in network_ids[2:end]
        second_stage_model!(pm, n)
    end

    objective_min_cost_frequency(pm)
end


function first_stage_model!(pm, n)
    for i in _PM.ids(pm, n, :ref_buses)
        _PM.constraint_theta_ref(pm, i; nw = n)
    end

    for i in _PM.ids(pm, n, :bus)
        constraint_power_balance_ac(pm, i; nw = n)
    end
    
    for i in _PM.ids(pm, n, :branch)
        _PM.constraint_ohms_yt_from(pm, i; nw = n)
        _PM.constraint_ohms_yt_to(pm, i; nw = n)
        _PM.constraint_voltage_angle_difference(pm, i; nw = n)
        _PM.constraint_thermal_limit_from(pm, i; nw = n)
        _PM.constraint_thermal_limit_to(pm, i; nw = n)
    end
    for i in _PM.ids(pm, n, :busdc)
        _PMACDC.constraint_power_balance_dc(pm, i; nw = n)
    end
    for i in _PM.ids(pm, n, :branchdc)
        _PMACDC.constraint_ohms_dc_branch(pm, i; nw = n)
    end
    for i in _PM.ids(pm, n, :convdc)
        _PMACDC.constraint_converter_losses(pm, i; nw = n)
        _PMACDC.constraint_converter_current(pm, i; nw = n)
        _PMACDC.constraint_conv_transformer(pm, i; nw = n)
        _PMACDC.constraint_conv_reactor(pm, i; nw = n)
        _PMACDC.constraint_conv_filter(pm, i; nw = n)
        if pm.ref[:it][:pm][:nw][n][:convdc][i]["islcc"] == 1
            _PMACDC.constraint_conv_firing_angle(pm, i; nw = n)
        end
    end

    for i in _PM.ids(pm, n, :flex_load)
        constraint_total_flexible_demand(pm, i; nw = n)
    end

    for i in _PM.ids(pm, n, :gen)
        constraint_generator_on_off(pm, i; nw = n, use_status = false)
    end

    for i in _PM.ids(pm, n, :pst)
        constraint_ohms_y_from_pst(pm, i; nw = n)
        constraint_ohms_y_to_pst(pm, i; nw = n)
        constraint_limits_pst(pm, i; nw = n)
    end
end


function second_stage_model!(pm, n)
    for i in _PM.ids(pm, n, :gen)
        constraint_generator_on_off(pm, i; nw = n, use_status = false)
        constraint_generator_status(pm, i; nw = n)
    end

    zones = [i for i in _PM.ids(pm, n, :zones)]
    for i in zones
        constraint_frequency(pm, i; nw = n)
    end
end
