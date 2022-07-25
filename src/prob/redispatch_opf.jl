function solve_rdopf(file::String, model_type::Type, optimizer; kwargs...)
    data = PowerModels.parse_file(file)
    _PMACDC.process_additional_data!(data)
    add_flexible_demand_data!(data)
    process_pst_data!(data)
    return _PM.solve_model(data, model_type, optimizer, build_rdopf; ref_extensions = [_PMACDC.add_ref_dcgrid!, ref_add_pst!, ref_add_flex_load!], kwargs...)
end

function solve_cbaopf(data::Dict{String,Any}, model_type::Type, optimizer; kwargs...)
    return _PM.solve_model(data, model_type, optimizer, build_rdopf; ref_extensions = [_PMACDC.add_ref_dcgrid!, ref_add_pst!, ref_add_flex_load!], kwargs...)
end

""
function build_rdopf(pm::_PM.AbstractPowerModel)
    _PM.variable_bus_voltage(pm)
    _PM.variable_gen_power(pm)
    _PM.variable_branch_power(pm)

    _PMACDC.variable_active_dcbranch_flow(pm)
    _PMACDC.variable_dcbranch_current(pm)
    _PMACDC.variable_dc_converter(pm)
    _PMACDC.variable_dcgrid_voltage_magnitude(pm)

    _PM.constraint_model_voltage(pm)
    _PMACDC.constraint_voltage_dc(pm)

    variable_pst(pm)
    variable_flexible_demand(pm)
    variable_gen_redispatch(pm)

    objective_min_rd_cost(pm)

    for i in _PM.ids(pm, :ref_buses)
        _PM.constraint_theta_ref(pm, i)
    end

    for i in _PM.ids(pm, :bus)
         constraint_power_balance_ac(pm, i)
    end

    for i in _PM.ids(pm, :branch)
        _PM.constraint_ohms_yt_from(pm, i)
        _PM.constraint_ohms_yt_to(pm, i)
        _PM.constraint_voltage_angle_difference(pm, i) #angle difference across transformer and reactor - useful for LPAC if available?
        _PM.constraint_thermal_limit_from(pm, i)
        _PM.constraint_thermal_limit_to(pm, i)
    end
    for i in _PM.ids(pm, :busdc)
        _PMACDC.constraint_power_balance_dc(pm, i)
    end
    for i in _PM.ids(pm, :branchdc)
        _PMACDC.constraint_ohms_dc_branch(pm, i)
    end
    for i in _PM.ids(pm, :convdc)
        _PMACDC.constraint_converter_losses(pm, i)
        _PMACDC.constraint_converter_current(pm, i)
        _PMACDC.constraint_conv_transformer(pm, i)
        _PMACDC.constraint_conv_reactor(pm, i)
        _PMACDC.constraint_conv_filter(pm, i)
        if pm.ref[:it][:pm][:nw][_PM.nw_id_default][:convdc][i]["islcc"] == 1
            _PMACDC.constraint_conv_firing_angle(pm, i)
        end
        if !haskey(pm.setting, "fix_converter_setpoints") && pm.setting["fix_converter_setpoints"] == true
            _PMACDC.constraint_active_conv_setpoint(pm, i)
        end
    end

    for i in _PM.ids(pm, :pst)
        constraint_ohms_y_from_pst(pm, i)
        constraint_ohms_y_to_pst(pm, i)
        constraint_limits_pst(pm, i)
    end

    for i in _PM.ids(pm, :flex_load)
        constraint_total_flexible_demand(pm, i)
    end


    for i in _PM.ids(pm, :gen)
        constraint_gen_redispatch(pm, i)
    end

    if pm.setting["fix_cross_border_flows"] == true
        if !haskey(pm.setting, "borders")
            borders = [i for i in _PM.ids(pm, :borders)]
        else
            borders = [i for i in pm.setting["borders"]]
        end
        for i in borders
            constraint_fixed_xb_flows(pm, i)
        end
    end
end
