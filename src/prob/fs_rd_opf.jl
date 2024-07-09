function solve_fsrdopf(data::Dict{String,Any}, model_type::Type, optimizer; kwargs...)
    return _PM.solve_model(data, model_type, optimizer, build_fsrdopf; ref_extensions = [_PMACDC.add_ref_dcgrid!, ref_add_flex_load!, ref_add_pst!], kwargs...)
end

""
function build_fsrdopf(pm::_PM.AbstractPowerModel)
    for (n, networks) in pm.ref[:it][:pm][:nw]
        _PM.variable_bus_voltage(pm; nw = n)

        _PMACDC.variable_active_dcbranch_flow(pm; nw = n)
        _PMACDC.variable_dcbranch_current(pm; nw = n)
        _PMACDC.variable_dc_converter(pm; nw = n)
        _PMACDC.variable_dcgrid_voltage_magnitude(pm; nw = n)

        _PM.constraint_model_voltage(pm; nw = n)
        _PMACDC.constraint_voltage_dc(pm; nw = n)
        _PM.variable_storage_power(pm; nw = n)

        variable_inertia(pm; nw = n)
        variable_hvdc_contribution(pm; nw = n)
        if haskey(pm.setting, "use_droop") && pm.setting["use_droop"] == true
            variable_gen_contribution(pm; nw = n)
            _PM.variable_gen_power(pm; nw = n)
        end
    end

    for n in pm.ref[:it][:pm][:hour_ids]
        first_stage_model_rd_fs_opf!(pm, n)
    end

    if haskey(pm.setting, "use_droop") && pm.setting["use_droop"] == true
        for n in pm.ref[:it][:pm][:cont_ids]
            second_stage_model_uc_droop!(pm, n)
        end
        objective_min_rdcost_frequency(pm; droop = true)
    else
        for n in pm.ref[:it][:pm][:cont_ids]
            second_stage_model_uc!(pm, n)
        end
        objective_min_rdcost_frequency(pm; droop = false)
    end
end


function first_stage_model_rd_fs_opf!(pm, n)
    _PM.variable_branch_power(pm; nw = n)
    variable_gen_redispatch(pm; nw = n)
    variable_generator_states(pm; nw = n, uc = false)
    variable_flexible_demand(pm; nw = n)
    variable_pst(pm; nw = n)

    if !(haskey(pm.setting, "use_droop") && pm.setting["use_droop"] == true)
        _PM.variable_gen_power(pm; nw = n)
    end

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
    end

    for i in _PM.ids(pm, n, :flex_load)
        constraint_total_flexible_demand(pm, i; nw = n)
    end

    gen_status = haskey(pm.setting, "use_gen_status") && pm.setting["use_gen_status"] == true
    for i in _PM.ids(pm, n, :gen)
        constraint_gen_redispatch(pm, i, nw = n)
        constraint_generator_on_off(pm, i; nw = n, use_status = gen_status)
    end

    for i in _PM.ids(pm, n, :pst)
        constraint_ohms_y_from_pst(pm, i; nw = n)
        constraint_ohms_y_to_pst(pm, i; nw = n)
        constraint_limits_pst(pm, i; nw = n)
    end
end
