function solve_fsuc_lean(data::Dict{String,Any}, model_type::Type, optimizer; kwargs...)
    return _PM.solve_model(data, model_type, optimizer, build_fsuc_lean; ref_extensions = [_PMACDC.add_ref_dcgrid!, ref_add_flex_load!, ref_add_pst!], kwargs...)
end

""
function build_fsuc_lean(pm::_PM.AbstractPowerModel)

    for (n, networks) in pm.ref[:it][:pm][:nw]
        _PM.variable_bus_voltage(pm; nw = n)

        _PMACDC.variable_active_dcbranch_flow(pm; nw = n)
        _PMACDC.variable_dcbranch_current(pm; nw = n)
        _PMACDC.variable_dc_converter(pm; nw = n)
        _PMACDC.variable_dcgrid_voltage_magnitude(pm; nw = n)

        _PM.constraint_model_voltage(pm; nw = n)
        _PMACDC.constraint_voltage_dc(pm; nw = n)

        variable_inertia(pm; nw = n)
        variable_hvdc_contribution(pm; nw = n)
        variable_gen_contribution(pm; nw = n)
        _PM.variable_gen_power(pm; nw = n)
    end

    for n in pm.ref[:it][:pm][:hour_ids]
        fc_model_uc!(pm, n)
    end

    for n in pm.ref[:it][:pm][:cont_ids]
        contingency_contraints!(pm, n)
    end

    objective_min_cost_frequency_uc(pm; droop = true)    
end


function fc_model_uc!(pm, n)
    variable_contingencies(pm, nw = n)
    _PM.variable_branch_power(pm; nw = n)
    variable_generator_states(pm; nw = n, uc = true)
    variable_flexible_demand(pm; nw = n)
    variable_pst(pm; nw = n)
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
        constraint_generator_on_off(pm, i; nw = n, use_status = gen_status)
        contstraint_unit_commitment(pm, i; nw = n)
    end

    for i in _PM.ids(pm, n, :pst)
        constraint_ohms_y_from_pst(pm, i; nw = n)
        constraint_ohms_y_to_pst(pm, i; nw = n)
        constraint_limits_pst(pm, i; nw = n)
    end

    constraint_generator_contingency(pm; nw = n)
    constraint_select_generator_contingency(pm; nw = n)
    for i in _PM.ids(pm, n, :gen)
        constraint_generator_contingency_indicator(pm, i; nw = n)
    end

    constraint_converter_contingency(pm; nw = n)
    constraint_select_converter_contingency(pm; nw = n)
    for i in _PM.ids(pm, n, :convdc)
        constraint_converter_contingency_indicator(pm, i; nw = n)
    end

    # constraint_tieline_contingency(pm; nw = n)
    # constraint_select_tieline_contingency(pm; nw = n)
    # for i in _PM.ids(pm, n, :convdc)
    #     constraint_tieline_contingency_indicator(pm, i; nw = n)
    # end

end


function contingency_contraints!(pm, n)
    rn_idx = (n - get_reference_network_id(pm, n)) # get the reference network id
    cont_type = ["gen" "conv" "tie_line"][rn_idx]
     
    if haskey(pm.setting, "hvdc_inertia_contribution") && pm.setting["hvdc_inertia_contribution"] == true
    # converter equations to make sure that we have power balance....
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
            constraint_converter_power_balance(pm, i; nw = n) 
        end
    end
    
    gen_status = haskey(pm.setting, "use_gen_status") && pm.setting["use_gen_status"] == true
    for i in _PM.ids(pm, n, :gen)
        constraint_generator_on_off(pm, i; nw = n, use_status = gen_status, second_stage = true)
        constraint_gen_droop_power_balance(pm, i; nw = n)
        constraint_generator_droop(pm, i; nw = n)
    end

    for i in _PM.ids(pm, n, :convdc)
        constraint_converter_contribution_absolute(pm, i; nw = n)
    end

    for i in _PM.ids(pm, n, :gen)
        constraint_generator_droop_absolute(pm, i; nw = n)
    end

    # for i in _PM.ids(pm, n, :convdc)
    #     constraint_converter_contribution_absolute(pm, i; nw = n)
    #     constraint_converter_power_balance(pm, i; nw = n)
    # #     constraint_converter_contingency(pm, i; nw = n)
    # #     constraint_converter_contingency_indicator(pm, i; nw = n)
    # end

    # for i in _PM.ids(pm, n, :tie_lines)
    #     constraint_branch_contingency(pm, i; nw = n)
    #     constraint_branch_contingency_indicator(pm, i; nw = n)
    # end

    
    # constraint_select_converter_contingency(pm, n)
    # constraint_select_branch_contingency(pm, n)
    if cont_type == "gen"
        if haskey(pm.setting, "hvdc_inertia_contribution") && pm.setting["hvdc_inertia_contribution"] == true
            constraint_frequency_droop_lean(pm; nw = n, hvdc_contribution = true)
        else
            constraint_frequency_droop_lean(pm; nw = n, hvdc_contribution = false)
        end
    end

    if cont_type == "conv"
        if haskey(pm.setting, "hvdc_inertia_contribution") && pm.setting["hvdc_inertia_contribution"] == true
            constraint_frequency_converter_droop_lean(pm; nw = n, hvdc_contribution = true)
        else
            constraint_frequency_converter_droop_lean(pm; nw = n, hvdc_contribution = false)
        end
    end
end