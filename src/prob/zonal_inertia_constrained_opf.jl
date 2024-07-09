function solve_zonal_inertia_opf(data::Dict{String,Any}, model_type::Type, optimizer; kwargs...)
    return _PM.solve_model(data, model_type, optimizer, build_inertia_opf; kwargs...)
end

function build_inertia_opf(pm::_PM.AbstractPowerModel)
    _PM.variable_bus_voltage(pm)
    _PM.variable_gen_power(pm)
    _PM.variable_branch_power(pm)
    _PM.constraint_model_voltage(pm)
    _PM.variable_storage_power(pm; nw = n)

    _PM.objective_min_fuel_cost(pm)

    for i in _PM.ids(pm, :ref_buses)
        _PM.constraint_theta_ref(pm, i)
    end

    for i in _PM.ids(pm, :bus)
        _PM.constraint_power_balance(pm, i)
    end

    for i in _PM.ids(pm, :branch)
        _PM.constraint_ohms_yt_from(pm, i)
        _PM.constraint_ohms_yt_to(pm, i)
        _PM.constraint_voltage_angle_difference(pm, i)
        _PM.constraint_thermal_limit_from(pm, i)
        _PM.constraint_thermal_limit_to(pm, i)
    end

    if haskey(pm.setting, "inertia_limit") && pm.setting["inertia_limit"] == true
        zones = [i for i in _PM.ids(pm, :inertia_limit)]
        for i in zones
            constraint_inertia_limit(pm, i)
        end
    end    
end
