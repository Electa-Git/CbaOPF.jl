function solve_zonal_tnep(data::Dict{String,Any}, model_type::Type, optimizer; kwargs...)
    return _PM.solve_model(data, model_type, optimizer, build_zonal_tnep; ref_extensions = [ref_add_flex_load!,  ref_add_storage!], kwargs...)
end

function build_zonal_tnep(pm::_PM.AbstractPowerModel)
    for (n, networks) in pm.ref[:it][:pm][:nw]
        _PM.variable_bus_voltage(pm; nw = n)
        _PM.variable_gen_power(pm; nw = n)
        _PM.variable_branch_power(pm; nw = n, bounded = false)
        _PM.constraint_model_voltage(pm; nw = n)
        variable_branch_capacity(pm; nw = n)
        variable_flexible_demand(pm; nw = n)
        variable_storage(pm, nw = n)
    end

    for (n, networks) in pm.ref[:it][:pm][:nw]
        for i in _PM.ids(pm, n, :ref_buses)
            _PM.constraint_theta_ref(pm, i; nw = n)
        end

        for i in _PM.ids(pm, n, :bus)
            constraint_power_balance(pm, i; nw = n)
        end

        for i in _PM.ids(pm, n, :branch)
            _PM.constraint_ohms_yt_from(pm, i; nw = n)
            _PM.constraint_ohms_yt_to(pm, i; nw = n)
            _PM.constraint_voltage_angle_difference(pm, i; nw = n)
            constraint_variable_branch_capacity_from(pm, i; nw = n)
            constraint_variable_branch_capacity_to(pm, i; nw = n)
        end

        for i in _PM.ids(pm, n, :load)
            constraint_total_flexible_demand(pm, i; nw = n)
        end

        for i in _PM.ids(pm, :storage_simple, nw = n)
            constraint_net_storage_injection(pm, i, nw = n)
        end
    end

    network_ids = sort(collect(_PM.nw_ids(pm)))
    for n in network_ids[2:end]
        for i in _PM.ids(pm, :branch, nw = n)
            constraint_branch_capacity(pm, i, nw = n)
        end
    end

    for i in _PM.ids(pm, :storage_simple, nw = 1)
        constraint_storage_initial_state(pm, i, nw = 1)
    end

    for n_2 in network_ids[2:end]
        for i in _PM.ids(pm, :storage_simple, nw = n_2)
            constraint_storage_state(pm, i, nw = n_2)
        end
    end

    objective_min_fuel_and_capex_cost(pm)

    # if haskey(pm.setting, "inertia_limit") && pm.setting["inertia_limit"] == true
    #     zones = [i for i in _PM.ids(pm, :inertia_limit)]
    #     for i in zones
    #         constraint_inertia_limit(pm, i)
    #     end
    # end
end