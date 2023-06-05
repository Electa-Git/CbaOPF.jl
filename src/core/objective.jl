function objective_min_fuel_cost(pm::_PM.AbstractPowerModel; kwargs...)
    model = _PM.check_gen_cost_models(pm)

    if model == 1
        return objective_min_fuel_cost_pwl(pm; kwargs...)
    elseif model == 2
        return objective_min_fuel_cost_polynomial(pm; kwargs...)
    else
        Memento.error(_LOGGER, "Only cost models of types 1 and 2 are supported at this time, given cost model type of $(model)")
    end

end


""
function objective_min_fuel_cost_polynomial(pm::_PM.AbstractPowerModel; kwargs...)
    order = _PM.calc_max_cost_index(pm.data)-1

    if order <= 2
        return _objective_min_fuel_cost_polynomial_linquad(pm; kwargs...)
    else
        return _objective_min_fuel_cost_polynomial_nl(pm; kwargs...)
    end
end

""
function _objective_min_fuel_cost_polynomial_linquad(pm::_PM.AbstractPowerModel; report::Bool=true)
   
    gen_cost = calc_gen_cost(pm)
    load_cost_red, load_cost_curt = calc_load_operational_cost(pm)

    return JuMP.@objective(pm.model, Min,
        sum( sum( gen_cost[(n,i)] for (i,gen) in nw_ref[:gen]) for (n, nw_ref) in _PM.nws(pm))
        + sum( sum( load_cost_curt[(n,i)] for (i,load) in nw_ref[:load]) for (n, nw_ref) in _PM.nws(pm))
        + sum( sum( load_cost_red[(n,i)] for (i,load) in nw_ref[:load]) for (n, nw_ref) in _PM.nws(pm))
    )
end

function objective_min_rd_cost(pm::_PM.AbstractPowerModel; report::Bool=true)
    gen_cost = Dict()

    for (n, nw_ref) in _PM.nws(pm)
        for (i,gen) in nw_ref[:gen]
            dpg_up = sum( _PM.var(pm, n, :dpg_up, i)[c] for c in _PM.conductor_ids(pm, n) )
            dpg_down = sum( _PM.var(pm, n, :dpg_down, i)[c] for c in _PM.conductor_ids(pm, n) )
            gen_cost[(n,i)] = gen["rdcost_up"][1] * dpg_up +  gen["rdcost_down"][1] * dpg_down
        end
    end

    load_cost_red, load_cost_curt = calc_load_operational_cost(pm)

    return JuMP.@objective(pm.model, Min,
        sum( sum( gen_cost[(n,i)] for (i,gen) in nw_ref[:gen] ) for (n, nw_ref) in _PM.nws(pm))
        + sum( sum( load_cost_curt[(n,i)] for (i,load) in nw_ref[:load]) for (n, nw_ref) in _PM.nws(pm))
        + sum( sum( load_cost_red[(n,i)] for (i,load) in nw_ref[:load]) for (n, nw_ref) in _PM.nws(pm))
    )
end


function objective_min_rd_cost_inertia(pm::_PM.AbstractPowerModel; report::Bool=true)
    gen_cost = Dict()

    for (n, nw_ref) in _PM.nws(pm)
        for (i,gen) in nw_ref[:gen]
            alpha_g =  _PM.var(pm, n, :alpha_g, i)
            gen_cost[(n,i)] = (alpha_g - gen["dispatch_status"]) * gen["start_up_cost"] * gen["pmax"] #+  (1 - alpha_g) * gen["rdcost_down"][1] * dpg_down
        end
    end

    load_cost_red, load_cost_curt = calc_load_operational_cost(pm)

    return JuMP.@objective(pm.model, Min,
        sum( sum( gen_cost[(n,i)] for (i,gen) in nw_ref[:gen]) for (n, nw_ref) in _PM.nws(pm)) 
        + sum( sum( load_cost_curt[(n,i)] for (i,load) in nw_ref[:load]) for (n, nw_ref) in _PM.nws(pm))
        + sum( sum( load_cost_red[(n,i)] for (i,load) in nw_ref[:load]) for (n, nw_ref) in _PM.nws(pm))
    )
end



function objective_min_cost_frequency(pm::_PM.AbstractPowerModel; report::Bool=true)
    gen_cost = Dict()

    for (n, nw_ref) in _PM.nws(pm)
        for (i,gen) in nw_ref[:gen]
            alpha_g =  _PM.var(pm, n, :alpha_g, i)
            pg =  _PM.var(pm, n, :pg, i)
            gen_cost[(n,i)] = (alpha_g * gen["start_up_cost"] * gen["pmax"]) + (gen["cost"][1]*pg + gen["cost"][2])  #+  (1 - alpha_g) * gen["rdcost_down"][1] * dpg_down
        end
    end

    load_cost_red, load_cost_curt = calc_load_operational_cost(pm)

    return JuMP.@objective(pm.model, Min,
        sum( sum( gen_cost[(n,i)] for (i,gen) in nw_ref[:gen]) for (n, nw_ref) in _PM.nws(pm)) 
        + sum( sum( load_cost_curt[(n,i)] for (i,load) in nw_ref[:load]) for (n, nw_ref) in _PM.nws(pm))
        + sum( sum( load_cost_red[(n,i)] for (i,load) in nw_ref[:load]) for (n, nw_ref) in _PM.nws(pm))
    )
end

function objective_min_fuel_and_capex_cost(pm::_PM.AbstractPowerModel; report::Bool=true)
    gen_cost = calc_gen_cost(pm)
    load_cost_red, load_cost_curt = calc_load_operational_cost(pm)
   
    branch_cost = Dict()
    for (n, nw_ref) in _PM.nws(pm)
        for (b,branch) in nw_ref[:branch]
            Δcap = sum( _PM.var(pm, n, :delta_cap, b)[c] for c in _PM.conductor_ids(pm, n) )
            branch_cost[(n,b)] = branch["capacity_cost"] * Δcap
        end
    end

    return JuMP.@objective(pm.model, Min,
        sum( sum( gen_cost[(n,i)] for (i,gen) in nw_ref[:gen] ) for (n, nw_ref) in _PM.nws(pm))
        + sum( sum( branch_cost[(n,b)] for (b,branch) in nw_ref[:branch]) for (n, nw_ref) in _PM.nws(pm))
        + sum( sum( load_cost_curt[(n,i)] for (i,load) in nw_ref[:load]) for (n, nw_ref) in _PM.nws(pm))
        + sum( sum( load_cost_red[(n,i)] for (i,load) in nw_ref[:load]) for (n, nw_ref) in _PM.nws(pm))
    )
end


function calc_gen_cost(pm::_PM.AbstractPowerModel; report::Bool=true)
    gen_cost = Dict()
    for (n, nw_ref) in _PM.nws(pm)
        for (i,gen) in nw_ref[:gen]
            pg = sum( _PM.var(pm, n, :pg, i)[c] for c in _PM.conductor_ids(pm, n) )

            if length(gen["cost"]) == 1
                gen_cost[(n,i)] = gen["cost"][1]
            elseif length(gen["cost"]) == 2
                gen_cost[(n,i)] = gen["cost"][1]*pg + gen["cost"][2]
            elseif length(gen["cost"]) == 3
                gen_cost[(n,i)] = gen["cost"][1]*pg^2 + gen["cost"][2]*pg + gen["cost"][3]
            else
                gen_cost[(n,i)] = 0.0
            end
        end
    end
    return gen_cost
end

function calc_load_operational_cost(pm::_PM.AbstractPowerModel)
    load_cost_red = Dict()
    load_cost_curt = Dict()
    for (n, nw_ref) in _PM.nws(pm)
        for (l, load) in nw_ref[:load]
            p_red = sum( _PM.var(pm, n, :pred, l)[c] for c in _PM.conductor_ids(pm, n) )
            p_curt = sum( _PM.var(pm, n, :pcurt, l)[c] for c in _PM.conductor_ids(pm, n) )
            load_cost_red[n, l] = load["cost_red"]  * p_red
            load_cost_curt[n, l] = load["cost_curt"] * p_curt
        end
    end

    return load_cost_red, load_cost_curt
end