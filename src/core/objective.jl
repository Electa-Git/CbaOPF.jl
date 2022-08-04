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

    return JuMP.@objective(pm.model, Min,
        sum(
            sum(    gen_cost[(n,i)] for (i,gen) in nw_ref[:gen] )
        for (n, nw_ref) in _PM.nws(pm))
        
        + calc_load_operational_cost(pm)
    )
end


function calc_load_operational_cost(pm::_PM.AbstractPowerModel; n::Int=_PM.nw_id_default)
    cost = 0.0
    flex_load = _PM.ref(pm, n, :flex_load)
    if !isempty(flex_load)
        cost += sum(
            + l["cost_red"] * _PM.var(pm, n, :pred, i)
            + l["cost_curt"] * _PM.var(pm, n, :pcurt, i)
            for (i,l) in flex_load
        )
    end
    return cost
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

    return JuMP.@objective(pm.model, Min,
        sum(
            sum(    gen_cost[(n,i)] for (i,gen) in nw_ref[:gen] )
        for (n, nw_ref) in _PM.nws(pm))
        
        + calc_load_operational_cost(pm)
    )
end


function objective_min_rd_cost_inertia(pm::_PM.AbstractPowerModel; report::Bool=true)
    gen_cost = Dict()

    for (n, nw_ref) in _PM.nws(pm)
        for (i,gen) in nw_ref[:gen]
            dpg_up = sum( _PM.var(pm, n, :dpg_up, i)[c] for c in _PM.conductor_ids(pm, n) )
            dpg_down = sum( _PM.var(pm, n, :dpg_down, i)[c] for c in _PM.conductor_ids(pm, n) )
            alpha_g =  _PM.var(pm, n, :alpha_g, i)
            gen_cost[(n,i)] = (alpha_g - gen["dispatch_status"]) * gen["start_up_cost"] * gen["pmax"] #+  (1 - alpha_g) * gen["rdcost_down"][1] * dpg_down
        end
    end

    return JuMP.@objective(pm.model, Min,
        sum(
            sum(    gen_cost[(n,i)] for (i,gen) in nw_ref[:gen] )
        for (n, nw_ref) in _PM.nws(pm))
        
        + calc_load_operational_cost(pm)
    )
end