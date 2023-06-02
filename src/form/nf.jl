function constraint_variable_ntc_from(pm::_PM.AbstractAPLossLessModels, n::Int, f_idx, pmax)
    l,i,j = f_idx
    p_fr = _PM.var(pm, n, :p, (l,i,j))
    delta_cap = _PM.var(pm, n, :delta_cap, l)

    JuMP.@constraint(pm.model, p_fr <=  (pmax + delta_cap))
end


function constraint_variable_ntc_to(pm::_PM.AbstractAPLossLessModels, n::Int, t_idx, pmax)
    l,i,j = t_idx
    p_to = _PM.var(pm, n, :p, (l,i,j))
    delta_cap = _PM.var(pm, n, :delta_cap, l)

    JuMP.@constraint(pm.model, p_to <=  (pmax + delta_cap))
end


function constraint_ntc_capacity(pm::_PM.AbstractAPLossLessModels, i::Int, n, n_1)
    delta_cap_n = _PM.var(pm, n, :delta_cap, i)
    delta_cap_n_1 = _PM.var(pm, n_1, :delta_cap, i)

    JuMP.@constraint(pm.model, delta_cap_n == delta_cap_n_1)
end

function constraint_total_flexible_demand(pm::_PM.AbstractAPLossLessModels, n::Int, i, pd, pf_angle)
    pflex       = _PM.var(pm, n, :pflex, i)
    pcurt       = _PM.var(pm, n, :pcurt, i)
    pred        = _PM.var(pm, n, :pred, i)

    # Active power demand is the reference demand `pd` plus the contributions from all the demand flexibility decision variables
    JuMP.@constraint(pm.model, pflex == pd - pcurt - pred)
end