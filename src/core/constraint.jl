function constraint_inertia_limit(pm::_PM.AbstractDCPModel, n::Int, generator_properties, inertia_limit)
    alpha_g = _PM.var(pm, n, :alpha_g)

    JuMP.@constraint(pm.model, sum([properties["inertia"] * properties["rating"] * alpha_g[g] / 0.9 for (g, properties) in generator_properties])  >= inertia_limit) #
    #JuMP.@constraint(pm.model, sum([pg[g] * iner_g  for (g, iner_g) in gen_inertia]) - sum([pconv_grid_ac[c] * iner_c  for (c, iner_c) in conv_inertia])  >= inertia_limit) #
end

function constraint_inertia_limit(pm::_PM.AbstractNFAModel, n::Int, generator_properties, inertia_limit)
    pg = _PM.var(pm, n, :pg)
    if !isempty(generator_properties)
        JuMP.@constraint(pm.model, sum([properties["inertia"] * pg[g] / 0.9 for (g, properties) in generator_properties])  >= inertia_limit)
    end #
    #JuMP.@constraint(pm.model, sum([pg[g] * iner_g  for (g, iner_g) in gen_inertia]) - sum([pconv_grid_ac[c] * iner_c  for (c, iner_c) in conv_inertia])  >= inertia_limit) #
end

function constraint_active_conv_setpoint(pm::_PM.AbstractPowerModel, n::Int, i, pconv)
    pconv_var = _PM.var(pm, n, :pconv_tf_fr, i)

    if pconv >= 0
        JuMP.@constraint(pm.model, pconv_var >= - pconv - pconv/10)
        JuMP.@constraint(pm.model, pconv_var <= + pconv + pconv/10)
    else
        JuMP.@constraint(pm.model, pconv_var >= + pconv + pconv/10)
        JuMP.@constraint(pm.model, pconv_var <= - pconv - pconv/10)
    end
end

# Calucating the net storage injection
function constraint_net_storage_injection(pm::_PM.AbstractPowerModel, i::Int; nw::Int = _PM.nw_id_default)
    ps = _PM.var(pm, nw, :ps, i)
    ps_ch = _PM.var(pm, nw, :ps_ch, i)
    ps_dch = _PM.var(pm, nw, :ps_dch, i)

    JuMP.@constraint(pm.model, ps == ps_dch - ps_ch)
end

# Initial state of charge of storage
function constraint_storage_initial_state(pm::_PM.AbstractPowerModel, n::Int, i::Int, energy, charge_eff, discharge_eff, time_elapsed)
    es = _PM.var(pm, n, :es, i)
    ps_ch = _PM.var(pm, n, :ps_ch, i)
    ps_dch = _PM.var(pm, n, :ps_dch, i)

    JuMP.@constraint(pm.model, es - energy == time_elapsed*(charge_eff*ps_ch - ps_dch/discharge_eff))
end

# State of charge of storage
function constraint_storage_state(pm::_PM.AbstractPowerModel, n::Int, i::Int, charge_eff, discharge_eff, time_elapsed)
    es = _PM.var(pm, n, :es, i)
    es_t_1 = _PM.var(pm, n-1, :es, i)
    ps_ch = _PM.var(pm, n, :ps_ch, i)
    ps_dch = _PM.var(pm, n, :ps_dch, i)

    JuMP.@constraint(pm.model, es - es_t_1 == time_elapsed*(charge_eff*ps_ch - ps_dch/discharge_eff))
end

function constraint_power_balance(pm::_PM.AbstractAPLossLessModels, n::Int, i::Int, bus_arcs, bus_gens, bus_loads, bus_storage)
    p    = _PM.var(pm, n, :p)
    pg   = _PM.var(pm, n,   :pg)
    ps   = _PM.var(pm, n,   :ps)
    pflex = _PM.var(pm, n, :pflex)


    cstr_p = JuMP.@constraint(pm.model,
        sum(p[a] for a in bus_arcs)
        ==
        sum(pg[g] for g in bus_gens)
        + sum(ps[s] for s in bus_storage) 
        - sum(pflex[d] for d in bus_loads)
    )

    if _IM.report_duals(pm)
        _PM.sol(pm, n, :bus, i)[:lam_kcl_r] = cstr_p
    end
end

function constraint_branch_capacity(pm::_PM.AbstractPowerModel, i::Int, n, n_1)
    delta_cap_n = _PM.var(pm, n, :delta_cap, i)
    delta_cap_n_1 = _PM.var(pm, n_1, :delta_cap, i)

    JuMP.@constraint(pm.model, delta_cap_n == delta_cap_n_1)
end

function constraint_frequency(pm::_PM.AbstractPowerModel, n::Int, ref_id, generator_properties, gcont, ΔT, f0, fmin, fdb, fmax, zone_convs, hvdc_contribution, zone)
    ΔPg = _PM.var(pm, ref_id, :pg)[gcont]
    alpha_g = _PM.var(pm, ref_id, :alpha_g)
    pconv_in = _PM.var(pm, n, :pconv_in)
    htot = _PM.var(pm, n, :htot, zone)
    dc_contr = _PM.var(pm, n, :dc_contr, zone)

    if isempty(zone_convs) || hvdc_contribution == false
        dc_contribution = 0
    else
        dc_contribution = calculate_hvdc_contribution(pconv_in, ΔT, zone_convs)
    end

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * ((f0-fdb) - fmin)) >= 
     f0 * (ΔPg *  ΔT - dc_contribution)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmax - (f0+fdb))) >= 
     - f0 * (ΔPg *  ΔT - dc_contribution)
    )

    JuMP.@constraint(pm.model, htot == sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]))
    JuMP.@constraint(pm.model, dc_contr == dc_contribution)
end

function constraint_frequency(pm::_PM.AbstractPowerModel, n::Int, ref_id, generator_properties, ΔT, f0, fmin, fmax, fdb,  zone_convs, hvdc_contribution, zone)
    alpha_g = _PM.var(pm, ref_id, :alpha_g)
    pconv_in = _PM.var(pm, n, :pconv_in)
    htot = _PM.var(pm, n, :htot, zone)
    dc_contr = _PM.var(pm, n, :dc_contr, zone)

    if isempty(zone_convs) || hvdc_contribution == false
        dc_contribution = 0
    else
        dc_contribution = calculate_hvdc_contribution(pconv_in, ΔT, zone_convs)
    end

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * ((f0-fdb) - fmin)) >= 
    - f0 * dc_contribution
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmax - (f0+fdb))) >= 
      f0 * dc_contribution
    )

    JuMP.@constraint(pm.model, htot == sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]))
    JuMP.@constraint(pm.model, dc_contr == dc_contribution)
end

function constraint_frequency_tie_line(pm::_PM.AbstractPowerModel, n::Int, ref_id, generator_properties, ΔT, f0, fmin, fmax, fdb, zone_convs, hvdc_contribution, br_idx, area)
    ΔP = _PM.var(pm, ref_id, :p)[br_idx]
    alpha_g = _PM.var(pm, ref_id, :alpha_g)
    pconv_in = _PM.var(pm, n, :pconv_in)
    htot = _PM.var(pm, n, :htot_area, area)
    dc_contr = _PM.var(pm, n, :dc_contr_area, area)

    if isempty(zone_convs) || hvdc_contribution == false
        dc_contribution = 0
    else
        dc_contribution = calculate_hvdc_contribution(pconv_in, ΔT, zone_convs)
    end

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * ((f0-fdb) - fmin)) >= 
     f0 * (-ΔP *  ΔT - dc_contribution)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmax - (f0+fdb))) >= 
     -f0 * (-ΔP *  ΔT - dc_contribution)
    )

    JuMP.@constraint(pm.model, htot == sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]))
    JuMP.@constraint(pm.model, dc_contr == dc_contribution)
end

function constraint_frequency_converter(pm::_PM.AbstractPowerModel, n::Int, ref_id, generator_properties, ccont, ΔT, f0, fmin, fmax, fdb, zone_convs, hvdc_contribution, zone)
    ΔPc = - _PM.var(pm, ref_id, :pconv_ac)[ccont]
    alpha_g = _PM.var(pm, ref_id, :alpha_g)
    pconv_in = _PM.var(pm, n, :pconv_in)
    htot = _PM.var(pm, n, :htot, zone)
    dc_contr = _PM.var(pm, n, :dc_contr, zone)

    if isempty(zone_convs) || hvdc_contribution == false
        dc_contribution = 0
    else
        dc_contribution = calculate_hvdc_contribution(pconv_in, ΔT, zone_convs)
    end

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * ((f0-fdb) - fmin)) >= 
     f0 * (ΔPc *  ΔT - dc_contribution)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmax - (f0+fdb))) >= 
     -f0 * (ΔPc *  ΔT - dc_contribution)
    )

    JuMP.@constraint(pm.model, htot == sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]))
    JuMP.@constraint(pm.model, dc_contr == dc_contribution)
end

function constraint_frequency_converter(pm::_PM.AbstractPowerModel, n::Int, ref_id, generator_properties, ΔT, f0, fmin, fmax, fdb, zone_convs, hvdc_contribution, zone)
    alpha_g = _PM.var(pm, ref_id, :alpha_g)
    pconv_in = _PM.var(pm, n, :pconv_in)
    htot = _PM.var(pm, n, :htot, zone)
    dc_contr = _PM.var(pm, n, :dc_contr, zone)

    if isempty(zone_convs) || hvdc_contribution == false
        dc_contribution = 0
    else
        dc_contribution = calculate_hvdc_contribution(pconv_in, ΔT, zone_convs)
    end

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * ((f0-fdb) - fmin)) >= 
     f0 * (- dc_contribution)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmax - (f0+fdb))) >= 
     -f0 * (- dc_contribution)
    )

    JuMP.@constraint(pm.model, htot == sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]))
    JuMP.@constraint(pm.model, dc_contr == dc_contribution)
end

function calculate_hvdc_contribution(pconv_in, ΔT, zone_convs)
    sum([(pconv_in[c] * min(1, ΔT / conv["t_hvdc"]) * min(ΔT, conv["t_hvdc"]) / 2 ) + (pconv_in[c] * max(0, ΔT - conv["t_hvdc"])) for (c, conv) in zone_convs])
end

function constraint_generator_status(pm::_PM.AbstractPowerModel, n::Int, i::Int)
    alpha_n = _PM.var(pm, n, :alpha_g, i)
    alpha_n_1 = _PM.var(pm, n-1, :alpha_g, i)

    JuMP.@constraint(pm.model, alpha_n == alpha_n_1)
end

function constraint_generator_status_cont(pm::_PM.AbstractPowerModel, n::Int, i::Int, ref_id)
    alpha_n = _PM.var(pm, n, :alpha_g, i)
    alpha_n_1 = _PM.var(pm, ref_id, :alpha_g, i)

    JuMP.@constraint(pm.model, alpha_n == alpha_n_1)
end


function constraint_generator_status_uc(pm::_PM.AbstractPowerModel, n::Int, i::Int, prev_hour)
    alpha_n = _PM.var(pm, n, :alpha_g, i)
    alpha_n_1 = _PM.var(pm, prev_hour, :alpha_g, i)

    JuMP.@constraint(pm.model, alpha_n == alpha_n_1)
end


function constraint_generator_decisions(pm::_PM.AbstractPowerModel, i::Int, n::Int, prev_hour)
    alpha_n = _PM.var(pm, n, :alpha_g, i)
    alpha_n_1 = _PM.var(pm, prev_hour, :alpha_g, i)
    beta_n = _PM.var(pm, n, :beta_g, i)
    gamma_n = _PM.var(pm, n, :gamma_g, i)

    JuMP.@constraint(pm.model, alpha_n_1 - alpha_n + beta_n - gamma_n == 0)
end

function constraint_initial_generator_decisions(pm::_PM.AbstractPowerModel, i::Int, n::Int)
    alpha_n = _PM.var(pm, n, :alpha_g, i)
    beta_n = _PM.var(pm, n, :beta_g, i)
    gamma_n = _PM.var(pm, n, :gamma_g, i)

    JuMP.@constraint(pm.model, - alpha_n + beta_n - gamma_n == 0)
end

function constraint_minimum_up_time(pm::_PM.AbstractPowerModel, i::Int, n::Int, τ)
    alpha_n = _PM.var(pm, n, :alpha_g, i)

    JuMP.@constraint(pm.model, alpha_n >= sum([_PM.var(pm, t, :beta_g, i) for t in τ]))
end

function constraint_minimum_down_time(pm::_PM.AbstractPowerModel, i::Int, n::Int, τ)
    alpha_n = _PM.var(pm, n, :alpha_g, i)

    JuMP.@constraint(pm.model, (1 - alpha_n) >= sum([_PM.var(pm, t, :gamma_g, i) for t in τ]))
end

function constraint_generator_ramping(pm::_PM.AbstractPowerModel, i::Int, n::Int, prev_hour, ΔPg_up, ΔPg_down, pmin)
    pg_n = _PM.var(pm, n, :pg, i)
    pg_n_1 = _PM.var(pm, prev_hour, :pg, i)
    alpha_n = _PM.var(pm, n, :alpha_g, i)
    beta_n = _PM.var(pm, n, :beta_g, i)
    gamma_n = _PM.var(pm, n, :gamma_g, i)

    JuMP.@constraint(pm.model, pg_n - pg_n_1 <= ΔPg_up * alpha_n + (pmin - ΔPg_up) * beta_n)
    JuMP.@constraint(pm.model, pg_n_1 - pg_n <= ΔPg_down * alpha_n + pmin * gamma_n)
end

function  constraint_unit_commitment_reserves(pm::_PM.AbstractPowerModel, i::Int, n::Int)
    pg = _PM.var(pm, n, :pg, i)
  
    JuMP.@constraint(pm.model, sum([gen["pmax"] * _PM.var(pm, n, :alpha_g, g) for (g, gen) in _PM.ref(pm, n, :gen)]) >= pg)
end

function constraint_converter_power_balance(pm::_PM.AbstractPowerModel, i::Int, n::Int, reference_network_idx)
    pconv = _PM.var(pm, n, :pconv_ac, i)
    pconv_ref = _PM.var(pm, reference_network_idx, :pconv_ac, i)
    pconv_in = _PM.var(pm, n, :pconv_in, i)

    JuMP.@constraint(pm.model, pconv == pconv_ref - pconv_in)
end

function  constraint_converter_contribution_absolute(pm::_PM.AbstractPowerModel, i::Int, n::Int)
    pconv_in = _PM.var(pm, n, :pconv_in, i)
    pconv_in_abs = _PM.var(pm, n, :pconv_in_abs, i)

    JuMP.@constraint(pm.model, pconv_in_abs >=  pconv_in)
    JuMP.@constraint(pm.model, pconv_in_abs >= -pconv_in)
end


function constraint_frequency_droop(pm::_PM.AbstractPowerModel, n::Int, ref_id, generator_properties, ΔPg, gcont, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, zone_convs, hvdc_contribution, zone)
    ΔPg = _PM.var(pm, ref_id, :pg)[gcont]
    alpha_g = _PM.var(pm, ref_id, :alpha_g)
    pconv_in = _PM.var(pm, n, :pconv_in)
    pg_droop = _PM.var(pm, n, :pg_droop)
    htot = _PM.var(pm, n, :htot, zone)
    dc_contr = _PM.var(pm, n, :dc_contr, zone)

    if isempty(zone_convs) || hvdc_contribution == false
        dc_contribution_in = 0
        dc_contribution_droop = 0
        # ΔTin = fdb / (ΔPg *  f0)
        # print(zone, " ", ΔTin, "\n")
    else
        dc_contribution_in = calculate_hvdc_contribution(pconv_in, ΔTin, zone_convs)
        dc_contribution_droop = calculate_hvdc_contribution_droop(pconv_in, ΔTin, ΔTdroop, zone_convs)
    end

    gen_contribution_droop = calculate_gen_contribution_droop(pg_droop, ΔTin, ΔTdroop, generator_properties)

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmin - (f0-fdb))) <= 
     - f0 * (ΔPg *  ΔTin - dc_contribution_in)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmax - (f0+fdb))) >= 
     - f0 * (ΔPg *  ΔTin - dc_contribution_in)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmin - (f0-fdb))) <= 
     - f0 * (ΔPg * ΔTdroop - dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    )


    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmax - (f0+fdb))) >= 
     - f0 * (ΔPg * ΔTdroop - dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    )

    # JuMP.@constraint(pm.model, 
    # sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (-Δfss)) <= 
    #  - f0 * (ΔPg * ΔTdroop - dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    # )


    # JuMP.@constraint(pm.model, 
    # sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (Δfss)) >= 
    #  - f0 * (ΔPg * ΔTdroop - dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    # )

    JuMP.@constraint(pm.model, htot == sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]))
    JuMP.@constraint(pm.model, dc_contr == dc_contribution_in + dc_contribution_droop)
end


function constraint_frequency_droop(pm::_PM.AbstractPowerModel, n::Int, ref_id, generator_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, zone_convs, hvdc_contribution, zone)
    alpha_g = _PM.var(pm, ref_id, :alpha_g)
    pconv_in = _PM.var(pm, n, :pconv_in)
    pg_droop = _PM.var(pm, n, :pg_droop)
    htot = _PM.var(pm, n, :htot, zone)
    dc_contr = _PM.var(pm, n, :dc_contr, zone)

    if isempty(zone_convs) || hvdc_contribution == false
        dc_contribution_in = 0
        dc_contribution_droop = 0
    else
        dc_contribution_in = calculate_hvdc_contribution(pconv_in, ΔTin, zone_convs)
        dc_contribution_droop = calculate_hvdc_contribution_droop(pconv_in, ΔTin, ΔTdroop, zone_convs)
    end

    gen_contribution_droop = calculate_gen_contribution_droop(pg_droop, ΔTin, ΔTdroop, generator_properties)

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmin - (f0-fdb))) <= 
     - f0 * (- dc_contribution_in)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmax - (f0+fdb))) >= 
     - f0 * (- dc_contribution_in)
    )


    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmin - (f0-fdb))) <=  
     -f0 * ((- dc_contribution_in) + (- dc_contribution_droop - gen_contribution_droop))
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmax - (f0+fdb))) >= 
     - f0 * ((- dc_contribution_in) + (- dc_contribution_droop - gen_contribution_droop))
    )

    # JuMP.@constraint(pm.model, 
    # sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (-2 * Δfss) <=  
    #  -f0 * ((- dc_contribution_in) + (- dc_contribution_droop - gen_contribution_droop))
    # )

    # JuMP.@constraint(pm.model, 
    # sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * Δfss) >= 
    #  - f0 * ((- dc_contribution_in) + (- dc_contribution_droop - gen_contribution_droop))
    # )

    JuMP.@constraint(pm.model, htot == sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]))
    JuMP.@constraint(pm.model, dc_contr == dc_contribution_in + dc_contribution_droop)
end

function constraint_frequency_tie_line_droop(pm::_PM.AbstractPowerModel, n::Int, ref_id, generator_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, zone_convs, hvdc_contribution, br_idx, area)
    ΔP = _PM.var(pm, ref_id, :p)[br_idx]
    alpha_g = _PM.var(pm, ref_id, :alpha_g)
    pconv_in = _PM.var(pm, n, :pconv_in)
    pg_droop = _PM.var(pm, n, :pg_droop)
    htot = _PM.var(pm, n, :htot_area, area)
    dc_contr = _PM.var(pm, n, :dc_contr_area, area)

    if isempty(zone_convs) || hvdc_contribution == false
        dc_contribution_in = 0
        dc_contribution_droop = 0
    else
        dc_contribution_in = calculate_hvdc_contribution(pconv_in, ΔTin, zone_convs)
        dc_contribution_droop = calculate_hvdc_contribution_droop(pconv_in, ΔTin, ΔTdroop, zone_convs)
    end

    gen_contribution_droop = calculate_gen_contribution_droop(pg_droop, ΔTin, ΔTdroop, generator_properties)

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmin - (f0-fdb))) <= 
     - f0 * (-ΔP *  ΔTin - dc_contribution_in)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmax - (f0+fdb))) >= 
     -f0 * (-ΔP *  ΔTin - dc_contribution_in)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmin - (f0-fdb))) <= 
     - f0 * (-ΔP * ΔTdroop - dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmax - (f0+fdb))) >= 
    - f0 * (-ΔP * ΔTdroop - dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    ) 

    # JuMP.@constraint(pm.model, 
    # sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (-Δfss)) <= 
    #  - f0 * (-ΔP * ΔTdroop - dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    # )

    # JuMP.@constraint(pm.model, 
    # sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (Δfss)) >= 
    # - f0 * (-ΔP * ΔTdroop - dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    # ) 

    JuMP.@constraint(pm.model, htot == sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]))
    JuMP.@constraint(pm.model, dc_contr == dc_contribution_in + dc_contribution_droop)
end

function constraint_frequency_converter_droop(pm::_PM.AbstractPowerModel, n::Int, ref_id, generator_properties, ccont, ΔTin,  ΔTdroop, f0, fmin, fmax, fdb, Δfss, zone_convs, hvdc_contribution, zone)
    ΔPc = - _PM.var(pm, ref_id, :pconv_ac)[ccont]
    alpha_g = _PM.var(pm, ref_id, :alpha_g)
    pconv_in = _PM.var(pm, n, :pconv_in)
    pg_droop = _PM.var(pm, n, :pg_droop)
    htot = _PM.var(pm, n, :htot, zone)
    dc_contr = _PM.var(pm, n, :dc_contr, zone)

    if isempty(zone_convs) || hvdc_contribution == false
        dc_contribution_in = 0
        dc_contribution_droop = 0
    else
        dc_contribution_in = calculate_hvdc_contribution(pconv_in, ΔTin, zone_convs)
        dc_contribution_droop = calculate_hvdc_contribution_droop(pconv_in, ΔTin, ΔTdroop, zone_convs)
    end

    gen_contribution_droop = calculate_gen_contribution_droop(pg_droop, ΔTin, ΔTdroop, generator_properties)

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmin - (f0-fdb))) <= 
     - f0 * (ΔPc *  ΔTin - dc_contribution_in)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmax - (f0+fdb))) >= 
     - f0 * (ΔPc *  ΔTin - dc_contribution_in)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmin - (f0-fdb))) <=  
     - f0 * (ΔPc * ΔTdroop - dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmax - (f0+fdb))) >= 
     - f0 * (ΔPc * ΔTdroop - dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    ) 

    # JuMP.@constraint(pm.model, 
    # sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (-Δfss)) <=  
    #  - f0 * (ΔPc * ΔTdroop - dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    # )

    # JuMP.@constraint(pm.model, 
    # sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (Δfss)) >= 
    #  - f0 * (ΔPc * ΔTdroop - dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    # )

    JuMP.@constraint(pm.model, htot == sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]))
    JuMP.@constraint(pm.model, dc_contr == dc_contribution_in + dc_contribution_droop)
end

function constraint_frequency_converter_droop(pm::_PM.AbstractPowerModel, n::Int, ref_id, generator_properties, ΔTin,  ΔTdroop, f0, fmin, fmax, fdb, Δfss, zone_convs, hvdc_contribution, zone)
    alpha_g = _PM.var(pm, ref_id, :alpha_g)
    pconv_in = _PM.var(pm, n, :pconv_in)
    pg_droop = _PM.var(pm, n, :pg_droop)
    htot = _PM.var(pm, n, :htot, zone)
    dc_contr = _PM.var(pm, n, :dc_contr, zone)

    if isempty(zone_convs) || hvdc_contribution == false
        dc_contribution_in = 0
        dc_contribution_droop = 0
    else
        dc_contribution_in = calculate_hvdc_contribution(pconv_in, ΔTin, zone_convs)
        dc_contribution_droop = calculate_hvdc_contribution_droop(pconv_in, ΔTin, ΔTdroop, zone_convs)
    end

    gen_contribution_droop = calculate_gen_contribution_droop(pg_droop, ΔTin, ΔTdroop, generator_properties)

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmin - (f0-fdb))) <= 
     - f0 * ( - dc_contribution_in)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmax - (f0+fdb))) >= 
     -f0 * (- dc_contribution_in)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmin - (f0-fdb))) <= 
     - f0 * (- dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (fmax - (f0+fdb))) >= 
     - f0 * (- dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    ) 

    # JuMP.@constraint(pm.model, 
    # sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (-Δfss)) <=  
    #  - f0 * (- dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    # )

    # JuMP.@constraint(pm.model, 
    # sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]) *  (2 * (Δfss)) >= 
    #  - f0 * (- dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    # )

    JuMP.@constraint(pm.model, htot == sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]))
    JuMP.@constraint(pm.model, dc_contr == dc_contribution_in + dc_contribution_droop)
end


function calculate_hvdc_contribution_droop(pconv_in, ΔTin, ΔTdroop, zone_convs)
    sum([pconv_in[c] * (ΔTdroop - ΔTin) for (c, conv) in zone_convs])
end

function calculate_gen_contribution_droop(pg_droop, ΔTin, ΔTdroop, zone_gens)
    sum([pg_droop[g]/2 * (ΔTdroop - ΔTin)  for (g, gen) in zone_gens]) 
end

function constraint_gen_droop_power_balance(pm::_PM.AbstractPowerModel, i::Int, n::Int, reference_network_idx)
    pg = _PM.var(pm, n, :pg, i)
    pg_ref = _PM.var(pm, reference_network_idx, :pg, i)
    pg_droop = _PM.var(pm, n, :pg_droop, i)

    JuMP.@constraint(pm.model, pg == pg_ref + pg_droop)
end


function constraint_generator_droop(pm::_PM.AbstractPowerModel, i::Int, n::Int, ramp_rate, ΔTin, ΔTdroop)
    pg_droop = _PM.var(pm, n, :pg_droop, i)

    JuMP.@constraint(pm.model, pg_droop >= - ramp_rate * (ΔTdroop - ΔTin))
    JuMP.@constraint(pm.model, pg_droop <=   ramp_rate * (ΔTdroop - ΔTin))
end


function  constraint_generator_droop_absolute(pm::_PM.AbstractPowerModel, i::Int, n::Int)
    pg_droop = _PM.var(pm, n, :pg_droop, i)
    pg_droop_abs = _PM.var(pm, n, :pg_droop_abs, i)

    JuMP.@constraint(pm.model, pg_droop_abs >=  pg_droop)
    JuMP.@constraint(pm.model, pg_droop_abs >= -pg_droop)
end

function constraint_generator_contingency(pm::_PM.AbstractPowerModel, i::Int, n::Int, zone)
    pg = _PM.var(pm, n, :pg, i)
    δPg = _PM.var(pm, n, :gen_cont, zone)

    JuMP.@constraint(pm.model, δPg >= pg)
end

function constraint_select_generator_contingency(pm::_PM.AbstractPowerModel, n::Int, zone_gens)
    δg = _PM.var(pm, n, :delta_g)

    JuMP.@constraint(pm.model, sum(δg[g] for g in zone_gens) == 1) # could probably be relaxed as >= 1, to be tested.
end

function constraint_generator_contingency_indicator(pm::_PM.AbstractPowerModel, i::Int, n::Int, bigM, zone)
    pg = _PM.var(pm, n, :pg, i)
    δg = _PM.var(pm, n, :delta_g, i)
    δPg = _PM.var(pm, n, :gen_cont, zone)

    JuMP.@constraint(pm.model, (δg- 1) * bigM <= δPg - pg)
    JuMP.@constraint(pm.model, δPg - pg <= (1-δg) * bigM)
end

function constraint_converter_contingency(pm::_PM.AbstractPowerModel, i::Int, n::Int, zone)
    pc = _PM.var(pm, n, :pconv_ac, i)
    δPc_plus = _PM.var(pm, n, :conv_cont_plus, zone)
    δPc_minus = _PM.var(pm, n, :conv_cont_minus, zone)

    JuMP.@constraint(pm.model, δPc_plus >= pc)
    JuMP.@constraint(pm.model, δPc_minus <= pc)
end

function constraint_select_converter_contingency(pm::_PM.AbstractPowerModel, n::Int, zone_convs)
    δc_plus = _PM.var(pm, n, :delta_c_plus)
    δc_minus = _PM.var(pm, n, :delta_c_minus)

    JuMP.@constraint(pm.model, sum(δc_plus[c] for c in zone_convs) == 1) # could probably be relaxed as >= 1, to be tested.
    JuMP.@constraint(pm.model, sum(δc_minus[c] for c in zone_convs) == 1) # could probably be relaxed as >= 1, to be tested.
end

function constraint_converter_contingency_indicator(pm::_PM.AbstractPowerModel, i::Int, n::Int, bigM, zone)
    pc = _PM.var(pm, n, :pconv_ac, i)
    δc_plus = _PM.var(pm, n, :delta_c_plus, i)
    δc_minus = _PM.var(pm, n, :delta_c_minus, i)
    δPc_plus = _PM.var(pm, n, :conv_cont_plus, zone)
    δPc_minus = _PM.var(pm, n, :conv_cont_minus, zone)

    JuMP.@constraint(pm.model, (δc_plus- 1) * bigM <= δPc_plus - pc)
    JuMP.@constraint(pm.model, δPc_plus - pc <= (1-δc_plus) * bigM)

    JuMP.@constraint(pm.model, (δc_minus- 1) * bigM <= δPc_minus - pc)
    JuMP.@constraint(pm.model, δPc_minus - pc <= (1-δc_minus) * bigM)
end

function constraint_branch_contingency(pm::_PM.AbstractPowerModel, br_idx, n::Int)
    pl = _PM.var(pm, n, :p)[br_idx]
    δPl = _PM.var(pm, n, :branch_cont)

    JuMP.@constraint(pm.model, δPl >=  pl)
    JuMP.@constraint(pm.model, δPl >= -pl)
end

function constraint_select_branch_contingency(pm::_PM.AbstractPowerModel, n::Int)
    δl = _PM.var(pm, n, :delta_l)

    JuMP.@constraint(pm.model, sum(δl) == 1)
end

function constraint_branch_contingency_indicator(pm::_PM.AbstractPowerModel, i, n::Int, br_idx, bigM)
    pl = _PM.var(pm, n, :p)[br_idx]
    δl = _PM.var(pm, n, :delta_l, i)
    δPl = _PM.var(pm, n, :branch_cont)

    JuMP.@constraint(pm.model, (δl- 1) * bigM <= δPl - pl)
    JuMP.@constraint(pm.model, δPl - pl <= (1-δl) * bigM)
end

function constraint_frequency_droop_lean(pm::_PM.AbstractPowerModel, n::Int, ref_id, generator_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, zone_convs, hvdc_contribution, zone; zone_cont = false)
    ΔPg = _PM.var(pm, ref_id, :gen_cont, zone)
    αg = _PM.var(pm, ref_id, :alpha_g)
    δg = _PM.var(pm, ref_id, :delta_g)
    pconv_in = _PM.var(pm, n, :pconv_in)
    pg_droop = _PM.var(pm, n, :pg_droop)
    htot = _PM.var(pm, n, :htot, zone)
    dc_contr = _PM.var(pm, n, :dc_contr, zone)

    if isempty(zone_convs) || hvdc_contribution == false
        dc_contribution_in = 0
        dc_contribution_droop = 0
    else
        dc_contribution_in = calculate_hvdc_contribution(pconv_in, ΔTin, zone_convs)
        dc_contribution_droop = calculate_hvdc_contribution_droop(pconv_in, ΔTin, ΔTdroop, zone_convs)
    end

    gen_contribution_droop = calculate_gen_contribution_droop(pg_droop, ΔTin, ΔTdroop, generator_properties)

    if zone_cont == true
        ΔPg_ =  JuMP.@expression(pm.model, ΔPg)
    else
        ΔPg_ =  JuMP.@expression(pm.model, 0)
    end

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * (αg[g] - δg[g]) for (g, properties) in generator_properties]) *  (2 * (fmin - (f0-fdb))) <= 
     - f0 * (ΔPg_ *  ΔTin - dc_contribution_in)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * (αg[g] - δg[g]) for (g, properties) in generator_properties]) *  (2 * (fmax - (f0+fdb))) >= 
     - f0 * (ΔPg_ *  ΔTin - dc_contribution_in)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] *  (αg[g] - δg[g]) for (g, properties) in generator_properties]) *  (2 * (fmin - (f0-fdb))) <= 
     - f0 * (ΔPg_ * ΔTdroop - dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    )


    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * (αg[g] - δg[g]) for (g, properties) in generator_properties]) *  (2 * (fmax - (f0+fdb))) >= 
     - f0 * (ΔPg_ * ΔTdroop - dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    )

    JuMP.@constraint(pm.model, htot == sum([properties["inertia"] * properties["rating"] * (αg[g] - δg[g]) for (g, properties) in generator_properties]))
    JuMP.@constraint(pm.model, dc_contr == dc_contribution_in + dc_contribution_droop)
end

function constraint_frequency_converter_droop_lean(pm::_PM.AbstractPowerModel, n::Int, ref_id, generator_properties, ΔTin, ΔTdroop, f0, fmin, fmax, fdb, Δfss, zone_convs, hvdc_contribution, zone; zone_cont = false, direction = "plus")
    ΔPc_plus = _PM.var(pm, ref_id, :conv_cont_plus, zone)
    ΔPc_minus = _PM.var(pm, ref_id, :conv_cont_minus, zone)
    αg = _PM.var(pm, ref_id, :alpha_g)
    pconv_in = _PM.var(pm, n, :pconv_in)
    pg_droop = _PM.var(pm, n, :pg_droop)
    htot = _PM.var(pm, n, :htot, zone)
    dc_contr = _PM.var(pm, n, :dc_contr, zone)

    if isempty(zone_convs) || hvdc_contribution == false
        dc_contribution_in = 0
        dc_contribution_droop = 0
    else
        dc_contribution_in = calculate_hvdc_contribution(pconv_in, ΔTin, zone_convs)
        dc_contribution_droop = calculate_hvdc_contribution_droop(pconv_in, ΔTin, ΔTdroop, zone_convs)
        constraint_contingent_converter(pm, n, ref_id, zone_convs, direction)
    end

    gen_contribution_droop = calculate_gen_contribution_droop(pg_droop, ΔTin, ΔTdroop, generator_properties)

    if zone_cont == true && direction == "plus"
        ΔPc_ =  JuMP.@expression(pm.model, ΔPc_plus)
    elseif zone_cont == true && direction == "minus"
        ΔPc_ =  JuMP.@expression(pm.model, ΔPc_minus)
    else
        ΔPc_ =  JuMP.@expression(pm.model, 0)
    end

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * αg[g] for (g, properties) in generator_properties]) *  (2 * (fmin - (f0-fdb))) <= 
     - f0 * (ΔPc_ *  ΔTin - dc_contribution_in)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * αg[g] for (g, properties) in generator_properties]) *  (2 * (fmax - (f0+fdb))) >= 
     - f0 * (ΔPc_ *  ΔTin - dc_contribution_in)
    )

    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] *  αg[g] for (g, properties) in generator_properties]) *  (2 * (fmin - (f0-fdb))) <= 
     - f0 * (ΔPc_ * ΔTdroop - dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    )


    JuMP.@constraint(pm.model, 
    sum([properties["inertia"] * properties["rating"] * αg[g] for (g, properties) in generator_properties]) *  (2 * (fmax - (f0+fdb))) >= 
     - f0 * (ΔPc_ * ΔTdroop - dc_contribution_in - dc_contribution_droop - gen_contribution_droop)
    )

    JuMP.@constraint(pm.model, htot == sum([properties["inertia"] * properties["rating"] * αg[g] for (g, properties) in generator_properties]))
    JuMP.@constraint(pm.model, dc_contr == dc_contribution_in + dc_contribution_droop)
end

function constraint_contingent_converter(pm::_PM.AbstractPowerModel, n::Int, ref_id::Int, zone_convs, direction)
    if direction == "plus"
        δc = _PM.var(pm, ref_id, :delta_c_plus)
    else
        δc = _PM.var(pm, ref_id, :delta_c_minus)
    end
    pconv_in = _PM.var(pm, n, :pconv_in)
    for (c, conv) in zone_convs
        JuMP.@constraint(pm.model, -2 * _PM.ref(pm, ref_id, :convdc, c)["Pacrated"] <= pconv_in[c] * δc[c]) 
        JuMP.@constraint(pm.model,  2 * _PM.ref(pm, ref_id, :convdc, c)["Pacrated"] >= pconv_in[c] * δc[c])
    end
end


# function constraint_frequency_inertia(pm::_PM.AbstractPowerModel, n::Int, ref_id, generator_properties, gcont, f0, fmin, fmax, Tfcr, zone_convs, hvdc_contribution, zone)
#     ΔPg = _PM.var(pm, ref_id, :pg)[gcont]
#     alpha_g = _PM.var(pm, ref_id, :alpha_g)
#     pconv_in = _PM.var(pm, n, :pconv_in)
#     htot = _PM.var(pm, n, :htot, zone)
#     dc_contr = _PM.var(pm, n, :dc_contr, zone)

#     T = 0:0.1:Tfcr

#     if isempty(zone_convs) || hvdc_contribution == false
#         dc_contribution = 0
#     else
#         dc_contribution = calculate_hvdc_contribution(pconv_in, Tfcr, zone_convs)
#     end

#     JuMP.@constraint(pm.model, htot == sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]))
#     JuMP.@constraint(pm.model, htot *  (2 * (f0 - fmin)) >= f0 * (ΔPg *  Tfcr - dc_contribution))
#     JuMP.@constraint(pm.model, htot *  (2 * (fmax - f0)) >= -f0 * (ΔPg *  Tfcr - dc_contribution))
#     JuMP.@constraint(pm.model, dc_contr == dc_contribution)
# end

# function constraint_frequency_inertia(pm::_PM.AbstractPowerModel, n::Int, ref_id, generator_properties, f0, fmin, fmax, Tfcr, zone_convs, hvdc_contribution, zone)
#     alpha_g = _PM.var(pm, ref_id, :alpha_g)
#     pconv_in = _PM.var(pm, n, :pconv_in)
#     htot = _PM.var(pm, n, :htot, zone)
#     dc_contr = _PM.var(pm, n, :dc_contr, zone)

#     JuMP.@constraint(pm.model, htot == sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]))

#     T = 0:0.1:Tfcr

#     if isempty(zone_convs) || hvdc_contribution == false
#         dc_contribution = 0
#     else
#         dc_contribution = calculate_hvdc_contribution(pconv_in, Tfcr, zone_convs)
#     end

#     JuMP.@constraint(pm.model, htot == sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]))
#     JuMP.@constraint(pm.model, htot *  (2 * (f0 - fmin)) >=  -f0 * dc_contribution)
#     JuMP.@constraint(pm.model, htot *  (2 * (fmax - f0)) >=   f0 * dc_contribution)
#     JuMP.@constraint(pm.model, dc_contr == dc_contribution)
# end

# function constraint_converter_power_balance_ramp(pm::_PM.AbstractPowerModel, i::Int, n::Int, reference_network_idx)
#     pconv = _PM.var(pm, n, :pconv_ac, i)
#     pconv_ref = _PM.var(pm, reference_network_idx, :pconv_ac, i)
#     rdc = _PM.var(pm, n, :rdc, i)

#     JuMP.@constraint(pm.model, pconv == pconv_ref + rdc)
# end

# function constraint_frequency_droop(pm::_PM.AbstractPowerModel, n::Int, ref_id, generator_properties, gcont, f0, fmin, Tfcr, zone, zone_convs)
#     ΔPg = _PM.var(pm, ref_id, :pg)[gcont]
#     rg = _PM.var(pm, ref_id, :rg)
#     rdc = _PM.var(pm, ref_id, :rdc)
#     htot = _PM.var(pm, n, :htot, zone)
#     alpha_g = _PM.var(pm, ref_id, :alpha_g)

#     JuMP.@constraint(pm.model, htot == sum([properties["inertia"] * properties["rating"] * alpha_g[g] for (g, properties) in generator_properties]))

#     T = Tfcr:0.1:Tfcrd

#     for t in T
#         JuMP.@constraint(pm.model, ΔPg * t - rdc * t^2 - sum([rg[g] * t^2 for (g, property) in generator_properties])  >= fmin / f0 * 2 * htot)
#     end 

#     JuMP.@constraint(pm.model, [pconv[c] + rdc * T[end]^2 <= conv["pmax"] for (c, conv) in zone_convs])
# end