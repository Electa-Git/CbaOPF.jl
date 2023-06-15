# DATA SCALINNG PST
function process_pst_data!(data)
    if !haskey(data, "multinetwork") || data["multinetwork"] == false
        to_pu_single_network_pst!(data)
        fix_data_single_network_pst!(data)
    else
        to_pu_multi_network_pst!(data)
        fix_data_multi_network_pst!(data)
    end
end

function to_pu_single_network_pst!(data)
    MVAbase = data["baseMVA"]
    for (i, pst) in data["pst"]
        scale_pst_data!(pst, MVAbase)
    end
end

function fix_data_single_network_pst!(data)
    for (i, pst) in data["pst"]
        pst["g_fr"] = 0
        pst["b_fr"] = 0
        pst["g_to"] = 0
        pst["b_to"] = 0
        pst["tap"] = 1.0
    end
end
function to_pu_multi_network_pst!(data)
    MVAbase = data["baseMVA"]
    for (n, network) in data["nw"]
        MVAbase = network["baseMVA"]
        for (i, pst) in network[n]["pst"]
            scale_pst_data!(pst, MVAbase)
        end
    end
end

function fix_data_multi_network_pst!(data)
    for (n, network) in data["nw"]
        for (i, pst) in network[n]data["pst"]
            pst["g_fr"] = 0
            pst["b_fr"] = 0
            pst["g_to"] = 0
            pst["b_to"] = 0
            pst["tap"] = 1.0
        end
    end
end

function scale_pst_data!(pst, MVAbase)
    rescale_power = x -> x/MVAbase
    _PM._apply_func!(pst, "rate_a", rescale_power)
    _PM._apply_func!(pst, "rate_b", rescale_power)
    _PM._apply_func!(pst, "rate_c", rescale_power)
    _PM._apply_func!(pst, "angle", deg2rad)
    _PM._apply_func!(pst, "angmin", deg2rad)
    _PM._apply_func!(pst, "angmax", deg2rad)
end


function add_flexible_demand_data!(data)
    for (le, load_extra) in data["load_extra"]

        # ID of load point
        idx = load_extra["load_id"]

        # Superior bound on voluntary load reduction (not consumed power) as a fraction of the total reference demand (0 ≤ pred_rel_max ≤ 1)
        data["load"]["$idx"]["pred_rel_max"] = load_extra["pred_rel_max"]

        # Compensation for consuming less (i.e. voluntary demand reduction) (€/MWh)
        data["load"]["$idx"]["cost_red"] = load_extra["cost_red"]

        # Compensation for load curtailment (i.e. involuntary demand reduction) (€/MWh)
        data["load"]["$idx"]["cost_curt"] = load_extra["cost_curt"]

        # Whether load is flexible (boolean)
        data["load"]["$idx"]["flex"] = load_extra["flex"]

        # Power factor angle θ, giving the reactive power as Q = P ⨉ tan(θ)
        if haskey(load_extra, "pf_angle")
            data["load"]["$idx"]["pf_angle"] = load_extra["pf_angle"]
        end

        # Rescale cost and power input values to the p.u. values used internally in the model
        rescale_cost = x -> x*data["baseMVA"]
        rescale_power = x -> x/data["baseMVA"]
        _PM._apply_func!(data["load"]["$idx"], "cost_red", rescale_cost)
        _PM._apply_func!(data["load"]["$idx"], "cost_curt", rescale_cost)
    end
    delete!(data, "load_extra")
    return data
end

function create_contingencies!(mn_data, number_of_hours, number_of_contingencies)

    gen_keys = sort(parse.(Int, collect(keys(mn_data["nw"]["1"]["gen"]))))
    conv_keys = sort(parse.(Int, collect(keys(mn_data["nw"]["1"]["convdc"]))))
    tie_line_keys = sort(parse.(Int, collect(keys(mn_data["nw"]["1"]["tie_lines"]))))
    dc_branch_keys = sort(parse.(Int, collect(keys(mn_data["nw"]["1"]["branchdc"]))))

    for idx in 1:number_of_hours * number_of_contingencies
        if any(idx .== mn_data["hour_ids"])
            mn_data["nw"]["$idx"]["contingency"] = Dict{String, Any}("gen_id" => nothing, "branch_id" => nothing, "conv_id" => nothing, "dcbranch_id" => nothing)
        elseif mod(idx - 1, number_of_contingencies) <= length(gen_keys)
            c_id = mod(idx - 1, number_of_contingencies)
            mn_data["nw"]["$idx"]["contingency"] = Dict{String, Any}("gen_id" => gen_keys[c_id], "branch_id" => nothing, "conv_id" => nothing, "dcbranch_id" => nothing)
        elseif mod(idx - 1, number_of_contingencies) <= length(gen_keys) + length(tie_line_keys)
            c_id = mod(idx - 1, number_of_contingencies)
            b_idx = c_id - length(gen_keys)
            mn_data["nw"]["$idx"]["contingency"] = Dict{String, Any}("gen_id" => nothing, "branch_id" => tie_line_keys[b_idx], "conv_id" => nothing, "dcbranch_id" => nothing)
        elseif mod(idx - 1, number_of_contingencies) <= length(gen_keys) + length(tie_line_keys) + length(conv_keys)
            c_id = mod(idx - 1, number_of_contingencies)
            c_idx = c_id - length(gen_keys) - length(tie_line_keys)
            mn_data["nw"]["$idx"]["contingency"] = Dict{String, Any}("gen_id" => nothing, "branch_id" => nothing, "conv_id" => conv_keys[c_idx], "dcbranch_id" => nothing)
        else
            c_id = mod(idx-1, number_of_contingencies)
            b_idx = c_id - length(gen_keys) - length(tie_line_keys) - length(conv_keys)
            mn_data["nw"]["$idx"]["contingency"] = Dict{String, Any}("gen_id" => nothing, "branch_id" => nothing, "conv_id" => nothing, "dcbranch_id" => dc_branch_keys[b_idx])
        end
    end
    return mn_data
end




function create_multinetwork_model!(data, number_of_hours, g_series, l_series)

    generator_contingencies = length(data["gen"])
    tie_line_contingencies = length(data["tie_lines"]) 
    converter_contingencies = length(data["convdc"]) 
    dc_branch_contingencies = length(data["branchdc"]) 
    number_of_contingencies = generator_contingencies + tie_line_contingencies + converter_contingencies +  dc_branch_contingencies + 1 # to also add the N case



    # This for loop determines which "network" belongs to an hour, and which to a contingency, for book-keeping of the network ids
    # Format: [h1, c1 ... cn, h2, c1 ... cn, .... , hn, c1 ... cn]
    hour_ids = [];
    cont_ids = [];
    for i in 1:number_of_hours * number_of_contingencies
        if mod(i, number_of_contingencies) == 1
            push!(hour_ids, i)
        else
            push!(cont_ids, i)
        end
    end

    ########### Using _IM.replicate networks

    mn_data = _IM.replicate(data, number_of_hours * number_of_contingencies, Set{String}(["source_type", "name", "source_version", "per_unit"]))

    # Add hour_ids and contingency_ids to the data dictionary 
    mn_data["hour_ids"] = hour_ids
    mn_data["cont_ids"] = cont_ids
    mn_data["number_of_hours"] = number_of_hours
    mn_data["number_of_contingencies"] = number_of_contingencies

    create_contingencies!(mn_data, number_of_hours, number_of_contingencies)

    # This loop writes the generation and demand time series data
    iter = 0
    for nw = 1:number_of_hours * number_of_contingencies
        if mod(nw, number_of_contingencies) == 1
            iter += 1
            h_idx = Int(nw - number_of_contingencies + 1)
        end
        h_idx = iter
        for (g, gen) in mn_data["nw"]["$nw"]["gen"]
            if gen["res"] == true
                gen["pmax"] = g_series[h_idx] * gen["pmax"]
            end
        end
        for (l, load) in mn_data["nw"]["$nw"]["load"]
            load["pd"] = l_series[h_idx] * load["pd"]
        end
    end

    _PMACDC.process_additional_data!(mn_data)




    return mn_data
end