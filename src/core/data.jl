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
        end
    end
end

function scale_pst_data!(pst, MVAbase)
    rescale_power = x -> x/MVAbase
    _PM._apply_func!(pst, "rateA", rescale_power)
    _PM._apply_func!(pst, "rateB", rescale_power)
    _PM._apply_func!(pst, "rateC", rescale_power)
    _PM._apply_func!(pst, "angle", deg2rad)
    _PM._apply_func!(pst, "angmin", deg2rad)
    _PM._apply_func!(pst, "angmax", deg2rad)
end
