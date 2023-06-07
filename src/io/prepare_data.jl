function prepare_data!(data; borders = nothing, t_hvdc = nothing, ffr_cost = nothing)
    prepare_generator_data!(data)

    if !isnothing(borders)
        find_and_assign_xb_lines!(data, borders)
    end

    if haskey(data, "convdc")
        for (c, conv) in data["convdc"]
            conv_bus = conv["busac_i"]
            conv["zone"] = data["bus"]["$conv_bus"]["zone"]
            conv["t_hvdc"] = t_hvdc
            conv["ffr_cost"] = ffr_cost
        end
    end

    return data
end


function prepare_generator_data!(data)
    for (g, gen) in data["gen"]
        bus_id = gen["gen_bus"]
        gen["zone"] = data["bus"]["$bus_id"]["zone"]
        gen["start_up_cost"] = gen["startup"]
        gen["inertia_constants"] = data["inertia_constants"][g]["inertia_constant"]
    end
end


function find_and_assign_xb_lines!(data, borders)

    for (b, border) in borders
        border["xb_lines"] = Dict{String, Any}()
        border["xb_convs"] = Dict{String, Any}()
        idx_br = 1
        idx_c = 1
        for (b, bus) in data["bus"]
            if any(bus["index"] .== border["xb_nodes"])
                bus_id = bus["index"]
                for (br, branch) in data["branch"]
                    if branch["f_bus"] == bus_id 
                        border["xb_lines"]["$idx_br"] = Dict{String, Any}()
                        branch["direction"] = "to"
                        border["xb_lines"]["$idx_br"] = branch
                        idx_br = idx_br + 1
                    elseif branch["t_bus"] == bus_id
                        border["xb_lines"]["$idx_br"] = Dict{String, Any}()
                        branch["direction"] = "from"
                        border["xb_lines"]["$idx_br"] = branch
                        idx_br = idx_br + 1
                    end
                end
                for (c, conv) in data["convdc"]
                    if conv["busac_i"] == bus_id 
                        border["xb_convs"]["$idx_c"] = Dict{String, Any}()
                        conv["status"] = 1
                        border["xb_convs"]["$idx_c"] = conv
                        idx_c = idx_c + 1
                    end
                end
            end
        end
    end

    data["borders"] = borders

    return data
end


function prepare_redispatch_data(opf_result, grid_data; contingency = nothing, rd_cost_factor = 4, inertia_limit = nothing, zonal_input = nothing, zonal_result = nothing, zone = nothing, border_slack = nothing)
    grid_data_rd = deepcopy(grid_data)
    result = opf_result["solution"]

    for (g, gen) in grid_data_rd["gen"]
        if haskey(result["gen"], g)
            gen["pg"] = result["gen"][g]["pg"]
            if gen["pg"] == 0.0
                gen["dispatch_status"] = 0
            else
                gen["dispatch_status"] = 1
            end 
        else
            gen["dispatch_status"] = 0
        end
        
        gen["rdcost_up"] = gen["cost"][1] * rd_cost_factor
        gen["rdcost_down"] = gen["cost"][1] * rd_cost_factor
        if !haskey(gen, "start_up_cost")
            gen["start_up_cost"] = 500
        end
    end

    for (l, load) in grid_data_rd["load"]
        if haskey(result["load"], l)
            load["pd"] = result["load"][l]["pflex"]
        end
    end

    for (c, conv) in grid_data_rd["convdc"]
        conv["P_g"] = -result["convdc"][c]["ptf_to"]
    end

    if !isnothing(contingency)
        for (b, border) in grid_data_rd["borders"]
            for (br, branch) in  border["xb_lines"]
                if branch["index"] == contingency
                    print(b, " ", br)
                    delete!(grid_data_rd["borders"][b]["xb_lines"], br)
                end
            end
        end
        grid_data_rd["branch"]["$contingency"]["br_status"] = 0
    end

    if !isnothing(inertia_limit)
        grid_data_rd["inertia_limit"] = inertia_limit
    end

    if  !isnothing(zone)
        determine_total_xb_flow!(zonal_input, grid_data_rd, grid_data_rd, zonal_result, hour, zone)
    end

    for (bo, border) in grid_data_rd["borders"]
        if !isnothing(border_slack)
            border["slack"] = border_slack
        else
            border["slack"] = 0
        end
    end
    return grid_data_rd
end


function print_frequency_information(result, mn_data; fmin_idx = 1)
    print("Objective = ", result["$fmin_idx"]["objective"], "\n")
    contingencies = sort(parse.(Int, collect(keys(result["$fmin_idx"]["solution"]["nw"]))))[2:end]
    for c in contingencies
        c_res = result["$fmin_idx"]["solution"]["nw"]["$c"]
        gen_id = mn_data["nw"]["$c"]["contingency"]["gen_id"]
        ΔPg = result["$fmin_idx"]["solution"]["nw"]["1"]["gen"]["$gen_id"]["pg"] * result["$fmin_idx"]["solution"]["nw"]["1"]["gen"]["$gen_id"]["alpha_g"] 
        f0 = mn_data["nw"]["1"]["frequency_parameters"]["f0"]
        ΔT = mn_data["nw"]["1"]["frequency_parameters"]["t_fcr"]
        for (z, zone) in c_res["zones"]
            g_zone = mn_data["nw"]["1"]["gen"]["$gen_id"]["zone"]
            if  g_zone == parse(Int, z)
                f =  f0 * (1 - (ΔPg * ΔT - zone["dc_contr"]) / (2 * zone["htot"]))
            else 
                f =  f0 * (1 + (zone["dc_contr"]) / (2 * zone["htot"]))
            end
            print("Contingency of generator ", gen_id, " of ", round(ΔPg) * 100 ," MW in zone ", g_zone ," leads to f in zone ", z, " = ", f, " Hz", "\n")
        end
    end
end