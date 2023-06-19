function prepare_data!(data; borders = nothing, t_hvdc = nothing, ffr_cost = nothing, uc = false)
    prepare_generator_data!(data; uc = uc)

    if !isnothing(borders)
        find_and_assign_xb_lines!(data, borders)
    end

    if haskey(data, "convdc")
        for (c, conv) in data["convdc"]
            conv_bus = conv["busac_i"]
            conv["zone"] = data["bus"]["$conv_bus"]["zone"]
            conv["area"] = data["bus"]["$conv_bus"]["area"]
            conv["t_hvdc"] = t_hvdc
            conv["ffr_cost"] = ffr_cost
        end
    end

    # Add empth dictionary for PSTs
    if !haskey(data, "pst")
        data["pst"] = Dict{String, Any}()
    end

    # Add empty dictionaries for HVDC if only AC grid...
    if !haskey(data, "convdc")
        data["busdc"] = Dict{String, Any}()
        data["convdc"] = Dict{String, Any}()
        data["branchdc"] = Dict{String, Any}()
    end

    # Add empty dictionary if no AC tie lines are defined
    if !haskey(data, "tie_lines")
        data["tie_lines"] = Dict{String, Any}()
    end

    # Add empty dictionary if no areas are defined
    if !haskey(data, "areas")
        data["areas"] = Dict{String, Any}()
    end

    return data
end


function prepare_generator_data!(data; uc = false)
    for (g, gen) in data["gen"]
        bus_id = gen["gen_bus"]
        gen["zone"] = data["bus"]["$bus_id"]["zone"]
        gen["area"] = data["bus"]["$bus_id"]["area"]
        gen["inertia_constants"] = data["inertia_constants"][g]["inertia_constant"]
        if uc == true
            gen["start_up_cost"] = gen["startup"]
            gen["ramp_rate"] = data["inertia_constants"][g]["ramp_rate"]
            if data["inertia_constants"][g]["inertia_constant"] <= 1
                gen["mut"] = 1
                gen["mdt"] = 1
                gen["res"] = true
            else
                gen["mut"] = 3
                gen["mdt"] = 4
                gen["res"] = false
            end
        end
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
        if !isnothing(mn_data["nw"]["$c"]["contingency"]["gen_id"])
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
                print("Contingency of generator ", gen_id, " of ", round(ΔPg * 100) ," MW in zone ", g_zone ," leads to f in zone ", z, " = ", f, " Hz", "\n")
            end
        elseif !isnothing(mn_data["nw"]["$c"]["contingency"]["branch_id"])
            br_id = mn_data["nw"]["$c"]["contingency"]["branch_id"]
            br_idx = mn_data["nw"]["$c"]["tie_lines"]["$br_id"]["br_idx"]
            area_fr = mn_data["nw"]["$c"]["tie_lines"]["$br_id"]["area_fr"]
            area_to = mn_data["nw"]["$c"]["tie_lines"]["$br_id"]["area_to"]
            f0 = mn_data["nw"]["1"]["frequency_parameters"]["f0"]
            ΔT = mn_data["nw"]["1"]["frequency_parameters"]["t_fcr"]
            areas = [area_fr area_to]
            for a in areas
                if a == area_fr
                    ΔP = result["$fmin_idx"]["solution"]["nw"]["1"]["branch"]["$br_idx"]["pf"]
                else
                    ΔP = result["$fmin_idx"]["solution"]["nw"]["1"]["branch"]["$br_idx"]["pt"]
                end
                area = c_res["areas"]["$a"]    
                f =  f0 * (1 - (-ΔP * ΔT - area["dc_contr"]) / (2 * area["htot"]))
                print("Contingency of branch ", br_idx, " carrying ", round(ΔP * 100) ," MW leads to f in zone ", a, " = ", f, " Hz", "\n")
            end
        end
    end
end