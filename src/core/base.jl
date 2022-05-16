# ADD REF MODEL
function ref_add_pst!(pm::_PM.AbstractPowerModel)
    for (nw, nw_ref) in pm.ref[:nw]
        if !haskey(nw_ref, :pst)
            error(_LOGGER, "required pst data not found")
        end

        nw_ref[:pst] = Dict(x for x in nw_ref[:pst] if (x.second["status"] == 1 && x.second["fbus"] in keys(nw_ref[:bus]) && x.second["tbus"] in keys(nw_ref[:bus])))

        nw_ref[:arcs_from_pst] = [(i,pst["fbus"],pst["tbus"]) for (i,pst) in nw_ref[:pst]]
        nw_ref[:arcs_to_pst]   = [(i,pst["tbus"],pst["fbus"]) for (i,pst) in nw_ref[:pst]]
        nw_ref[:arcs_pst] = [nw_ref[:arcs_from_pst]; nw_ref[:arcs_to_pst]]

        bus_arcs_pst = Dict((i, []) for (i,bus) in nw_ref[:bus])
        for (l,i,j) in nw_ref[:arcs_pst]
            push!(bus_arcs_pst[i], (l,i,j))
        end
        nw_ref[:bus_arcs_pst] = bus_arcs_pst
    end
end