import PowerModelsACDC
const _PMACDC = PowerModelsACDC
import PowerModels
const _PM = PowerModels
import CbaOPF
import Ipopt
import Memento
import JuMP
import Gurobi


# Load test file
file_ac = "./test/data/case5_2grids_inertia.m"
file_dc = "./test/data/case5_2grids_inertia_hvdc.m"

# Define solvers
ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6)
gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer)
# Parse file using PowerModels
data = PowerModels.parse_file(file_dc)
# Process demand reduction and curtailment data
CbaOPF.add_flexible_demand_data!(data)
# Process inertia data
CbaOPF.prepare_data!(data; t_hvdc = 0.1)
# Add empth dictionary for PSTs
data["pst"] = Dict{String, Any}()
if !haskey(data, "convdc")
    data["busdc"] = Dict{String, Any}()
    data["convdc"] = Dict{String, Any}()
    data["branchdc"] = Dict{String, Any}()
end
# Add frequency stability data
data["frequency_parameters"] = Dict{String, Any}()
data["frequency_parameters"]["fmin"] = 49.3
data["frequency_parameters"]["fmax"] = 50.0
data["frequency_parameters"]["f0"] = 50.0
data["frequency_parameters"]["t_fcr"] = 0.2
# Add generator contingencies
mn_data = CbaOPF.create_generator_contingencies(data)
# Process DC grid data
_PMACDC.process_additional_data!(mn_data)
# Provide addtional settings as part of the PowerModels settings dictionary
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => true)

result = CbaOPF.solve_fsopf(mn_data, _PM.DCPPowerModel, gurobi, setting = s, multinetwork = true)

contingencies = sort(parse.(Int, collect(keys(result["solution"]["nw"]))))[2:end]
for c in contingencies
    c_res = result["solution"]["nw"]["$c"]
    gen_id = mn_data["nw"]["$c"]["contingency"]["gen_id"]
    ΔPg = result["solution"]["nw"]["1"]["gen"]["$gen_id"]["pg"] * result["solution"]["nw"]["1"]["gen"]["$gen_id"]["alpha_g"] 
    f0 = mn_data["nw"]["1"]["frequency_parameters"]["f0"]
    ΔT = mn_data["nw"]["1"]["frequency_parameters"]["t_fcr"]
    for (z, zone) in c_res["zones"]
        if  mn_data["nw"]["1"]["gen"]["$gen_id"]["zone"] == parse(Int, z)
            f =  f0 * (1 - (ΔPg * ΔT - zone["dc_contr"]) / (2 * zone["htot"]))
        else 
            f =  f0 * (1 + (zone["dc_contr"]) / (2 * zone["htot"]))
        end

        print("Contingency of generator ", gen_id, " of ", ΔPg*100 ," MW, -> f in zone ", z, " = ", f, " Hz", "\n")
    end
end