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
data = PowerModels.parse_file(file_ac)
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
data["frequency_parameters"]["fmin"] = 49
data["frequency_parameters"]["f0"] = 50
data["frequency_parameters"]["t_fcr"] = 0.2
# Add generator contingencies
mn_data = CbaOPF.create_generator_contingencies(data)
# Process DC grid data
_PMACDC.process_additional_data!(mn_data)
# Provide addtional settings as part of the PowerModels settings dictionary
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => true)

result = CbaOPF.solve_fsopf(mn_data, _PM.DCPPowerModel, gurobi, setting = s, multinetwork = true)

h_tot = [0.0 0.0]
for (g, gen) in result["solution"]["nw"]["1"]["gen"]
    zone = mn_data["nw"]["1"]["gen"][g]["zone"]
    h = mn_data["nw"]["1"]["gen"][g]["pmax"] *  mn_data["nw"]["1"]["gen"][g]["inertia_constants"] * gen["alpha_g"]
    h_tot[zone] = h + h_tot[zone] 
end

for idx in 1:2
    for (g, gen) in result["solution"]["nw"]["1"]["gen"]
        if data["gen"][g]["zone"] == idx
            h = h_tot[idx] - data["gen"][g]["inertia_constants"]
            Δf = gen["alpha_g"] * gen["pg"] * data["frequency_parameters"]["f0"] * data["frequency_parameters"]["t_fcr"] / (2 * h)
            print(g, " ", gen["pg"], " ",Δf, "\n")
        end
    end
end