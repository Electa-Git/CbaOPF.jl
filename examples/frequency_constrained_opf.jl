import PowerModelsACDC
const _PMACDC = PowerModelsACDC
import PowerModels
const _PM = PowerModels
import CbaOPF
import Ipopt
import Memento
import JuMP
import Gurobi
import Plots
import  LaTeXStrings


# Load test file
file = "./test/data/case5_2grids_inertia_acdc.m"
plot_path = "/Users/hergun/Library/CloudStorage/OneDrive-KULeuven/Projects/HVDC_H2/plots"

# Define solvers
ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6)
gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer)
# Parse file using PowerModels
data = PowerModels.parse_file(file)
# Process demand reduction and curtailment data
CbaOPF.add_flexible_demand_data!(data)
# Process inertia data
CbaOPF.prepare_data!(data; t_hvdc = 0.10, ffr_cost = 50.0)

fmin = 49.0:0.05:49.8
objective_no_dc = zeros(1, length(fmin))
objective_dc = zeros(1, length(fmin))

result_no_dc = Dict{String, Any}(["$idx"=>Dict{String, Any}() for idx = 1:length(fmin)])
result_dc = Dict{String, Any}(["$idx"=>Dict{String, Any}() for idx = 1:length(fmin)])
for idx = 1:length(fmin)
    # Add frequency stability data
    data["frequency_parameters"] = Dict{String, Any}()
    data["frequency_parameters"]["fmin"] = fmin[idx]
    data["frequency_parameters"]["f0"] = 50.0
    data["frequency_parameters"]["fmax"] =  data["frequency_parameters"]["f0"] + ((data["frequency_parameters"]["f0"] - fmin[idx]))
    data["frequency_parameters"]["t_fcr"] = 0.2
    # Add generator contingencies
    mn_data = CbaOPF.create_contingencies(data)
    # Process DC grid data
    _PMACDC.process_additional_data!(mn_data)
    # Provide addtional settings as part of the PowerModels settings dictionary
    s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => false)
    result_no_dc["$idx"] = CbaOPF.solve_fsopf(mn_data, _PM.DCPPowerModel, gurobi, setting = s, multinetwork = true)
    objective_no_dc[1, idx] = result_no_dc["$idx"]["objective"] 

    s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => true)
    result_dc["$idx"] = CbaOPF.solve_fsopf(mn_data, _PM.DCPPowerModel, gurobi, setting = s, multinetwork = true)
    objective_dc[1, idx] = result_dc["$idx"]["objective"] 
end

p1 = Plots.plot(fmin, objective_no_dc', xlabel = "\$f_{min} in~Hz\$", ylabel = "\$Cost~in~€\$", label = "without HVDC contribution")
Plots.plot!(p1, fmin, objective_dc',  xlabel = "\$f_{min} in~Hz\$", ylabel = "\$Cost~in~€\$", label = "with HVDC contribution")
plot_filename = joinpath(plot_path, "objective_comparison.pdf")
Plots.savefig(p1, plot_filename)


fmin_idx = length(fmin)
mn_data = CbaOPF.create_contingencies(data)
print("########### NO HVDC contribution for fmin = ", fmin[fmin_idx], " Hz #####################", "\n")
CbaOPF.print_frequency_information(result_no_dc, mn_data; fmin_idx = fmin_idx)
print("########### With HVDC contribution for fmin = ", fmin[fmin_idx], " Hz #####################", "\n")
CbaOPF.print_frequency_information(result_dc, mn_data; fmin_idx = fmin_idx)


br_idx = data["tie_lines"]["1"]["br_idx"]
tie_line_flows = [result_no_dc["$idx"]["solution"]["nw"]["1"]["branch"]["$br_idx"]["pf"] for idx in 1:length(fmin)] * data["baseMVA"]
tie_line_flows_dc = [result_dc["$idx"]["solution"]["nw"]["1"]["branch"]["$br_idx"]["pf"] for idx in 1:length(fmin)] * data["baseMVA"]

p2 = Plots.plot(fmin, tie_line_flows, xlabel = "\$f_{min} in~Hz\$", ylabel = "\$Flow~in~MVA\$", label = "tie line flow zone 1 -> zone 2, without HVDC contribution")
Plots.plot!(p2, fmin, tie_line_flows_dc,  xlabel = "\$f_{min} in~Hz\$", ylabel = "\$Cost~in~€\$", label = "tie line flow zone 1 -> zone 2, with HVDC contribution")
plot_filename = joinpath(plot_path, "tieline_flows.pdf")
Plots.savefig(p2, plot_filename)


# p1 = Plots.plot(fmin, objective_no_dc', xlabel = "\$f_{min} in~Hz\$", ylabel = "\$Cost~in~€\$", label = "total cost")
# plot_filename = joinpath(plot_path, "objective_comparison.pdf")
# Plots.savefig(p1, plot_filename)