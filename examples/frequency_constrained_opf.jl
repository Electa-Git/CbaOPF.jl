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
file_ac = "./test/data/case5_2grids_inertia.m"
file_dc = "./test/data/case5_3grids_inertia_hvdc.m"
plot_path = "/Users/hergun/Library/CloudStorage/OneDrive-KULeuven/Projects/HVDC_H2/plots"

# Define solvers
ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6)
gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer)
# Parse file using PowerModels
data = PowerModels.parse_file(file_dc)
# Process demand reduction and curtailment data
CbaOPF.add_flexible_demand_data!(data)
# Process inertia data
CbaOPF.prepare_data!(data; t_hvdc = 0.10, ffr_cost = 50.0)
# Add empth dictionary for PSTs
data["pst"] = Dict{String, Any}()
if !haskey(data, "convdc")
    data["busdc"] = Dict{String, Any}()
    data["convdc"] = Dict{String, Any}()
    data["branchdc"] = Dict{String, Any}()
end

fmin = 49.0:0.05:49.6
objective_no_dc = zeros(1, length(fmin))
objective_dc = zeros(1, length(fmin))

result_no_dc = Dict{String, Any}(["$idx"=>Dict{String, Any}() for idx = 1:length(fmin)])
result_dc = Dict{String, Any}(["$idx"=>Dict{String, Any}() for idx = 1:length(fmin)])

for idx = 1:length(fmin)
    # Add frequency stability data
    data["frequency_parameters"] = Dict{String, Any}()
    data["frequency_parameters"]["fmin"] = fmin[idx]
    data["frequency_parameters"]["fmax"] = 50.0
    data["frequency_parameters"]["f0"] = 50.0
    data["frequency_parameters"]["t_fcr"] = 0.2
    # Add generator contingencies
    mn_data = CbaOPF.create_generator_contingencies(data)
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


fmin_idx = 1
mn_data = CbaOPF.create_generator_contingencies(data)
print("########### NO HVDC contribution for fmin = ", fmin[fmin_idx], " Hz #####################", "\n")
CbaOPF.print_frequency_information(result_no_dc, mn_data; fmin_idx = fmin_idx)
print("########### With HVDC contribution for fmin = ", fmin[fmin_idx], " Hz #####################", "\n")
CbaOPF.print_frequency_information(result_dc, mn_data; fmin_idx = fmin_idx)

for idx in 2:16
    for z_idx in 1:3
        c = idx - 1
        print("Contingency ", c, " -> hvdc contribution into zone ", z_idx, " ", result_dc["1"]["solution"]["nw"]["$idx"]["zones"]["$z_idx"]["dc_contr"], "\n")
    end
end