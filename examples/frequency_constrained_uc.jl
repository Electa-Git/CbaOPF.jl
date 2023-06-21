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
import LaTeXStrings


# Load test file
file = "./test/data/case5_2grids_inertia_hvdc.m"
plot_path = "/Users/hergun/Library/CloudStorage/OneDrive-KULeuven/Projects/HVDC_H2/plots"

# Define solvers
ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6)
gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer)
# Parse file using PowerModels
data = PowerModels.parse_file(file)
# Process demand reduction and curtailment data
CbaOPF.add_flexible_demand_data!(data)
# Process inertia data
CbaOPF.prepare_data!(data; uc = true, t_hvdc = 0.10, ffr_cost = 50.0)

fmin = 49.0:0.05:49.8
objective_no_dc = zeros(1, length(fmin))
objective_dc = zeros(1, length(fmin))
objective_no_dc_relax = zeros(1, length(fmin))
objective_dc_relax = zeros(1, length(fmin))

number_of_hours = 24

g_series = [0.4  0.5  0.66  0.7   0.7   0.9   0.95 1.02  1.15  1.3   1.35 1.3   1.21  1.08  1.0  0.96  0.93 1.0  1.1   1.2  1.08  1.05  0.99 0.89]
# g_series = [1.02  1.15  1.3   1.35 1.3   1.21  1.08  1.0 0.4  0.5  0.66  0.7   0.7   0.9   0.95   0.96  0.93 1.0  1.1   1.2  1.08  1.05  0.99 0.89]
l_series = [0.6  0.7  0.75  0.78  0.85  0.88  0.9  1.0   1.12  1.25  1.2  1.08  0.99  0.92  0.8  0.73  0.8  0.9  1.03  1.2  1.11  0.99  0.8  0.69]
# l_series = ones(1,24) .* 0.75
# g_series = ones(1,24) .* 2

result_no_dc = Dict{String, Any}(["$idx"=>Dict{String, Any}() for idx = 1:length(fmin)])
result_dc = Dict{String, Any}(["$idx"=>Dict{String, Any}() for idx = 1:length(fmin)])
result_no_dc_relax = Dict{String, Any}(["$idx"=>Dict{String, Any}() for idx = 1:length(fmin)])
result_dc_relax = Dict{String, Any}(["$idx"=>Dict{String, Any}() for idx = 1:length(fmin)])
for idx = 1:length(fmin)
    # Add frequency stability data
    data["frequency_parameters"] = Dict{String, Any}()
    data["frequency_parameters"]["fmin"] = fmin[idx]
    data["frequency_parameters"]["f0"] = 50.0
    data["frequency_parameters"]["fmax"] =  data["frequency_parameters"]["f0"] + ((data["frequency_parameters"]["f0"] - fmin[idx]))
    data["frequency_parameters"]["t_fcr"] = 0.2
    data["frequency_parameters"]["uc_time_interval"] = 1 # hours
    # Add generator contingencies
    mn_data = CbaOPF.create_multinetwork_model!(data, number_of_hours, g_series, l_series)
    # Provide addtional settings as part of the PowerModels settings dictionary
    s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => false, "relax_uc_binaries" => false)
    result_no_dc["$idx"] = CbaOPF.solve_fsuc(mn_data, _PM.DCPPowerModel, gurobi, setting = s, multinetwork = true)
    objective_no_dc[1, idx] = result_no_dc["$idx"]["objective"] 

    s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => false, "relax_uc_binaries" => true)
    result_no_dc_relax["$idx"] = CbaOPF.solve_fsuc(mn_data, _PM.DCPPowerModel, gurobi, setting = s, multinetwork = true)
    objective_no_dc_relax[1, idx] = result_no_dc_relax["$idx"]["objective"] 

    s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => true, "relax_uc_binaries" => false)
    result_dc["$idx"] = CbaOPF.solve_fsuc(mn_data, _PM.DCPPowerModel, gurobi, setting = s, multinetwork = true)
    objective_dc[1, idx] = result_dc["$idx"]["objective"] 

    s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => true, "relax_uc_binaries" => true)
    result_dc_relax["$idx"] = CbaOPF.solve_fsuc(mn_data, _PM.DCPPowerModel, gurobi, setting = s, multinetwork = true)
    objective_dc_relax[1, idx] = result_dc_relax["$idx"]["objective"] 
end

p1 = Plots.plot(fmin, objective_no_dc'/1e6, xlabel = "\$f_{min} in~Hz\$", ylabel = "\$Cost~in~M€\$", label = "without HVDC contribution")
Plots.plot!(p1, fmin, objective_dc'/1e6,  xlabel = "\$f_{min} in~Hz\$", ylabel = "\$Cost~in~M€\$", label = "with HVDC contribution")
# Plots.plot!(p1, fmin, objective_no_dc_relax'/1e6,  xlabel = "\$f_{min} in~Hz\$", ylabel = "\$Cost~in~M€\$", label = "without HVDC contribution - relax")
# Plots.plot!(p1, fmin, objective_dc_relax'/1e6,  xlabel = "\$f_{min} in~Hz\$", ylabel = "\$Cost~in~M€\$", label = "with HVDC contribution - relax")
plot_filename = joinpath(plot_path, "objective_comparison.pdf")
Plots.savefig(p1, plot_filename)

f_idx = 17
mn_data = CbaOPF.create_multinetwork_model!(data, number_of_hours, g_series, l_series)
number_of_generators = length(data["gen"])
alpha_dc = zeros(number_of_hours, number_of_generators)
alpha_p_dc = zeros(number_of_hours, number_of_generators)
color_dc = Array{String}(undef, number_of_hours, number_of_generators)

alpha_no_dc = zeros(number_of_hours, number_of_generators)
alpha_p_no_dc = zeros(number_of_hours, number_of_generators)
color_no_dc = Array{String}(undef, number_of_hours, number_of_generators)
interval =  mn_data["number_of_contingencies"] 
solutions =  mn_data["number_of_contingencies"] * mn_data["number_of_hours"]
for g = 1:number_of_generators
    alpha_dc[:, g] = [result_dc["$f_idx"]["solution"]["nw"]["$idx"]["gen"]["$g"]["alpha_g"] for idx in 1:interval:solutions]
    alpha_no_dc[:, g] = [result_no_dc["$f_idx"]["solution"]["nw"]["$idx"]["gen"]["$g"]["alpha_g"] for idx in 1:interval:solutions]
end

for i in 1:size(alpha_dc, 1)
    for j in 1:size(alpha_dc, 2)
        if alpha_dc[i,j] == 1
            color_dc[i,j] = "black"
        else
            color_dc[i,j] = "white"
        end
        if alpha_no_dc[i,j] == 1
            color_no_dc[i,j] = "black"
        else
            color_no_dc[i,j] = "white"
        end
        alpha_p_dc[i,j] = j
        alpha_p_no_dc[i,j] = j
    end
end

p3 = Plots.scatter(alpha_p_dc,color=color_dc,legend=nothing, xlabel = "\$hour\$", ylabel = "\$g_{id}\$", title = "generator status with HVDC contribution")
plot_filename = joinpath(plot_path, "generator_status_dc.pdf")
Plots.savefig(p3, plot_filename)

p4 = Plots.scatter(alpha_p_no_dc,color=color_no_dc,legend=nothing, xlabel = "\$hour\$", ylabel = "\$g_{id}\$", title = "generator status without HVDC contribution")
plot_filename = joinpath(plot_path, "generator_status_no_dc.pdf")
Plots.savefig(p4, plot_filename)

p5 = Plots.plot(1:number_of_hours,g_series', xlabel = "\$hour\$", ylabel = "\$profile in p.u.\$", label = "RES generation profile")
Plots.plot!(p5, 1:number_of_hours,l_series', xlabel = "\$hour\$", ylabel = "\$profile in p.u.\$", label = "demand profile")
plot_filename = joinpath(plot_path, "profiles.pdf")
Plots.savefig(p5, plot_filename)

p2 = Plots.plot(fmin, tie_line_flows, xlabel = "\$f_{min} in~Hz\$", ylabel = "\$Flow~in~MVA\$", label = "tie line flow zone 1 -> zone 2, without HVDC contribution")
Plots.plot!(p2, fmin, tie_line_flows_dc,  xlabel = "\$f_{min} in~Hz\$", ylabel = "\$Cost~in~€\$", label = "tie line flow zone 1 -> zone 2, with HVDC contribution")
plot_filename = joinpath(plot_path, "tieline_flows.pdf")
Plots.savefig(p2, plot_filename)


res = result_no_dc["$f_idx"]
res_dc = result_dc["$f_idx"]
for g in sort(parse.(Int, collect(keys(res["solution"]["nw"]["1"]["gen"]))))
    print("No HVDC: Generator, ", g, " dispatch = ", res["solution"]["nw"]["1"]["gen"]["$g"]["pg"], "\n")
    print("With HVDC: Generator, ", g, " dispatch = ", res_dc["solution"]["nw"]["1"]["gen"]["$g"]["pg"], "\n")
end
for l in sort(parse.(Int, collect(keys(res["solution"]["nw"]["1"]["load"]))))
    print("No HVDC: Load, ", l, " curtailment = ", res["solution"]["nw"]["1"]["load"]["$l"]["pcurt"], "\n")
    print("With HVDC: Load, ", l, " curtailment = ", res_dc["solution"]["nw"]["1"]["load"]["$l"]["pcurt"], "\n")
end

# fmin_idx = f_idx
# print("########### NO HVDC contribution for fmin = ", fmin[fmin_idx], " Hz #####################", "\n")
# CbaOPF.print_frequency_information(result_no_dc, mn_data; fmin_idx = fmin_idx)
# print("########### With HVDC contribution for fmin = ", fmin[fmin_idx], " Hz #####################", "\n")
# CbaOPF.print_frequency_information(result_dc, mn_data; fmin_idx = fmin_idx)
# br_idx = data["tie_lines"]["1"]["br_idx"]
# tie_line_flows = [result_no_dc["$idx"]["solution"]["nw"]["1"]["branch"]["$br_idx"]["pf"] for idx in 1:length(fmin)] * data["baseMVA"]
# tie_line_flows_dc = [result_dc["$idx"]["solution"]["nw"]["1"]["branch"]["$br_idx"]["pf"] for idx in 1:length(fmin)] * data["baseMVA"]

# p1 = Plots.plot(fmin, objective_no_dc', xlabel = "\$f_{min} in~Hz\$", ylabel = "\$Cost~in~€\$", label = "total cost")
# plot_filename = joinpath(plot_path, "objective_comparison.pdf")
# Plots.savefig(p1, plot_filename)