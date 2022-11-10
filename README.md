Status:
[![CI](https://github.com/Electa-Git/CBAOPF.jl/workflows/CI/badge.svg)](https://github.com/Electa-Git/CBAOPF.jl/actions?query=workflow%3ACI)
<a href="https://codecov.io/gh/Electa-Git/CBAOPF.jl"><img src="https://img.shields.io/codecov/c/github/Electa-Git/CBAOPF.jl?logo=Codecov"></img></a>



# CBAOPF.jl

A julia package based on PowerModels.jl, PowerModelsACDC.jl and FlexPlan.jl created to solve the AC/DC grid OPF calculations for CBA purposes. The package contains besides the classical AC/DC OPF also an optimisation of PST angles, demand response and load-shedding.

Following problem types are implemented:

- Economic dispatch model (network flow model) with inertia constraints per zone
- AC/DC grid OPF model with optimisation of HVDC converter set points, PST angles, demand response and load-shedding 
- Redispatch cost minimization for fixed and variable HVDC converter set points
- Start-up cost minimisation considering minimum inertia constraints

## How to use?

Add the package to your working environment using:

```julia
] develop https://github.com/Electa-Git/CBAOPF.jl.git
```

You can run the OPF in AC or DC formulation using the command:

```julia
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)
resultAC = CBAOPF.solve_cbaopf(data, _PM.ACPPowerModel, nlp_solver; setting = s)
resultDC = CBAOPF.solve_cbaopf(data, _PM.DCPPowerModel, lp_solver; setting = s)
```

where ```data``` is a PowerModels(ACDC) dictionary.

A sample script to run the OPF is given below, which uses an m-file (see folder examples) as input:

```julia
import PowerModelsACDC
const _PMACDC = PowerModelsACDC
import PowerModels
const _PM = PowerModels
import CBAOPF
import Ipopt
import Memento
import JuMP
import HiGHS  # needs startvalues for all variables!


# Load test file
file = "./test/data/case5_acdc_pst_3_grids.m"

# Define solvers
ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6)
highs = JuMP.optimizer_with_attributes(HiGHS.Optimizer)

# define nodes 13 & 14 of grid three as cross-border nodes (outside zone) with a dedicated flow of 200 MW in total (e.g. 2 pu), define some slack by which XB flows can deviate
borders = Dict(1 => Dict("xb_nodes" => [13 14], "flow" => 2, "slack" => 0))

# Parse file using PowerModels
data = PowerModels.parse_file(file)
# Process DC grid data
_PMACDC.process_additional_data!(data)
# Process demand reduction and curtailment data
CBAOPF.add_flexible_demand_data!(data)
# Process PST data
CBAOPF.process_pst_data!(data)
# Process generator and cross border flow data
CBAOPF.prepare_data!(data, borders = borders)


# Provide addtional settings as part of the PowerModels settings dictionary
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "fix_cross_border_flows" => true)

############ CASE 1: OPF problem ###############

# Let us run the CBA OPF with the objective of minimizinf total generation, demand recution and load shedding cost.
resultOPF = CBAOPF.solve_cbaopf(data, _PM.DCPPowerModel, highs; setting = s)

############ CASE 2: Redispacth minimization problem ###############

# Let us deactivate a line (branch 5) and run the redispatch minimisation problem
contingency = 1
# we define a redispatch cost factor of 2, e.g. redispatch cost = 2 * dispatch cost
rd_cost_factor = 2
# Write OPF solution as starting point to the redispatch minimisation problem
dataRD = CBAOPF.prepare_redispatch_data(resultOPF, data; contingency = contingency, rd_cost_factor = rd_cost_factor)
# Provide settings for the optimisation problem, here we fix the HVDC converter set points
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "fix_cross_border_flows" => true, "fix_converter_setpoints" => true, "inertia_limit" => false)
# Run optimisation problem
resultRD_no_control = CBAOPF.solve_rdopf(dataRD, _PM.DCPPowerModel, highs; setting = s) 
# Now we allow the HVDC converter set points to be determined optimally
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "fix_cross_border_flows" => true, "fix_converter_setpoints" => false, "inertia_limit" => false)
resultRD_with_control = CBAOPF.solve_rdopf(dataRD, _PM.DCPPowerModel, highs; setting = s) 
# Print the difference between both costs
print("Redispatch cost difference: ",  resultRD_no_control["objective"] - resultRD_with_control["objective"], "\n")

############ CASE 3: Inertia constrained start-up cost minimisation ####################

# we define inertia limits for zone 1, which cannot be achoeved without additional generation start-up
# the online generation in zone 1 is 13.3 pu (pmax), and the inertia constraint is determined as \sum(pmax) = limit / 0.9, thus a limit of 14.8 pu.*seconnds causes the start-up of generator 4
inertia_limit = Dict(1 => Dict("limit" => 14.8), 2 => Dict("limit" => 0), 3 => Dict("limit" => 0))
# we write the OPF results into the input data
dataRD = CBAOPF.prepare_redispatch_data(resultOPF, data; inertia_limit = inertia_limit)
# we update settings to include the inertia constraints
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "fix_cross_border_flows" => true, "fix_converter_setpoints" => false, "inertia_limit" => true)
#we perform the optimisation
resultRD_inertia = CBAOPF.solve_rdopf(dataRD, _PM.DCPPowerModel, highs; setting = s) 
# we inspect the results
print([gen["alpha_g"] for (g, gen) in resultRD_inertia["solution"]["gen"]] .- [gen["dispatch_status"] for (g, gen) in dataRD["gen"]],"\n")
print(resultRD_inertia["objective"], "\n")


###############  CASE 4: NTC based network flow model using zonal inertia constraints

# Load test file
file = "./test/data/case5_2_grids.m"
inertia_limit = Dict(1 => Dict("limit" => 12), 2 => Dict("limit" => 10))
# Parse file using PowerModels
data = PowerModels.parse_file(file)
# Add inertia limit to the data
data["inertia_limit"] = inertia_limit
# Process generator data
CBAOPF.prepare_data!(data)

s = Dict("output" => Dict("branch_flows" => true), "inertia_limit" => false)
resultOPF_zonal_no_limit =  CBAOPF.solve_zonal_inertia_opf(data, _PM.NFAPowerModel, highs; setting = s) 

s = Dict("output" => Dict("branch_flows" => true), "inertia_limit" => true)
resultOPF_zonal_with_limit =  CBAOPF.solve_zonal_inertia_opf(data, _PM.NFAPowerModel, highs; setting = s) 

print("Cost difference through inertia constraint: ", resultOPF_zonal_with_limit["objective"] - resultOPF_zonal_no_limit["objective"], "\n")
```

## Developed by:

Hakan Ergun - KU Leuven / EnergyVille
