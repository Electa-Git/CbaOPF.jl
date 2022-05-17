# CBAOPF.jl

A julia package based on PowerModels.jl, PowerModelsACDC.jl and FlexPlan.jl created to solve the AC/DC grid OPF calculations for CBA purposes. The package contains besides the classical AC/DC OPF also an optimisation of PST angles, demand response and load-shedding.

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

A sample script to run the OPF is given below, which uses an m-file (see test folder) as input:

```julia
import PowerModelsACDC
const _PMACDC = PowerModelsACDC
import PowerModels
const _PM = PowerModels
import CBAOPF
import Ipopt
import Memento
import JuMP
import Gurobi  # needs startvalues for all variables!

file = "./test/data/case5_acdc_pst.m"

ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0)
gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer)

s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

data = PowerModels.parse_file(file)
_PMACDC.process_additional_data!(data)
CBAOPF.add_flexible_demand_data!(data)
CBAOPF.process_pst_data!(data)

resultAC = CBAOPF.solve_cbaopf(data, _PM.ACPPowerModel, ipopt; setting = s)
resultDC = CBAOPF.solve_cbaopf(data, _PM.DCPPowerModel, gurobi; setting = s)
```

## Developed by:

Hakan Ergun - KU Leuven / EnergyVille
