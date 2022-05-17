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

## Developed by:

Hakan Ergun - KU Leuven / EnergyVille
