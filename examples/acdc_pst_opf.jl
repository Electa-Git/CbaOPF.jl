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
resultAC = CBAOPF.solve_cbaopf(file, _PM.ACPPowerModel, ipopt; setting = s)
resultDC = CBAOPF.solve_cbaopf(file, _PM.DCPPowerModel, ipopt; setting = s)