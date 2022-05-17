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