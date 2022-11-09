import PowerModelsACDC
const _PMACDC = PowerModelsACDC
import PowerModels
const _PM = PowerModels
import CBAOPF
import Ipopt
import Memento
import JuMP
import HiGHS  # needs startvalues for all variables!

# Suppress warnings during testing.
Memento.setlevel!(Memento.getlogger(_PMACDC), "error")
Memento.setlevel!(Memento.getlogger(_PM), "error")


using Test

include("prepare_test_data.jl")
data = prepare_test_data!("./test/data/case5_acdc_pst_3_grids.m")

@testset "CBAOPF" begin

include("cbaopf.jl")


end