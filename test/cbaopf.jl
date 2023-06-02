# Define solvers
ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6)
highs = JuMP.optimizer_with_attributes(HiGHS.Optimizer)


# Provide addtional settings as part of the PowerModels settings dictionary
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "fix_cross_border_flows" => true)

############ CASE 1: OPF problem ###############
resultOPF = CbaOPF.solve_cbaopf(data, _PM.DCPPowerModel, highs; setting = s)

@testset  "Nodal CBA OPF" begin

    @test isapprox(resultOPF["objective"], 34021.7, atol = 1e-1)
    @test isapprox(resultOPF["solution"]["gen"]["3"]["pg"], 0.955, atol = 1e-2)
    @test isapprox(resultOPF["solution"]["branch"]["2"]["pt"], -1.202, atol = 1e-2)
    @test isapprox(resultOPF["solution"]["convdc"]["4"]["ptf_to"], 0.977907, atol = 1e-2)
end

# Let us deactivate a line (branch 5) and run the redispatch minimisation problem
contingency = 4
# we define a redispatch cost factor of 2, e.g. redispatch cost = 2 * dispatch cost
rd_cost_factor = 2
# Write OPF solution as starting point to the redispatch minimisation problem
dataRD = CbaOPF.prepare_redispatch_data(resultOPF, data; contingency = contingency, rd_cost_factor = rd_cost_factor)

@testset "Redispatch OPF" begin
# Provide settings for the optimisation problem, here we fix the HVDC converter set points
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => false, "fix_cross_border_flows" => true, "fix_converter_setpoints" => true, "inertia_limit" => false)
# Run optimisation problem
resultRD_no_control = CbaOPF.solve_rdopf(dataRD, _PM.DCPPowerModel, highs; setting = s) 
# Now we allow the HVDC converter set points to be determined optimally
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => false, "fix_cross_border_flows" => true, "fix_converter_setpoints" => false, "inertia_limit" => false)
resultRD_with_control = CbaOPF.solve_rdopf(dataRD, _PM.DCPPowerModel, highs; setting = s) 

@test isapprox(resultRD_no_control["objective"] - resultRD_with_control["objective"] , 5485.38, atol = 1e-1)

end

# we define inertia limits for zone 1, which cannot be achoeved without additional generation start-up
# the online generation in zone 1 is 13.3 pu (pmax), and the inertia constraint is determined as \sum(pmax) = limit / 0.9, thus a limit of 14.8 pu.*seconnds causes the start-up of generator 4
inertia_limit = Dict(1 => Dict("limit" => 14.8), 2 => Dict("limit" => 0), 3 => Dict("limit" => 0))
# we write the OPF results into the input data
dataRD = CbaOPF.prepare_redispatch_data(resultOPF, data; inertia_limit = inertia_limit)
# we update settings to include the inertia constraints
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "fix_cross_border_flows" => true, "fix_converter_setpoints" => false, "inertia_limit" => true)
#we perform the optimisation

@testset "Inertia redispatch OPF" begin
    # we define inertia limits for zone 1, which cannot be achoeved without additional generation start-up
    # the online generation in zone 1 is 13.3 pu (pmax), and the inertia constraint is determined as \sum(pmax) = limit / 0.9, thus a limit of 14.8 pu.*seconnds causes the start-up of generator 4
    inertia_limit = Dict(1 => Dict("limit" => 14.8), 2 => Dict("limit" => 0), 3 => Dict("limit" => 0))
    # we write the OPF results into the input data
    dataRD = CbaOPF.prepare_redispatch_data(resultOPF, data; inertia_limit = inertia_limit)
    # we update settings to include the inertia constraints
    s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "fix_cross_border_flows" => true, "fix_converter_setpoints" => false, "inertia_limit" => true)
    #we perform the optimisation
    result = CbaOPF.solve_rdopf(dataRD, _PM.DCPPowerModel, highs; setting = s) 

    @test isapprox(result["solution"]["gen"]["1"]["alpha_g"], 1.0, atol = 1e-3)
    @test isapprox(result["solution"]["gen"]["8"]["alpha_g"] - dataRD["gen"]["8"]["dispatch_status"], 0.0, atol = 1e-3)
    @test isapprox(result["objective"], 800.0, atol = 1e-2)
end


#### AC OPF problem

# Provide addtional settings as part of the PowerModels settings dictionary
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "fix_cross_border_flows" => false)

############ OPF problem ###############
resultOPF = CbaOPF.solve_cbaopf(data, _PM.ACPPowerModel, ipopt; setting = s)

@testset  "Nodal AC - CBA OPF" begin

    @test isapprox(resultOPF["objective"], 24715.7, atol = 1e-1)
    @test isapprox(resultOPF["solution"]["gen"]["3"]["pg"], 1.205709, atol = 1e-2)
    @test isapprox(resultOPF["solution"]["branch"]["2"]["pt"], -1.70106, atol = 1e-2)
    @test isapprox(resultOPF["solution"]["convdc"]["4"]["ptf_to"], 0.80970, atol = 1e-2)
end

resultOPF = CbaOPF.solve_cbaopf(data, _PM.SOCWRPowerModel, ipopt; setting = s)

@testset  "Nodal SOC - CBA OPF" begin
    @test isapprox(resultOPF["objective"], 3113.2, atol = 1e-1)
    @test isapprox(resultOPF["solution"]["gen"]["3"]["pg"], 0, atol = 1e-2)
    @test isapprox(resultOPF["solution"]["branch"]["2"]["pt"], -0.0074, atol = 1e-2)
    @test isapprox(resultOPF["solution"]["convdc"]["4"]["ptf_to"], -0.005477, atol = 1e-2)
end

resultOPF = CbaOPF.solve_cbaopf(data, _PM.LPACCPowerModel, ipopt; setting = s)

@testset  "Nodal LPACC - CBA OPF" begin

    @test isapprox(resultOPF["objective"],  24519.2, atol = 1e-1)
    @test isapprox(resultOPF["solution"]["gen"]["3"]["pg"], 1.15441, atol = 1e-2)
    @test isapprox(resultOPF["solution"]["branch"]["2"]["pt"], -1.75847, atol = 1e-2)
    @test isapprox(resultOPF["solution"]["convdc"]["4"]["ptf_to"], 0.831186, atol = 1e-2)
end