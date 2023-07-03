# Define solvers
highs = JuMP.optimizer_with_attributes(HiGHS.Optimizer)

# Data

@testset "2 Grids HVDC" begin
    file = "../test/data/case5_2grids_inertia_acdc.m"
    # Parse file using PowerModels
    data = PowerModels.parse_file(file)
    # Process demand reduction and curtailment data
    CbaOPF.add_flexible_demand_data!(data)
    # Process inertia data
    CbaOPF.prepare_data!(data; uc = true, t_hvdc = 0.10, ffr_cost = 50.0)

    fmin = 49.8

    number_of_hours = 1
    l_series = [1]
    g_series = [1]

    data["frequency_parameters"] = Dict{String, Any}()
    data["frequency_parameters"]["fmin"] = fmin
    data["frequency_parameters"]["f0"] = 50.0
    data["frequency_parameters"]["fmax"] =  data["frequency_parameters"]["f0"] + ((data["frequency_parameters"]["f0"] - fmin))
    data["frequency_parameters"]["t_fcr"] = 0.2
    data["frequency_parameters"]["uc_time_interval"] = 1 # hours
    mn_data = CbaOPF.create_multinetwork_model!(data, number_of_hours, g_series, l_series)

    @testset "FS UC, no HVDC, no binary relaxation" begin
    s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => false)
    result = CbaOPF.solve_fsuc(mn_data, _PM.DCPPowerModel, highs, setting = s, multinetwork = true)
    @test isapprox(result["objective"], 3.48605e5, atol = 1e2)
    end

    @testset "FS UC, with HVDC, no binary relaxation" begin
        s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => true)
        result = CbaOPF.solve_fsuc(mn_data, _PM.DCPPowerModel, highs, setting = s, multinetwork = true)
        @test isapprox(result["objective"], 327963, atol = 1e2)
    end
end