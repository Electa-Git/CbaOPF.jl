# Define solvers
highs = JuMP.optimizer_with_attributes(HiGHS.Optimizer)

# Data

@testset "2 Grids HVDC" begin
    file = "../test/data/case5_2grids_inertia_hvdc.m"
    # Parse file using PowerModels
    data = PowerModels.parse_file(file)
    # Process demand reduction and curtailment data
    CbaOPF.add_flexible_demand_data!(data)
    # Process inertia data
    CbaOPF.prepare_data!(data; uc = true, t_hvdc = 0.10, ffr_cost = 50.0)

    number_of_hours = 24
    fmin = 49.8

    g_series = [0.4  0.5  0.66  0.7   0.7   0.9   0.95 1.02  1.15  1.3   1.35 1.3   1.21  1.08  1.0  0.96  0.93 1.0  1.1   1.2  1.08  1.05  0.99 0.89]
    l_series = [0.6  0.7  0.75  0.78  0.85  0.88  0.9  1.0   1.12  1.25  1.2  1.08  0.99  0.92  0.8  0.73  0.8  0.9  1.03  1.2  1.11  0.99  0.8  0.69]

    data["frequency_parameters"] = Dict{String, Any}()
    data["frequency_parameters"]["fmin"] = fmin
    data["frequency_parameters"]["f0"] = 50.0
    data["frequency_parameters"]["fmax"] =  data["frequency_parameters"]["f0"] + ((data["frequency_parameters"]["f0"] - fmin))
    data["frequency_parameters"]["t_fcr"] = 0.2
    data["frequency_parameters"]["uc_time_interval"] = 1 # hours
    mn_data = CbaOPF.create_multinetwork_model!(data, number_of_hours, g_series, l_series)

    @testset "FS UC, no HVDC, no binary relaxation" begin
    s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => false, "relax_uc_binaries" => false)
    result = CbaOPF.solve_fsuc(mn_data, _PM.DCPPowerModel, highs, setting = s, multinetwork = true)
    @test isapprox(result["objective"], 3.6787584502607845e7, atol = 1e3)
    end

    @testset "FS UC, with HVDC, no binary relaxation" begin
        s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => true, "relax_uc_binaries" => false)
        result = CbaOPF.solve_fsuc(mn_data, _PM.DCPPowerModel, highs, setting = s, multinetwork = true)
        @test isapprox(result["objective"], 1.3438685762374746e6, atol = 1e2)
    end


    @testset "FS UC, no HVDC, with binary relaxation" begin
        s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => false, "relax_uc_binaries" => true)
        result = CbaOPF.solve_fsuc(mn_data, _PM.DCPPowerModel, highs, setting = s, multinetwork = true)
        @test isapprox(result["objective"], 3.6787584502607845e7, atol = 1e3)
    end
        
    @testset "FS UC, with HVDC, with binary relaxation" begin
            s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => true, "relax_uc_binaries" => true)
            result = CbaOPF.solve_fsuc(mn_data, _PM.DCPPowerModel, highs, setting = s, multinetwork = true)
            @test isapprox(result["objective"], 1.3438685762374746e6, atol = 1e2)
    end
end

@testset "2 Grids ACDC" begin
    file = "../test/data/case5_2grids_inertia_acdc.m"
    # Parse file using PowerModels
    data = PowerModels.parse_file(file)
    # Process demand reduction and curtailment data
    CbaOPF.add_flexible_demand_data!(data)
    # Process inertia data
    CbaOPF.prepare_data!(data; uc = true, t_hvdc = 0.10, ffr_cost = 50.0)

    number_of_hours = 24
    fmin = 49.8

    g_series = [0.4  0.5  0.66  0.7   0.7   0.9   0.95 1.02  1.15  1.3   1.35 1.3   1.21  1.08  1.0  0.96  0.93 1.0  1.1   1.2  1.08  1.05  0.99 0.89]
    l_series = [0.6  0.7  0.75  0.78  0.85  0.88  0.9  1.0   1.12  1.25  1.2  1.08  0.99  0.92  0.8  0.73  0.8  0.9  1.03  1.2  1.11  0.99  0.8  0.69]

    data["frequency_parameters"] = Dict{String, Any}()
    data["frequency_parameters"]["fmin"] = fmin
    data["frequency_parameters"]["f0"] = 50.0
    data["frequency_parameters"]["fmax"] =  data["frequency_parameters"]["f0"] + ((data["frequency_parameters"]["f0"] - fmin))
    data["frequency_parameters"]["t_fcr"] = 0.2
    data["frequency_parameters"]["uc_time_interval"] = 1 # hours
    mn_data = CbaOPF.create_multinetwork_model!(data, number_of_hours, g_series, l_series)

    @testset "FS UC, no HVDC, no binary relaxation" begin
    s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => false, "relax_uc_binaries" => false)
    result = CbaOPF.solve_fsuc(mn_data, _PM.DCPPowerModel, highs, setting = s, multinetwork = true)
    @test isapprox(result["objective"], 1.3299042434953006e6, atol = 1e3)
    end

    @testset "FS UC, with HVDC, no binary relaxation" begin
        s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => true, "relax_uc_binaries" => false)
        result = CbaOPF.solve_fsuc(mn_data, _PM.DCPPowerModel, highs, setting = s, multinetwork = true)
        @test isapprox(result["objective"], 1.3299042434953016e6, atol = 1e2)
    end


    @testset "FS UC, no HVDC, with binary relaxation" begin
        s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => false, "relax_uc_binaries" => true)
        result = CbaOPF.solve_fsuc(mn_data, _PM.DCPPowerModel, highs, setting = s, multinetwork = true)
        @test isapprox(result["objective"], 1.3299042434953016e6, atol = 1e3)
    end
        
    @testset "FS UC, with HVDC, with binary relaxation" begin
            s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => true, "relax_uc_binaries" => true)
            result = CbaOPF.solve_fsuc(mn_data, _PM.DCPPowerModel, highs, setting = s, multinetwork = true)
            @test isapprox(result["objective"], 1.3299042434953016e6, atol = 1e2)
    end
end

@testset "3 Grids HVDC" begin
    file = "../test/data/case5_3grids_inertia_hvdc.m"
    # Parse file using PowerModels
    data = PowerModels.parse_file(file)
    # Process demand reduction and curtailment data
    CbaOPF.add_flexible_demand_data!(data)
    # Process inertia data
    CbaOPF.prepare_data!(data; uc = true, t_hvdc = 0.10, ffr_cost = 50.0)

    number_of_hours = 24
    fmin = 49.8

    g_series = [0.4  0.5  0.66  0.7   0.7   0.9   0.95 1.02  1.15  1.3   1.35 1.3   1.21  1.08  1.0  0.96  0.93 1.0  1.1   1.2  1.08  1.05  0.99 0.89]
    l_series = [0.6  0.7  0.75  0.78  0.85  0.88  0.9  1.0   1.12  1.25  1.2  1.08  0.99  0.92  0.8  0.73  0.8  0.9  1.03  1.2  1.11  0.99  0.8  0.69]

    data["frequency_parameters"] = Dict{String, Any}()
    data["frequency_parameters"]["fmin"] = fmin
    data["frequency_parameters"]["f0"] = 50.0
    data["frequency_parameters"]["fmax"] =  data["frequency_parameters"]["f0"] + ((data["frequency_parameters"]["f0"] - fmin))
    data["frequency_parameters"]["t_fcr"] = 0.2
    data["frequency_parameters"]["uc_time_interval"] = 1 # hours
    mn_data = CbaOPF.create_multinetwork_model!(data, number_of_hours, g_series, l_series)

    @testset "FS UC, no HVDC, no binary relaxation" begin
    s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => false, "relax_uc_binaries" => false)
    result = CbaOPF.solve_fsuc(mn_data, _PM.DCPPowerModel, highs, setting = s, multinetwork = true)
    @test isapprox(result["objective"], 1.3420141930823623e8, atol = 1e3)
    end

    @testset "FS UC, with HVDC, no binary relaxation" begin
        s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => true, "relax_uc_binaries" => false)
        result = CbaOPF.solve_fsuc(mn_data, _PM.DCPPowerModel, highs, setting = s, multinetwork = true)
        @test isapprox(result["objective"], 1.4971006915876116e6, atol = 1e2)
    end


    @testset "FS UC, no HVDC, with binary relaxation" begin
        s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => false, "relax_uc_binaries" => true)
        result = CbaOPF.solve_fsuc(mn_data, _PM.DCPPowerModel, highs, setting = s, multinetwork = true)
        @test isapprox(result["objective"], 1.3420141930823623e8, atol = 1e3)
    end
        
    @testset "FS UC, with HVDC, with binary relaxation" begin
            s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true, "hvdc_inertia_contribution" => true, "relax_uc_binaries" => true)
            result = CbaOPF.solve_fsuc(mn_data, _PM.DCPPowerModel, highs, setting = s, multinetwork = true)
            @test isapprox(result["objective"], 1.4971006915876116e6, atol = 1e2)
    end
end
