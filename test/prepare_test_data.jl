function prepare_test_data!(file)

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

return data

end