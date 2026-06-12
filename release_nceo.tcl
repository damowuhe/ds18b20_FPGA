# Open project and release dual-purpose pins, then compile
project_open led
set_global_assignment -name RESERVE_NCEO_AFTER_CONFIGURATION "Use as regular IO"
export_assignments
puts "NCEO released. Starting compilation..."

# Run each step
execute_module -tool map
execute_module -tool fit
execute_module -tool asm

puts "Compilation done."
project_close
