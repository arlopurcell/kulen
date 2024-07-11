use fixed::traits::ToFixed;
use fixed::types::I16F16;
use nalgebra::Vector3;

pub struct Config {
    // Printer settings
    pub nozzle_diameter: I16F16,
    pub filament_diameter: I16F16,
    pub build_volume: Vector3<I16F16>,

    // Slice settings
    pub layer_height: I16F16,
    pub infill_percent: I16F16,
    pub wall_count: usize,
    pub wall_layers: usize,

    // Material settings
    pub standby_nozzle_temp: u32,
    pub printing_nozzle_temp: u32,
    pub bed_temp: u32,
}

impl Config {
    pub fn default() -> Self {
        Config {
            nozzle_diameter: 0.4.to_fixed(),
            filament_diameter: 1.75.to_fixed(),
            build_volume: Vector3::new(220.to_fixed(), 220.to_fixed(), 250.to_fixed()),

            layer_height: 0.2.to_fixed(),
            infill_percent: 0.3.to_fixed(),
            wall_count: 3,
            wall_layers: 3,

            standby_nozzle_temp: 175,
            printing_nozzle_temp: 200,
            bed_temp: 60,
        }
    }
}
