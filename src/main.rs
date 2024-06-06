use itertools::Itertools;
use std::collections::HashMap;
use std::fs::File;
use std::io::Write;

use clap::Parser;
use fixed::traits::ToFixed;
use fixed::types::I16F16;
use nalgebra::{Vector2, Vector3};

mod geometry;
use crate::geometry::{
    point_away_from_normal, point_segment_distance, polygon_order_points, ray_intersect_segment,
    triangle_plane_intersection, Intersection, Plane, Triangle,
};

mod stl;
use crate::stl::read_stl_file;

mod config;
use crate::config::Config;

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
    input: String,
    output: String,
}

fn main() -> std::io::Result<()> {
    let cli = Args::parse();
    // TODO get config path from cli and read from file
    let config = Config::default();

    let triangles = read_stl_file(&cli.input)?;
    let mut output = File::create(&cli.output)?;
    slice_triangles(triangles, &config, &mut output)?;

    Ok(())
}

fn slice_triangles(
    triangles: Vec<Triangle>,
    config: &Config,
    output: &mut File,
) -> std::io::Result<()> {
    output.write_all(&init_gcode(config).as_bytes())?;
    let triangles = center_triangles(triangles, config);
    //println!("triangle {:?}", triangles);

    let max_height = triangles
        .iter()
        .map(|t| t.points.iter().map(|point| point.z))
        .flatten()
        .max()
        .unwrap();

    // TODO check that it fits in build volume
    // this probably means moving the object to the center of the build area?
    let layer_height: I16F16 = config.layer_height;
    let eps = 0.05.to_fixed();
    let mut slice_plane = Plane {
        normal: Vector3::new(0.to_fixed(), 0.to_fixed(), (-1).to_fixed()),
        distance: 0.to_fixed(),
    };
    let layers: u32 = (max_height / layer_height).to_num();
    output.write_all(&format!(";LAYER_COUNT:{}\n", layers).as_bytes())?;
    let mut extruder_position = 0.to_fixed();
    for layer in 1..layers {
        //for layer in 1..2 {
        slice_plane.distance = layer_height * (layer).to_fixed::<I16F16>();
        let intersections: Vec<Intersection> = triangles
            .iter()
            .map(|triange| triangle_plane_intersection(triange, &slice_plane, &eps))
            .flatten()
            .collect();
        let intersections = merge_duplicate_intersections(intersections);
        //println!("layer {}", layer);
        //for point in &fixed_points {
        //    print!("{}", point);
        //}
        let outline_points = polygon_order_points(intersections);
        //write_layer_image(layer, &outline_points);
        let infill_percent = if layer < 3 || layer > layers - 3 {
            1.to_fixed()
        } else {
            config.infill_percent
        };
        output.write_all(&format!("G0 F1200 Z{}\n", slice_plane.distance).as_bytes())?;
        let gcode = layer_gcode(
            layer,
            &outline_points,
            infill_percent,
            config,
            &mut extruder_position,
        );
        output.write_all(gcode.as_bytes())?;
    }
    output.write_all(&finish_gcode(config).as_bytes())?;
    Ok(())
}

fn center_triangles(triangles: Vec<Triangle>, config: &Config) -> Vec<Triangle> {
    // TODO actually center it instead of this nonesense. also panic if it doesn't fit in build
    // volume
    let x_center = config.build_volume.x / 2.to_fixed::<I16F16>();
    let y_center = config.build_volume.y / 2.to_fixed::<I16F16>();
    triangles
        .into_iter()
        .map(|mut triangle| {
            triangle.points[0].x += x_center;
            triangle.points[0].y += y_center;
            triangle.points[1].x += x_center;
            triangle.points[1].y += y_center;
            triangle.points[2].x += x_center;
            triangle.points[2].y += y_center;

            triangle
        })
        .collect()
}

// find duplicate points and add their normals together
fn merge_duplicate_intersections(intersects: Vec<Intersection>) -> Vec<Intersection> {
    let mut merged = HashMap::new();
    for intersect in intersects {
        if merged.contains_key(&intersect.point) {
            merged.insert(
                intersect.point,
                merged.get(&intersect.point).unwrap() + intersect.normal,
            );
        } else {
            merged.insert(intersect.point, intersect.normal);
        }
    }

    merged
        .into_iter()
        .map(|(point, normal)| Intersection { point, normal })
        .collect()
}

fn init_gcode(config: &Config) -> String {
    let mut output = String::new();
    // TODO skirt? brim? raft?
    output.push_str("; Haxaw Start G-code\n");
    output.push_str("G92 E0 ; Reset Extruder\n");
    output.push_str("G28 ; Home all axes\n");
    output.push_str("G1 Z5.0 F3000 ; Move Z Axis up a bit during heating to not damage bed\n");
    output.push_str(&format!(
        "M104 S{} ; Start heating up the nozzle most of the way\n",
        config.standby_nozzle_temp
    ));
    output.push_str(&format!(
        "M190 S{} ; Start heating the bed, wait until target temperature reached\n",
        config.bed_temp
    ));
    output.push_str(&format!(
        "M109 S{} ; Finish heating the nozzle\n",
        config.printing_nozzle_temp
    ));
    output.push_str("G1 Z2.0 F3000 ; Move Z Axis up little to prevent scratching of Heat Bed\n");
    output.push_str("G1 X0.1 Y20 Z0.3 F5000.0 ; Move to start position\n");
    output.push_str("G1 X0.1 Y200.0 Z0.3 F1500.0 E15 ; Draw the first line\n");
    output.push_str("G1 X0.4 Y200.0 Z0.3 F5000.0 ; Move to side a little\n");
    output.push_str("G1 X0.4 Y20 Z0.3 F1500.0 E30 ; Draw the second line\n");
    output.push_str("G92 E0 ; Reset Extruder\n");
    output.push_str("G1 Z2.0 F3000 ; Move Z Axis up little to prevent scratching of Heat Bed\n");
    output.push_str("G1 X5 Y20 Z0.3 F5000.0 ; Move over to prevent blob squish\n");
    output.push_str("M106 S85 ; Turn on fan\n");
    output
}

fn finish_gcode(config: &Config) -> String {
    let mut output = String::new();
    output.push_str("G91 ;Relative positioning\n");
    output.push_str("G1 E-2 F2700 ;Retract a bit\n");
    output.push_str("G1 E-2 Z0.2 F2400 ;Retract and raise Z\n");
    output.push_str("G1 X5 Y5 F3000 ;Wipe out\n");
    output.push_str("G1 Z10 ;Raise Z more\n");
    output.push_str("G90 ;Absolute positioning\n");
    output.push_str("\n");
    output.push_str(&format!(
        "G1 X0 Y{} ;Present print\n",
        config.build_volume.y
    ));
    output.push_str("M106 S0 ;Turn-off fan\n");
    output.push_str("M104 S0 ;Turn-off hotend\n");
    output.push_str("M140 S0 ;Turn-off bed\n");
    output.push_str("\n");
    output.push_str("M84 X Y E ;Disable all steppers but Z\n");
    output
}

fn layer_gcode(
    layer: u32,
    outline_points: &[Intersection],
    infill_percent: I16F16,
    config: &Config,
    extruder_position: &mut I16F16,
) -> String {
    let mut output = String::new();
    output.push_str(&format!(";LAYER:{}\n", layer));
    // TODO don't print 3 layers of walls when they're close to each other
    // do this by checking whether each wall point is inside the previous wall polygon and discard
    // if not
    let mut prev_wall: Option<Vec<[Vector2<I16F16>; 2]>> = None;
    let ray_direction: Vector2<I16F16> = Vector2::new(1.to_fixed(), 0.to_fixed());
    for i in 0..config.wall_count {
        output.push_str(if i == 0 {
            ";TYPE:WALL-OUTER\n"
        } else {
            ";TYPE:WALL-INNER\n"
        });
        //println!("wall {}: prev_wall: {:?}", i, prev_wall);
        let wall_points: Vec<_> = outline_points
            .iter()
            .map(|intersect| {
                point_away_from_normal(
                    intersect,
                    (config.nozzle_diameter / 2.to_fixed::<I16F16>())
                        + config.nozzle_diameter * i.to_fixed::<I16F16>(),
                )
            })
            .filter(|point| {
                // Filter out points that are within nozzle_diameter / 2 of the previous wall
                if let Some(prev_wall) = &prev_wall {
                    prev_wall.iter().all(|[start, end]| {
                        point_segment_distance(start, end, point) >= config.nozzle_diameter / 2
                    })
                } else {
                    true
                }
            })
            .filter(|point| {
                // Filter out points that are not inside the previous wall's polygon
                if let Some(prev_wall) = &prev_wall {
                    // Count number of sides a ray coming out of the point intersects
                    let intersect_count = prev_wall
                        .iter()
                        .map(|[start, end]| {
                            ray_intersect_segment(point, &ray_direction, start, end).is_some()
                        })
                        .filter(|b| *b)
                        .count();
                    //println!("point {}, intersect_count: {}", point, intersect_count);

                    // it's inside the polygon if the count is odd
                    intersect_count % 2 == 1
                } else {
                    true
                }
            })
            .collect();

        if wall_points.len() < 2 {
            break;
        }
        let mut wall_lines: Vec<[Vector2<I16F16>; 2]> = Vec::new();
        for i in 0..wall_points.len() {
            let start = wall_points[i];
            let end = wall_points[(i + 1) % wall_points.len()];
            wall_lines.push([start, end]);
        }

        output.push_str(&format!("G0 X{} Y{}\n", wall_points[0].x, wall_points[0].y));
        for [start, end] in wall_lines.iter() {
            let extrusion_distance = extrusion_distance(start, end, config);
            *extruder_position += extrusion_distance;

            // assume nozzle is already at start
            output.push_str(&format!(
                "G1 E{} X{} Y{}\n",
                extruder_position, end.x, end.y
            ));
        }
        //output.push_str(&format!("; end inner wall {}\n\n", i));
        prev_wall = Some(wall_lines);
    }

    // infill
    output.push_str(";TYPE:FILL\n");
    let unit_x: Vector2<I16F16> = Vector2::new(1.to_fixed(), 0.to_fixed());
    let unit_y: Vector2<I16F16> = Vector2::new(0.to_fixed(), 1.to_fixed());
    let (ray_direction, ray_origin_base) = if layer % 2 == 0 {
        (unit_y, unit_x)
    } else {
        (unit_x, unit_y)
    };
    let ray_origins = (0..)
        .map(|i| ray_origin_base * config.nozzle_diameter / infill_percent * i.to_fixed())
        .take_while(|point| point.x < config.build_volume.x && point.y < config.build_volume.y)
        .collect_vec();
    //println!("infill ray origins: {}", ray_origins.len());
    for ray_origin in ray_origins {
        let mut intersects: Vec<_> = prev_wall
            .as_ref()
            .unwrap()
            .iter()
            .map(|[segment_start, segment_end]| {
                ray_intersect_segment(&ray_origin, &ray_direction, segment_start, segment_end)
            })
            .flatten()
            .collect();
        intersects.sort_by_key(|point| point.y);
        //println!("intersect count: {}", intersects.len());
        assert!(intersects.len() % 2 == 0);
        let mut intersect_iter = intersects.iter();
        while let Some((start, end)) = intersect_iter.next_tuple() {
            //println!("start, end = {}, {}", start, end);
            output.push_str(&format!("G0 X{} Y{}\n", start.x, start.y));
            let extrusion_distance = extrusion_distance(start, end, config);
            *extruder_position += extrusion_distance;
            output.push_str(&format!(
                "G1 E{} X{} Y{}\n",
                extruder_position, end.x, end.y
            ));
        }
    }

    output
}

fn extrusion_distance(a: &Vector2<I16F16>, b: &Vector2<I16F16>, config: &Config) -> I16F16 {
    let diff = b - a;
    let line_length = (diff.x * diff.x + diff.y * diff.y).sqrt();
    let volume = config.layer_height * config.nozzle_diameter * line_length;
    let filament_crosssection_area = (config.filament_diameter / 2)
        * (config.filament_diameter / 2)
        * std::f32::consts::PI.to_fixed::<I16F16>();
    volume / filament_crosssection_area
}
