use std::collections::HashMap;
use std::fs::File;
use std::fs::OpenOptions;
use std::io::Write;
use std::iter::once;

use clap::Parser;
use fixed::traits::ToFixed;
use fixed::types::I16F16;
use nalgebra::{Vector2, Vector3};
use stl_io::{IndexedMesh, IndexedTriangle};

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

    let mesh = read_stl_file(&cli.input)?;
    let mut output = File::create(&cli.output)?;
    slice_mesh(&mesh, &config, &mut output)?;

    Ok(())
}

fn read_stl_file(path: &str) -> std::io::Result<IndexedMesh> {
    let mut file = OpenOptions::new().read(true).open(path)?;
    let mut reader = stl_io::create_stl_reader(&mut file)?;
    reader.as_indexed_triangles()
}

//type Triangle = (Vector3<f32>, Vector3<f32>, Vector3<f32>);
struct Triangle {
    points: [Vector3<I16F16>; 3],
    normal: Vector3<I16F16>,
}
struct Plane {
    normal: Vector3<I16F16>,
    distance: I16F16,
}

struct Config {
    layer_height: I16F16,
    nozzle_diameter: I16F16,
    infill_percent: I16F16,
    filament_diameter: I16F16,
    wall_count: usize,
    // TODO temperatures
}

impl Config {
    fn default() -> Self {
        Config {
            layer_height: 0.2.to_fixed(),
            nozzle_diameter: 0.4.to_fixed(),
            infill_percent: 0.3.to_fixed(),
            filament_diameter: 1.75.to_fixed(),

            wall_count: 3,
        }
    }
}

fn slice_mesh(mesh: &IndexedMesh, config: &Config, output: &mut File) -> std::io::Result<()> {
    output.write_all(&init_gcode().as_bytes())?;
    let triangles: Vec<_> = mesh
        .faces
        .iter()
        .map(|face| deindex_triangle(face, mesh))
        .collect();
    //println!("triangle {:?}", triangles);

    let max_height = triangles
        .iter()
        .map(|t| t.points.iter().map(|point| point.z))
        .flatten()
        .max()
        .unwrap();
    let layer_height: I16F16 = config.layer_height;
    let eps = 0.05.to_fixed();
    let mut slice_plane = Plane {
        normal: Vector3::new(0.to_fixed(), 0.to_fixed(), (-1).to_fixed()),
        distance: 0.to_fixed(),
    };
    let layers: u32 = (max_height / layer_height).to_num();
    for layer in 0..layers {
        //for layer in 0..1 {
        slice_plane.distance = layer_height * layer.to_fixed::<I16F16>();
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
        let gcode = layer_gcode(&outline_points, infill_percent, config);
        output.write_all(&format!("G0 Z{}\n", slice_plane.distance).as_bytes())?;
        output.write_all(gcode.as_bytes())?;
    }
    Ok(())
}

// find duplicates points and add their normals together
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

fn init_gcode() -> String {
    let mut output = String::new();
    // TODO set temps
    // TODO skirt? brim? raft?
    // TODO set mode = absolute
    // TODO set extruder relative mode
    output.push_str("; init gcode will go here\n");
    output
}

fn layer_gcode(outline_points: &[Intersection], infill_percent: I16F16, config: &Config) -> String {
    // TODO move points config.nozzle_diameter / 2 into the middle so the outer edge of the result
    // is in the right place
    let mut output = String::new();

    // TODO don't print 3 layers of walls when they're close to each other
    for i in 0..config.wall_count {
        output.push_str(&format!("; start outer wall {}\n", i));
        let wall_points: Vec<_> = outline_points
            .iter()
            .map(|intersect| {
                point_away_from_normal(
                    intersect,
                    (config.nozzle_diameter / 2.to_fixed::<I16F16>())
                        + config.nozzle_diameter * i.to_fixed::<I16F16>(),
                )
            })
            .collect();
        output.push_str(&format!("G0 X{} Y{}\n", wall_points[0].x, wall_points[0].y));
        for i in 0..wall_points.len() {
            let start = wall_points[i];
            let end = wall_points[(i + 1) % wall_points.len()];
            let extrusion_distance = extrusion_distance(start, end, config);

            // assume nozzle is already at start
            output.push_str(&format!(
                "G1 E{} X{} Y{}\n",
                extrusion_distance, end.x, end.y
            ));
        }
        output.push_str(&format!("; end outer wall {}\n\n", i));
    }

    // TODO infill

    output
}

fn extrusion_distance(a: Vector2<I16F16>, b: Vector2<I16F16>, config: &Config) -> I16F16 {
    let diff = b - a;
    let line_length = (diff.x * diff.x + diff.y * diff.y).sqrt();
    let volume = config.layer_height * config.nozzle_diameter * line_length;
    let filament_crosssection_area = (config.filament_diameter / 2)
        * (config.filament_diameter / 2)
        * std::f32::consts::PI.to_fixed::<I16F16>();
    volume / filament_crosssection_area
}

fn point_away_from_normal(intersect: &Intersection, dist: I16F16) -> Vector2<I16F16> {
    let transform = intersect.normal.clone();
    let normalized = transform / (transform.x * transform.x + transform.y * transform.y).sqrt();
    let transform = -normalized * dist;
    intersect.point + transform
}

fn polygon_order_points(mut intersects: Vec<Intersection>) -> Vec<Intersection> {
    intersects.sort_by_key(|p| p.point.x);
    let p = intersects.remove(0);
    let q = intersects.pop().unwrap();
    let (a, mut b): (Vec<_>, Vec<_>) = intersects
        .into_iter()
        .partition(|x| is_above(&p.point, &q.point, &x.point));
    b.reverse();
    once(p)
        .chain(a.into_iter())
        .chain(once(q))
        .chain(b.into_iter())
        .collect()
}

fn is_above(p: &Vector2<I16F16>, q: &Vector2<I16F16>, x: &Vector2<I16F16>) -> bool {
    let pq = q - p;
    let px = x - p;
    let cross = pq.x * px.y - px.x * pq.y;
    cross > 0.
}

fn deindex_triangle(face: &IndexedTriangle, mesh: &IndexedMesh) -> Triangle {
    let points = [
        stl_vector_to_nalg_fixed(mesh.vertices.get(face.vertices[0]).unwrap()),
        stl_vector_to_nalg_fixed(mesh.vertices.get(face.vertices[1]).unwrap()),
        stl_vector_to_nalg_fixed(mesh.vertices.get(face.vertices[2]).unwrap()),
    ];
    let normal = stl_vector_to_nalg_fixed(&face.normal);
    Triangle { points, normal }
}

fn stl_vector_to_nalg_fixed(vertex: &stl_io::Vector<f32>) -> Vector3<I16F16> {
    Vector3::new(
        vertex[0].to_fixed(),
        vertex[1].to_fixed(),
        vertex[2].to_fixed(),
    )
}

struct Intersection {
    point: Vector2<I16F16>,
    normal: Vector2<I16F16>,
}

fn triangle_plane_intersection(
    triangle: &Triangle,
    plane: &Plane,
    eps: &I16F16,
) -> Vec<Intersection> {
    let mut intersections = Vec::new();
    let Triangle {
        points: [p1, p2, p3],
        normal,
    } = triangle;

    intersections.extend(segment_plane_intersection(p1, p2, normal, plane, eps));
    intersections.extend(segment_plane_intersection(p2, p3, normal, plane, eps));
    intersections.extend(segment_plane_intersection(p3, p1, normal, plane, eps));

    intersections
}

fn segment_plane_intersection(
    p1: &Vector3<I16F16>,
    p2: &Vector3<I16F16>,
    normal: &Vector3<I16F16>,
    plane: &Plane,
    eps: &I16F16,
) -> Vec<Intersection> {
    let d1 = distance_from_plane(p1, plane);
    let d2 = distance_from_plane(p2, plane);

    let p1_on_plane = &d1.abs() < eps;
    let p2_on_plane = &d2.abs() < eps;

    let mut intersection_points = Vec::new();

    if p1_on_plane {
        //println!("p1 on plane: {}", p1);
        intersection_points.push(Intersection {
            point: p1.xy(),
            normal: normal.xy(),
        });
    }
    if p2_on_plane {
        //println!("p2 on plane: {}", p2);
        intersection_points.push(Intersection {
            point: p2.xy(),
            normal: normal.xy(),
        });
    }

    // Points not on plane and they're on opposite sides
    if intersection_points.is_empty() && d1 * d2 < 0. {
        let t = (d1 / (d1 - d2)).to_fixed(); // 'time' of intersection point on the segment
        let intersect_point = p1 + ((p2 - p1) * t);
        let intersect_point = Vector2::new(
            round_to_eps(intersect_point.x, eps),
            round_to_eps(intersect_point.y, eps),
        );
        //println!("intersect point: {}", intersect_point);
        intersection_points.push(Intersection {
            point: intersect_point,
            normal: normal.xy(),
        });
    }

    intersection_points
}

fn round_to_eps(value: I16F16, eps: &I16F16) -> I16F16 {
    (value / eps).round() * eps
}

// TODO memoize for performance
fn distance_from_plane(p: &Vector3<I16F16>, plane: &Plane) -> I16F16 {
    //(       p dot product plane.normal                              )
    p.x * plane.normal.x + p.y * plane.normal.y + p.z * plane.normal.z + plane.distance
}
