use std::collections::HashMap;
use std::iter::{self, once};
use std::{collections::HashSet, fs::OpenOptions};

use clap::Parser;
use fixed::traits::{LossyInto, ToFixed};
use fixed::types::I16F16;
use float_ord::FloatOrd;
use image::{GrayImage, Luma};
use imageproc::drawing::draw_hollow_polygon_mut;
use imageproc::{
    drawing::{draw_filled_rect_mut, draw_polygon_mut},
    point::Point,
    rect::Rect,
};
use nalgebra::{Vector2, Vector3};
use stl_io::{IndexedMesh, IndexedTriangle, Vertex};

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
    input: String,
    output: String,
}

fn main() {
    let cli = Args::parse();

    match read_stl_file(&cli.input) {
        Ok(mesh) => slice_mesh(&mesh),
        Err(e) => println!("Failed to read input file: {:?}", e),
    }
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
    // TODO temperatures
}

fn slice_mesh(mesh: &IndexedMesh) {
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
    let layer_height: I16F16 = 1.to_fixed();
    let eps = 0.05.to_fixed();
    let mut slice_plane = Plane {
        normal: Vector3::new(0.to_fixed(), 0.to_fixed(), (-1).to_fixed()),
        distance: 0.to_fixed(),
    };
    let layers: u32 = (max_height / layer_height).to_num();
    for layer in 0..layers {
        //for layer in 2..3 {
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
        write_layer_image(layer, &outline_points);
    }
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

fn write_layer_image(layer: u32, points: &[Intersection]) {
    let image_size = 600;
    let mut image = GrayImage::new(image_size, image_size);
    // Fill image with white
    for i in 0..image_size {
        for j in 0..image_size {
            image.put_pixel(i, j, image::Luma([255]))
        }
    }
    draw_filled_rect_mut(
        &mut image,
        Rect::at(0, 0).of_size(image_size, image_size),
        image::Luma([255]),
    );

    let image_points: Vec<Point<f32>> = points
        .iter()
        .map(|i| {
            let x: f32 = (i.point.x * I16F16::from_num(25)).lossy_into();
            let y: f32 = (i.point.y * I16F16::from_num(25)).lossy_into();
            Point::new(x + 300., y + 300.)
        })
        .collect();

    draw_hollow_polygon_mut(&mut image, &image_points, image::Luma([0]));
    image.save(format!("layers/layer_{}.png", layer)).unwrap();
}

/*
fn layer_gcode(outline_points: &[Vector2[I16F16]]) -> String {
    let mut outline_segments = Vec::new();
    for i in 0..outline_points.len() {
        let j = i + 1 % outline_points.len();
        outline_segments
    }

}
*/

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
