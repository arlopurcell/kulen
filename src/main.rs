use std::{collections::HashSet, fs::OpenOptions};
use std::iter::once;

use clap::Parser;
use fixed::traits::{LossyInto, ToFixed};
use fixed::types::I16F16;
use float_ord::FloatOrd;
use image::{GrayImage, Luma};
use imageproc::drawing::draw_hollow_polygon_mut;
use imageproc::{drawing::{draw_filled_rect_mut, draw_polygon_mut}, point::Point, rect::Rect};
use nalgebra::{Vector3, Vector2};
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

type Triangle = (Vector3<f32>, Vector3<f32>, Vector3<f32>);
struct Plane {
    normal: Vector3<f32>,
    distance: f32,
}

fn slice_mesh(mesh: &IndexedMesh) {
    let triangles: Vec<_> = mesh.faces.iter().map(|face| deindex_triangle(face, mesh)).collect();
    //println!("triangles {:?}", triangles);

    let FloatOrd(max_height) = mesh.vertices.iter().map(|vertex| FloatOrd(vertex[2])).max().unwrap();
    let layer_height = 1.0;
    let eps = 0.05;
    let mut slice_plane = Plane {
        normal: Vector3::new(0., 0., -1.),
        distance: 0.,
    };
    for layer in 0..(max_height / layer_height).floor() as u32 {
    //for layer in 0..5 {
        slice_plane.distance = layer as f32 * layer_height;
        let intersection_points: Vec<Vector3<f32>> = triangles.iter()
            .map(|triange| triangle_plane_intersection(triange, &slice_plane, &eps))
            .flatten().collect();
        let fixed_points: HashSet<Vector2<I16F16>> = intersection_points.iter()
            .map(|p| Vector2::new(p.x.to_fixed(), p.y.to_fixed()))
            .collect();
        //println!("layer {}", layer);
        //for point in &fixed_points {
        //    print!("{}", point);
        //}
        let outline_points = polygon_order_points(fixed_points);
        write_layer_image(layer, &outline_points);
    }
}

fn write_layer_image(layer: u32, points: &[Vector2<I16F16>]) {
    let image_size = 600;
    let mut image = GrayImage::new(image_size, image_size);
    // Fill image with white
    for i in 0..image_size {
        for j in 0..image_size {
            image.put_pixel(i, j, image::Luma([255]))
        }
    }
    draw_filled_rect_mut(&mut image, Rect::at(0, 0).of_size(image_size, image_size), image::Luma([255]));

    let image_points: Vec<Point<f32>> = points.iter()
        .map(|p| {
            let x: f32 = (p.x * I16F16::from_num(25)).lossy_into();
            let y: f32 = (p.y * I16F16::from_num(25)).lossy_into();
            Point::new(x + 300., y + 300.)
        }).collect();

    draw_hollow_polygon_mut(&mut image, &image_points, image::Luma([0]));
    image.save(format!("layers/layer_{}.png", layer)).unwrap();
}

fn polygon_order_points(points: HashSet<Vector2<I16F16>>) -> Vec<Vector2<I16F16>> {
    let mut points: Vec<_> = points.into_iter().collect();
    points.sort_by_key(|p| p.x);
    let p = points.remove(0);
    let q = points.pop().unwrap();
    let (a, mut b): (Vec<_>, Vec<_>) = points.into_iter().partition(|x| is_above(&p, &q, x));
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
    (
        vertex_to_vector(mesh.vertices.get(face.vertices[0]).unwrap()),
        vertex_to_vector(mesh.vertices.get(face.vertices[1]).unwrap()),
        vertex_to_vector(mesh.vertices.get(face.vertices[2]).unwrap()),
    )
}

fn vertex_to_vector(vertex: &Vertex) -> Vector3<f32> {
    Vector3::new(vertex[0], vertex[1], vertex[2])
}

fn triangle_plane_intersection(triangle: &Triangle, plane: &Plane, eps: &f32) -> Vec<Vector3<f32>> {
    let mut intersection_points = Vec::new();
    let (p1, p2, p3) = triangle;
    intersection_points.extend(segment_plane_intersection(p1, p2, plane, eps));
    intersection_points.extend(segment_plane_intersection(p2, p3, plane, eps));
    intersection_points.extend(segment_plane_intersection(p3, p1, plane, eps));
    intersection_points

}

fn segment_plane_intersection(p1: &Vector3<f32>, p2: &Vector3<f32>, plane: &Plane, eps: &f32) -> Vec<Vector3<f32>> {
    let d1 = distance_from_plane(p1, plane);
    let d2 = distance_from_plane(p2, plane);

    let p1_on_plane = &d1.abs() < eps;
    let p2_on_plane = &d2.abs() < eps;

    let mut intersection_points = Vec::new();

    if p1_on_plane {
        intersection_points.push(p1.clone());
    }
    if p2_on_plane {
        intersection_points.push(p2.clone());
    }

    // Points not on plane and they're on opposite sides
    if intersection_points.is_empty() && d1 * d2 < 0. {
        let t = d1 / (d1 - d2); // 'time' of intersection point on the segment
        intersection_points.push(p1 + ((p2 - p1) * t));
    }

    intersection_points
}

// TODO memoize for performance
fn distance_from_plane(p: &Vector3<f32>, plane: &Plane) -> f32 {
    p.dot(&plane.normal) + plane.distance
}
