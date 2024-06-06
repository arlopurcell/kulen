use std::fs::OpenOptions;

use fixed::traits::ToFixed;
use fixed::types::I16F16;
use nalgebra::Vector3;
use stl_io::{IndexedMesh, IndexedTriangle};

use crate::geometry::Triangle;

pub fn read_stl_file(path: &str) -> std::io::Result<Vec<Triangle>> {
    let mut file = OpenOptions::new().read(true).open(path)?;
    let mut reader = stl_io::create_stl_reader(&mut file)?;
    let indexed_triangles = reader.as_indexed_triangles()?;

    let triangles: Vec<_> = indexed_triangles
        .faces
        .iter()
        .map(|face| deindex_triangle(face, &indexed_triangles))
        .collect();
    Ok(triangles)
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
