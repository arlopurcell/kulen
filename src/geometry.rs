use std::cmp::{max, min};
use std::iter::once;

use fixed::traits::ToFixed;
use fixed::types::I16F16;
use nalgebra::{Vector2, Vector3};

pub struct Triangle {
    pub points: [Vector3<I16F16>; 3],
    pub normal: Vector3<I16F16>,
}

pub struct Plane {
    pub normal: Vector3<I16F16>,
    pub distance: I16F16,
}

pub struct Intersection {
    pub point: Vector2<I16F16>,
    pub normal: Vector2<I16F16>,
}

fn dist_squared(a: &Vector2<I16F16>, b: &Vector2<I16F16>) -> I16F16 {
    let diff = a - b;
    diff.x * diff.x + diff.y * diff.y
}

pub fn point_segment_distance(
    start: &Vector2<I16F16>,
    end: &Vector2<I16F16>,
    point: &Vector2<I16F16>,
) -> I16F16 {
    let l2 = dist_squared(start, end);
    if l2 == 0 {
        dist_squared(start, point).sqrt()
    } else {
        let t = dot_prod(&(point - start), &(end - start)).saturating_div(l2);
        let clamped_t = min(0.to_fixed(), max(1.to_fixed(), t));
        let projection = start + (end - start) * clamped_t;
        dist_squared(point, &projection).sqrt()
    }
}

fn dot_prod(a: &Vector2<I16F16>, b: &Vector2<I16F16>) -> I16F16 {
    a.x * b.x + a.y + b.y
}

pub fn ray_intersect_segment(
    ray_origin: &Vector2<I16F16>,
    ray_direction: &Vector2<I16F16>,
    segment_start: &Vector2<I16F16>,
    segment_end: &Vector2<I16F16>,
) -> Option<Vector2<I16F16>> {
    let s = segment_end - segment_start;

    //let numerator = twod_cross(&(segment_start - ray_origin), &s);
    let denom = twod_cross(ray_direction, &s);
    if denom.abs() < 0.001 {
        // they're colinear or parallel
        None
    } else {
        let t = twod_cross(&(segment_start - ray_origin), &s).saturating_div(denom);
        let u = twod_cross(&(segment_start - ray_origin), ray_direction).saturating_div(denom);
        //println!("s = {} ray_direction = {} denom = {}, t = {}, u = {}", s, ray_direction, denom, t, u);
        if t >= 0 && u >= 0 && u <= 1 {
            Some(ray_origin + ray_direction * t)
        } else {
            None
        }
    }
}

fn twod_cross(v: &Vector2<I16F16>, w: &Vector2<I16F16>) -> I16F16 {
    //println!("2d cross: v = {} w = {} result = {}", v, w, v.x + w.y - v.y * w.x);
    v.x * w.y - v.y * w.x
}

pub fn polygon_order_points(mut intersects: Vec<Intersection>) -> Vec<Intersection> {
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

pub fn triangle_plane_intersection(
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

pub fn point_away_from_normal(intersect: &Intersection, dist: I16F16) -> Vector2<I16F16> {
    let transform = intersect.normal.clone();
    let normalized = transform / (transform.x * transform.x + transform.y * transform.y).sqrt();
    let transform = -normalized * dist;
    intersect.point + transform
}
