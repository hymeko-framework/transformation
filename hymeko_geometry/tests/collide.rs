//! Collision detection.
//!
//! GJK is one of those algorithms that is easy to write into a state where it works on the cases you
//! tried. So nothing here trusts it:
//!
//! * four shape pairs have **closed-form** distances — sphere/sphere, sphere/capsule,
//!   capsule/capsule, sphere/box — and those are computed independently and compared;
//! * for the pairs with no closed form, a **separating direction is a certificate**: if a plane can
//!   be found with all of `A` on one side and all of `B` on the other, then GJK reporting contact is
//!   provably wrong, and the plane says so without appeal;
//! * and every pair is swept through its own boundary, where the answer must switch exactly once.

use hymeko_clifford::Se3;
use hymeko_geometry::collide::{intersects_at as intersects, Collider, Convex};

#[test]
fn declared_full_extents_match_the_mjcf_geometry_convention() {
    use hymeko_geometry::kinematics::kinematic::GeometryShape;
    for scale in [0.001, 0.1, 1.0, 10.0] {
        let box_shape = Convex::from_declaration(
            GeometryShape::Box, &[2.0 * scale, 4.0 * scale, 6.0 * scale],
        ).unwrap();
        let cylinder = Convex::from_declaration(GeometryShape::Cylinder, &[scale, 6.0 * scale]).unwrap();
        assert_eq!(
            box_shape.support([1.0, 1.0, 1.0]), [scale, 2.0 * scale, 3.0 * scale],
        );
        assert_eq!(cylinder.support([0.0, 0.0, 1.0])[2], 3.0 * scale);
        assert_eq!(cylinder.support([1.0, 0.0, 0.0])[0], scale);
    }
}

// --- reference geometry, computed without reference to the implementation ------------------------

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

/// Distance from a point to a segment `[p, q]`.
fn point_segment(x: [f64; 3], p: [f64; 3], q: [f64; 3]) -> f64 {
    let pq = sub(q, p);
    let l2 = dot(pq, pq);
    let t = if l2 < 1e-300 {
        0.0
    } else {
        (dot(sub(x, p), pq) / l2).clamp(0.0, 1.0)
    };
    norm(sub(x, [p[0] + t * pq[0], p[1] + t * pq[1], p[2] + t * pq[2]]))
}

/// Distance between two segments, by dense sampling of one against the other.
///
/// A closed form exists and is notoriously fiddly at the parallel case. Sampling is slower and
/// obviously right, which is the trade a reference should make; the resolution is chosen so its own
/// error is far below the tolerances the assertions use.
fn segment_segment(p1: [f64; 3], q1: [f64; 3], p2: [f64; 3], q2: [f64; 3]) -> f64 {
    let n = 4000;
    let mut best = f64::INFINITY;
    for i in 0..=n {
        let t = i as f64 / n as f64;
        let x = [
            p1[0] + t * (q1[0] - p1[0]),
            p1[1] + t * (q1[1] - p1[1]),
            p1[2] + t * (q1[2] - p1[2]),
        ];
        best = best.min(point_segment(x, p2, q2));
    }
    best
}

/// Distance from a point to a box, in the box's own frame.
fn point_box(x: [f64; 3], half: [f64; 3]) -> f64 {
    let mut acc = 0.0;
    for k in 0..3 {
        let over = x[k].abs() - half[k];
        if over > 0.0 {
            acc += over * over;
        }
    }
    acc.sqrt()
}

/// The endpoints of a capsule's or cylinder's axis, in world coordinates.
fn axis_ends(pose: &Se3, offset: [f64; 3], half_height: f64) -> ([f64; 3], [f64; 3]) {
    let a = pose.apply(&[offset[0], offset[1], offset[2] - half_height]);
    let b = pose.apply(&[offset[0], offset[1], offset[2] + half_height]);
    ([a[0], a[1], a[2]], [b[0], b[1], b[2]])
}

// --- fixtures ------------------------------------------------------------------------------------

struct Rng(u64);
impl Rng {
    fn f64(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn signed(&mut self) -> f64 {
        2.0 * self.f64() - 1.0
    }
    fn vec3(&mut self) -> [f64; 3] {
        [self.signed(), self.signed(), self.signed()]
    }
    fn pose(&mut self) -> Se3 {
        Se3::rpy_xyz(
            [
                self.signed() * 3.2,
                self.signed() * 3.2,
                self.signed() * 3.2,
            ],
            self.vec3(),
        )
    }
    fn shape(&mut self) -> Convex {
        match (self.f64() * 4.0) as u32 {
            0 => Convex::Sphere {
                radius: 0.1 + 0.3 * self.f64(),
            },
            1 => Convex::Cuboid {
                half: [
                    0.1 + 0.3 * self.f64(),
                    0.1 + 0.3 * self.f64(),
                    0.1 + 0.3 * self.f64(),
                ],
            },
            2 => Convex::Cylinder {
                radius: 0.1 + 0.3 * self.f64(),
                half_height: 0.1 + 0.3 * self.f64(),
            },
            _ => Convex::Capsule {
                radius: 0.1 + 0.3 * self.f64(),
                half_height: 0.1 + 0.3 * self.f64(),
            },
        }
    }
}

fn collider(shape: Convex) -> Collider {
    Collider {
        link: 0,
        shape,
        offset: [0.0; 3],
    }
}

fn at(t: [f64; 3]) -> Se3 {
    Se3::translation(&t)
}

// --- the closed forms ----------------------------------------------------------------------------

#[test]
fn two_spheres_touch_exactly_when_their_radii_reach() {
    let mut rng = Rng(1);
    for case in 0..400 {
        let (r1, r2) = (0.1 + rng.f64(), 0.1 + rng.f64());
        let a = collider(Convex::Sphere { radius: r1 });
        let b = collider(Convex::Sphere { radius: r2 });
        let xa = at(rng.vec3());
        let d = rng.vec3();
        let n = norm(d).max(1e-9);
        // place the second centre at a chosen distance along a random direction
        for gap in [-0.4, -0.05, -1e-3, 1e-3, 0.05, 0.4] {
            let reach = r1 + r2 + gap;
            let centre = [
                xa.translation[0] + d[0] / n * reach,
                xa.translation[1] + d[1] / n * reach,
                xa.translation[2] + d[2] / n * reach,
            ];
            let got = intersects(&a, &xa, &b, &at(centre));
            assert_eq!(
                got,
                gap < 0.0,
                "case {case}: r={r1:.3}+{r2:.3}, gap {gap}: reported {got}"
            );
        }
    }
}

#[test]
fn a_sphere_and_a_capsule_agree_with_the_point_to_segment_distance() {
    let mut rng = Rng(22);
    let mut checked = 0;
    for case in 0..600 {
        let sr = 0.1 + 0.3 * rng.f64();
        let cr = 0.1 + 0.3 * rng.f64();
        let ch = 0.1 + 0.4 * rng.f64();
        let a = collider(Convex::Sphere { radius: sr });
        let b = collider(Convex::Capsule {
            radius: cr,
            half_height: ch,
        });
        let xa = at([rng.signed(), rng.signed(), rng.signed()]);
        let xb = rng.pose();

        let (p, q) = axis_ends(&xb, [0.0; 3], ch);
        let centre = [xa.translation[0], xa.translation[1], xa.translation[2]];
        let separation = point_segment(centre, p, q) - sr - cr;
        if separation.abs() < 1e-6 {
            continue; // the boundary itself is checked by the sweep below
        }
        assert_eq!(
            intersects(&a, &xa, &b, &xb),
            separation < 0.0,
            "case {case}: separation {separation:e}"
        );
        checked += 1;
    }
    assert!(checked > 400, "only {checked} cases were conclusive");
}

#[test]
fn two_capsules_agree_with_the_segment_to_segment_distance() {
    let mut rng = Rng(333);
    let mut checked = 0;
    for case in 0..300 {
        let (r1, h1) = (0.05 + 0.2 * rng.f64(), 0.1 + 0.4 * rng.f64());
        let (r2, h2) = (0.05 + 0.2 * rng.f64(), 0.1 + 0.4 * rng.f64());
        let a = collider(Convex::Capsule {
            radius: r1,
            half_height: h1,
        });
        let b = collider(Convex::Capsule {
            radius: r2,
            half_height: h2,
        });
        let xa = rng.pose();
        let xb = rng.pose();

        let (p1, q1) = axis_ends(&xa, [0.0; 3], h1);
        let (p2, q2) = axis_ends(&xb, [0.0; 3], h2);
        let separation = segment_segment(p1, q1, p2, q2) - r1 - r2;
        if separation.abs() < 1e-3 {
            continue; // the sampled reference is only good to about this
        }
        assert_eq!(
            intersects(&a, &xa, &b, &xb),
            separation < 0.0,
            "case {case}: separation {separation:e}"
        );
        checked += 1;
    }
    assert!(checked > 150, "only {checked} cases were conclusive");
}

#[test]
fn a_sphere_and_a_box_agree_with_the_point_to_box_distance() {
    let mut rng = Rng(4444);
    let mut checked = 0;
    for case in 0..600 {
        let sr = 0.1 + 0.3 * rng.f64();
        let half = [
            0.1 + 0.3 * rng.f64(),
            0.1 + 0.3 * rng.f64(),
            0.1 + 0.3 * rng.f64(),
        ];
        let a = collider(Convex::Sphere { radius: sr });
        let b = collider(Convex::Cuboid { half });
        let xa = at([rng.signed(), rng.signed(), rng.signed()]);
        let xb = rng.pose();

        // the sphere's centre in the box's frame, where the distance is elementary
        let local = xb.inverse().apply(&[
            xa.translation[0],
            xa.translation[1],
            xa.translation[2],
        ]);
        let separation = point_box([local[0], local[1], local[2]], half) - sr;
        if separation.abs() < 1e-6 {
            continue;
        }
        assert_eq!(
            intersects(&a, &xa, &b, &xb),
            separation < 0.0,
            "case {case}: separation {separation:e}"
        );
        checked += 1;
    }
    assert!(checked > 400, "only {checked} cases were conclusive");
}

// --- the certificate -----------------------------------------------------------------------------

/// The largest gap any sampled plane opens between the two, as a signed width.
///
/// For a direction `d`, `a` reaches to `max_a(p . d)` and `b` starts at `min_b(p . d)`; the plane
/// separates them exactly when the second exceeds the first. So the returned value is
/// `max over d of (min_b - max_a)`: **positive means a plane provably separates them**, and the
/// plane is the proof — every point of `a` is on one side and every point of `b` on the other, said
/// by the support functions and not by any algorithm under test.
///
/// A non-positive value proves nothing, because a finite sample can always miss the separating
/// direction. So this is used in ONE direction only: to contradict a reported contact.
fn widest_separating_gap(a: &Collider, xa: &Se3, b: &Collider, xb: &Se3) -> f64 {
    let mut widest = f64::NEG_INFINITY;
    let n = 60;
    for i in 0..n {
        // a Fibonacci sphere, so the directions are spread rather than clustered at the poles
        let z = 1.0 - 2.0 * (i as f64 + 0.5) / n as f64;
        let r = (1.0 - z * z).max(0.0).sqrt();
        let theta = std::f64::consts::PI * (1.0 + 5f64.sqrt()) * i as f64;
        let d = [r * theta.cos(), r * theta.sin(), z];
        // a reaches to `dot(pa, d)`; b begins at `dot(pb, d)`
        let pa = hymeko_geometry::collide::Placement::of(a, xa).support(&a.shape, d);
        let pb = hymeko_geometry::collide::Placement::of(b, xb)
            .support(&b.shape, [-d[0], -d[1], -d[2]]);
        widest = widest.max(dot(pb, d) - dot(pa, d));
    }
    widest
}

#[test]
fn a_reported_contact_is_never_contradicted_by_a_separating_plane() {
    // The one-sided check that needs no closed form, over every shape pair including the two with
    // none: box/box, box/cylinder, cylinder/cylinder. If a plane separates them, contact is wrong,
    // and the plane is the proof.
    let mut rng = Rng(0xC0111DE);
    let mut contacts = 0;
    for case in 0..4000 {
        let a = collider(rng.shape());
        let b = collider(rng.shape());
        // close enough that both answers occur often
        let xa = Se3::translation(&[0.0, 0.0, 0.0]);
        let mut xb = rng.pose();
        for k in 0..3 {
            xb.translation[k] *= 0.7;
        }

        if intersects(&a, &xa, &b, &xb) {
            contacts += 1;
            let gap = widest_separating_gap(&a, &xa, &b, &xb);
            assert!(
                gap <= 1e-9,
                "case {case}: contact reported, but a plane separates them by {gap:e}"
            );
        }
    }
    assert!(
        contacts > 500,
        "only {contacts} contacts occurred; the fixture is not exercising the contact path"
    );
}

// --- the boundary --------------------------------------------------------------------------------

#[test]
fn sliding_two_shapes_apart_switches_the_answer_exactly_once() {
    // Any pair, driven along a line from deep overlap to clearly apart. A GJK that terminates badly
    // in the near-touching band shows up here as a second transition; one that is simply wrong shows
    // up as none.
    let mut rng = Rng(97);
    for case in 0..300 {
        let a = collider(rng.shape());
        let b = collider(rng.shape());
        let xa = Se3::identity();
        let rot = rng.pose();
        let dir = {
            let d = rng.vec3();
            let n = norm(d).max(1e-9);
            [d[0] / n, d[1] / n, d[2] / n]
        };

        let mut transitions = 0;
        let mut previous: Option<bool> = None;
        let steps = 400;
        for i in 0..=steps {
            let t = 2.0 * i as f64 / steps as f64; // 0 .. 2 metres
            let mut xb = rot;
            xb.translation = [dir[0] * t, dir[1] * t, dir[2] * t];
            let hit = intersects(&a, &xa, &b, &xb);
            if previous.is_some_and(|p| p != hit) {
                transitions += 1;
            }
            previous = Some(hit);
        }
        assert_eq!(
            transitions, 1,
            "case {case}: the answer changed {transitions} times along the sweep"
        );
    }
}

#[test]
fn a_shape_always_collides_with_itself_at_the_same_pose() {
    let mut rng = Rng(5);
    for case in 0..200 {
        let s = rng.shape();
        let x = rng.pose();
        assert!(
            intersects(&collider(s), &x, &collider(s), &x),
            "case {case}: {s:?} does not overlap itself"
        );
    }
}
