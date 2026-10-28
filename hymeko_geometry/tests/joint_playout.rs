use hymeko_geometry::kinematics::playout::{JointPlayout, Playout, PlayoutError};

/// Two joints moving on cubics, with their exact velocities.
fn joints(t: f64) -> ([f64; 2], [f64; 2]) {
    (
        [0.2 + 0.5 * t - 0.3 * t * t * t, -1. + 0.1 * t * t],
        [0.5 - 0.9 * t * t, 0.2 * t],
    )
}

/// Samples every 20 ms of controller time from 10 s, received after `lag` seconds plus `jitter(i)`.
fn stream(count: usize, lag: f64, jitter: impl Fn(usize) -> f64) -> JointPlayout {
    let mut playout = JointPlayout::new(2, 64, 0.04, 0.1).unwrap();
    for i in 0..count {
        let t = 10. + 0.02 * i as f64;
        let (q, qd) = joints(t);
        assert!(playout.push(t, 1000. + t + lag + jitter(i), &q, &qd));
    }
    playout
}

#[test]
fn between_samples_the_trajectory_is_read_a_fixed_delay_behind_on_the_smallest_lag() {
    // Jitter only adds lag: the clock mapping takes the least-delayed delivery (5 ms).
    let playout = stream(20, 0.005, |i| {
        if i % 3 == 0 {
            0.
        } else {
            0.004 * (i % 5) as f64
        }
    });
    for now in [1010.25, 1010.3017, 1010.333] {
        let t = playout.controller_time(now).unwrap();
        assert!((t - (now - 1000. - 0.005 - 0.04)).abs() < 1e-9);
        let mut q = [0.; 2];
        assert_eq!(playout.sample(now, &mut q), Playout::Interpolated);
        for (got, want) in q.iter().zip(joints(t).0) {
            assert!((got - want).abs() < 1e-9, "at {t}: {got} != {want}");
        }
    }
}

#[test]
fn past_the_newest_sample_the_motion_settles_instead_of_flying_on() {
    let playout = stream(10, 0.005, |_| 0.);
    let newest = 10. + 0.02 * 9.;
    let (q, qd) = joints(newest);
    let at = |ahead: f64| {
        let mut out = [0.; 2];
        let now = 1000. + newest + 0.005 + 0.04 + ahead;
        (playout.sample(now, &mut out), out)
    };
    let (kind, early) = at(1e-6);
    assert!(matches!(kind, Playout::Extrapolated { ahead_s } if (ahead_s - 1e-6).abs() < 1e-9));
    // Leaving the newest sample with its own velocity...
    for j in 0..2 {
        assert!(((early[j] - q[j]) / 1e-6 - qd[j]).abs() < 1e-4 * qd[j].abs().max(1.));
    }
    // ... and never further than |qd| times the settling time.
    let (_, late) = at(10.);
    for j in 0..2 {
        assert!((late[j] - q[j]).abs() <= qd[j].abs() * 0.1 + 1e-12);
        assert!(
            (late[j] - (q[j] + qd[j] * 0.1)).abs() < 1e-6,
            "settled at the asymptote: {} vs {} + {} * 0.1",
            late[j],
            q[j],
            qd[j]
        );
    }
}

#[test]
fn the_ring_keeps_its_newest_samples_and_refuses_stale_or_malformed_ones() {
    let mut playout = JointPlayout::new(2, 4, 0., 0.1).unwrap();
    for i in 0..6 {
        let t = i as f64;
        assert!(playout.push(t, t, &[t, 0.], &[0., 0.]));
    }
    assert_eq!(playout.len(), 4);
    let mut q = [0.; 2];
    // Before the oldest held sample (t = 2): its positions.
    assert_eq!(playout.sample(1.5, &mut q), Playout::Held);
    assert_eq!(q, [2., 0.]);
    assert!(
        !playout.push(5., 5., &[0., 0.], &[0., 0.]),
        "a duplicate stamp"
    );
    assert!(
        !playout.push(4.5, 6., &[0., 0.], &[0., 0.]),
        "a reordered delivery"
    );
    assert!(!playout.push(6., 6., &[f64::NAN, 0.], &[0., 0.]));
    assert!(!playout.push(6., 6., &[0.], &[0., 0.]));
    assert_eq!(
        playout.sample(3., &mut [0.; 3]),
        Playout::Empty,
        "wrong output length"
    );
    playout.clear();
    assert!(playout.is_empty());
    assert_eq!(playout.sample(3., &mut q), Playout::Empty);
    assert_eq!(
        JointPlayout::new(0, 4, 0., 0.1).unwrap_err(),
        PlayoutError::Joints
    );
    assert_eq!(
        JointPlayout::new(2, 1, 0., 0.1).unwrap_err(),
        PlayoutError::Capacity
    );
    assert_eq!(
        JointPlayout::new(2, 4, -0.1, 0.1).unwrap_err(),
        PlayoutError::Timing
    );
    assert_eq!(
        JointPlayout::new(2, 4, 0., 0.).unwrap_err(),
        PlayoutError::Timing
    );
}
