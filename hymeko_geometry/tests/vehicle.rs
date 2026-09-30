use hymeko_geometry::vehicle::{Bicycle, DriveCommand, VehicleConfig, VehicleState};

#[test]
fn straight_acceleration_and_in_step_stop_obey_closed_solutions() {
    let config = VehicleConfig::default();
    let mut car = Bicycle::new(config, VehicleState::default()).unwrap();
    for _ in 0..240 {
        car.step(
            1. / 240.,
            DriveCommand {
                acceleration: 6.,
                steering: 0.,
            },
        )
        .unwrap();
    }
    assert!((car.state().position[0] - 3.).abs() < 1e-12);
    assert!((car.state().speed - 6.).abs() < 1e-12);
    for _ in 0..137 {
        car.step(
            0.01,
            DriveCommand {
                acceleration: -12.,
                steering: 0.,
            },
        )
        .unwrap();
    }
    assert_eq!(car.state().speed, 0.);
    assert!((car.state().position[0] - 4.5).abs() < 1e-12);
    assert_eq!(car.state().position[1], 0.);
}

#[test]
fn signed_constant_steering_traces_a_circle() {
    for sign in [-1., 1.] {
        let config = VehicleConfig::default();
        let radius = 50.;
        let speed = 12.;
        let steering = sign * (config.wheelbase / radius).atan();
        let mut car = Bicycle::new(
            config,
            VehicleState {
                speed,
                steering,
                ..Default::default()
            },
        )
        .unwrap();
        let period = std::f64::consts::TAU * radius / speed;
        let dt = period / 4000.;
        for i in 1..=4000 {
            car.step(
                dt,
                DriveCommand {
                    acceleration: 0.,
                    steering,
                },
            )
            .unwrap();
            let angle = i as f64 * dt * speed / radius;
            assert!((car.state().position[0] - radius * angle.sin()).abs() < 2e-9);
            assert!((car.state().position[1] - sign * radius * (1. - angle.cos())).abs() < 2e-9);
        }
        assert!((car.state().distance - std::f64::consts::TAU * radius).abs() < 1e-9);
    }
}

#[test]
fn actuator_limits_and_failed_commands_preserve_state() {
    let config = VehicleConfig::default();
    let mut car = Bicycle::new(config, VehicleState::default()).unwrap();
    car.step(
        0.1,
        DriveCommand {
            acceleration: 1e6,
            steering: 1e6,
        },
    )
    .unwrap();
    assert!((car.state().steering - config.steering_rate * 0.1).abs() < 1e-15);
    assert!((car.state().speed - 0.6).abs() < 1e-15);
    let before = *car.state();
    for dt in [f64::NAN, -1., 0.1001] {
        assert!(car.step(dt, DriveCommand::default()).is_err());
        assert_eq!(*car.state(), before);
    }
    assert!(
        car.step(
            0.01,
            DriveCommand {
                acceleration: f64::NAN,
                steering: 0.
            }
        )
        .is_err()
    );
    assert_eq!(*car.state(), before);
    assert!(
        car.reset(VehicleState {
            speed: -1.,
            ..Default::default()
        })
        .is_err()
    );
    assert_eq!(*car.state(), before);
    car.step(0., DriveCommand::default()).unwrap();
    assert_eq!(*car.state(), before);
    car.reset(VehicleState::default()).unwrap();
    assert_eq!(*car.state(), VehicleState::default());
    assert!(
        Bicycle::new(
            VehicleConfig {
                wheelbase: 0.,
                ..config
            },
            VehicleState::default()
        )
        .is_err()
    );
    assert_eq!(car.config().max_speed, 300. / 3.6);
}
