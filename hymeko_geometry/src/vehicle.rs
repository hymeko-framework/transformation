//! Planar, bounded kinematic bicycle motion. Position denotes the rear axle.
//! This model does not claim tyre forces, suspension or aerodynamic dynamics.

use std::{error::Error, fmt};

/// Vehicle geometry and actuator bounds in SI units; angles are radians.
#[derive(Clone, Copy, Debug)]
pub struct VehicleConfig {
    /// Axle separation in metres.
    pub wheelbase: f64,
    /// Maximum front steering magnitude in radians.
    pub max_steering: f64,
    /// Maximum steering speed in rad/s.
    pub steering_rate: f64,
    /// Maximum forward acceleration in m/s².
    pub acceleration: f64,
    /// Maximum braking deceleration in m/s².
    pub braking: f64,
    /// Forward speed ceiling in m/s. Reverse driving is outside this contract.
    pub max_speed: f64,
}
impl Default for VehicleConfig {
    fn default() -> Self {
        Self {
            wheelbase: 3.4,
            max_steering: 0.6,
            steering_rate: 0.8,
            acceleration: 6.0,
            braking: 12.0,
            max_speed: 300.0 / 3.6,
        }
    }
}

/// Observable vehicle state; no reference route is stored in the physical model.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VehicleState {
    /// Rear-axle world position in metres.
    pub position: [f64; 2],
    /// Heading in radians.
    pub yaw: f64,
    /// Nonnegative forward speed in m/s.
    pub speed: f64,
    /// Actual, rate-limited front steering angle in radians.
    pub steering: f64,
    /// Accumulated travelled distance in metres, suitable for wheel animation.
    pub distance: f64,
}

/// Driver demand; actuator limits are applied by the model, not by callers.
#[derive(Clone, Copy, Debug, Default)]
pub struct DriveCommand {
    /// Requested acceleration, m/s².
    pub acceleration: f64,
    /// Requested front steering angle, radians.
    pub steering: f64,
}

/// Invalid configuration, state or step; failed operations are atomic.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VehicleError;
impl fmt::Display for VehicleError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("vehicle requires finite bounded configuration, state and dt in 0..=0.1 s")
    }
}
impl Error for VehicleError {}

/// Stateful bicycle with finite acceleration and steering rate.
#[derive(Clone, Debug)]
pub struct Bicycle {
    config: VehicleConfig,
    state: VehicleState,
}
impl Bicycle {
    /// Construct a vehicle at an independent world pose.
    /// # Preconditions
    /// Positive finite wheelbase, actuator limits and speed ceiling; valid state.
    /// # Postconditions
    /// Returns a bounded model, or an error without changing caller data.
    /// # Errors
    /// Rejects invalid config or state, including speed beyond its configured limit.
    /// # Invariants
    /// No spline, waypoint or rendering state affects the evolution law.
    pub fn new(config: VehicleConfig, state: VehicleState) -> Result<Self, VehicleError> {
        let values = [
            config.wheelbase,
            config.max_steering,
            config.steering_rate,
            config.acceleration,
            config.braking,
            config.max_speed,
        ];
        if values.iter().any(|v| !v.is_finite() || *v <= 0.0)
            || config.wheelbase > 30.0
            || config.max_steering >= 1.4
            || config.max_speed > 200.0
        {
            return Err(VehicleError);
        }
        let model = Self { config, state };
        model.validate_state(state)?;
        Ok(model)
    }
    /// Current state; observation does not advance or modify it.
    pub fn state(&self) -> &VehicleState {
        &self.state
    }
    /// Validated actuator/geometry configuration.
    pub fn config(&self) -> VehicleConfig {
        self.config
    }
    /// Restore a valid state atomically; retains the vehicle configuration.
    /// # Errors
    /// Invalid states leave the current state unchanged.
    pub fn reset(&mut self, state: VehicleState) -> Result<(), VehicleError> {
        self.validate_state(state)?;
        self.state = state;
        Ok(())
    }
    fn validate_state(&self, state: VehicleState) -> Result<(), VehicleError> {
        if state
            .position
            .iter()
            .chain([&state.yaw, &state.speed, &state.steering, &state.distance])
            .any(|v| !v.is_finite())
            || !(0.0..=self.config.max_speed).contains(&state.speed)
            || state.steering.abs() > self.config.max_steering
            || state.distance < 0.0
        {
            return Err(VehicleError);
        }
        Ok(())
    }
    /// Advance with acceleration and steering demand; dt is physical seconds.
    /// # Preconditions
    /// Finite commands and dt in 0..=0.1. Zero dt preserves every state field.
    /// # Postconditions
    /// Steering and speed stay within bounds; braking never creates reverse motion.
    /// Constant steering follows an exact circular arc, including the zero-angle limit.
    /// # Errors
    /// Invalid inputs or nonfinite results leave the entire prior state unchanged.
    /// # Invariants
    /// No route snapping. Distance is the integrated speed, including an in-step stop.
    pub fn step(&mut self, dt: f64, command: DriveCommand) -> Result<(), VehicleError> {
        if !dt.is_finite()
            || !(0.0..=0.1).contains(&dt)
            || !command.acceleration.is_finite()
            || !command.steering.is_finite()
        {
            return Err(VehicleError);
        }
        if dt == 0.0 {
            return Ok(());
        }
        let c = self.config;
        let mut next = self.state;
        let demand = command.steering.clamp(-c.max_steering, c.max_steering);
        next.steering +=
            (demand - next.steering).clamp(-c.steering_rate * dt, c.steering_rate * dt);
        let acceleration = command.acceleration.clamp(-c.braking, c.acceleration);
        let limit = if acceleration > 0.0 { c.max_speed } else { 0.0 };
        let ramp = if acceleration == 0.0 {
            dt
        } else {
            ((limit - next.speed) / acceleration).clamp(0.0, dt)
        };
        let distance = next.speed * ramp
            + 0.5 * acceleration * ramp * ramp
            + (next.speed + acceleration * ramp) * (dt - ramp);
        next.speed = (next.speed + acceleration * dt).clamp(0.0, c.max_speed);
        let angle = distance * ((next.steering + self.state.steering) * 0.5).tan() / c.wheelbase;
        let half = angle * 0.5;
        let chord = distance * if half == 0.0 { 1.0 } else { half.sin() / half };
        next.position[0] += chord * (next.yaw + half).cos();
        next.position[1] += chord * (next.yaw + half).sin();
        next.yaw = (next.yaw + angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
            - std::f64::consts::PI;
        next.distance += distance;
        self.validate_state(next)?;
        self.state = next;
        Ok(())
    }
}
