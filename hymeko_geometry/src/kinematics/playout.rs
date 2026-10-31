//! A sampled joint trajectory shown at display rate: a bounded ring of controller-stamped joint
//! samples (positions and the controller's own velocities), read a fixed delay behind the newest
//! with cubic Hermite interpolation, and past the newest sample extrapolated with its velocity
//! decaying to rest, so a stream that pauses settles instead of flying on.
//!
//! The controller's clock and the display's are mapped by the smallest transport lag the ring has
//! seen (receive time minus controller stamp): jitter only ever adds lag, so the minimum is the
//! least-delayed delivery. Positions come out per joint; forward kinematics stays the caller's
//! ([`super::dh::standard_dh_chain`]), so a drawn arm remains one rigid chain. The interpolation
//! itself is `hymeko_spline`'s cubic Hermite.
use hymeko_spline::hermite::interpolate_into;

/// Why a [`JointPlayout`] could not be made.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlayoutError {
    /// Joints per sample outside `1..=64`.
    Joints,
    /// Ring capacity outside `2..=4096` samples.
    Capacity,
    /// A delay that is negative or not finite, or a settling time that is not positive and finite.
    Timing,
}

impl std::fmt::Display for PlayoutError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Joints => "a joint playout carries 1..=64 joints",
            Self::Capacity => "a joint playout holds 2..=4096 samples",
            Self::Timing => {
                "playout delay must be finite and non-negative, settling time finite and positive"
            }
        })
    }
}

impl std::error::Error for PlayoutError {}

/// How [`JointPlayout::sample`] produced its positions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Playout {
    /// No sample yet, or an output of the wrong length (left untouched).
    Empty,
    /// Between two samples (cubic Hermite on positions and velocities).
    Interpolated,
    /// Past the newest sample by `ahead_s` controller seconds, its velocity decaying to rest.
    Extrapolated { ahead_s: f64 },
    /// Before the oldest sample the ring still holds: its positions.
    Held,
}

/// A ring of joint samples played out a fixed delay behind the controller.
#[derive(Debug, Clone)]
pub struct JointPlayout {
    joints: usize,
    capacity: usize,
    delay_s: f64,
    settle_s: f64,
    /// `capacity` rows `[controller_s, received_s, q.., qd..]`, contiguous; the oldest at `head`.
    rows: Vec<f64>,
    head: usize,
    len: usize,
}

impl JointPlayout {
    /// `capacity` samples of `joints` joints, read `delay_s` behind the newest; past it, velocities
    /// decay with time constant `settle_s`, so an extrapolation moves at most `|qd| settle_s`.
    pub fn new(
        joints: usize,
        capacity: usize,
        delay_s: f64,
        settle_s: f64,
    ) -> Result<Self, PlayoutError> {
        if !(1..=64).contains(&joints) {
            return Err(PlayoutError::Joints);
        }
        if !(2..=4096).contains(&capacity) {
            return Err(PlayoutError::Capacity);
        }
        if !delay_s.is_finite() || delay_s < 0. || !settle_s.is_finite() || settle_s <= 0. {
            return Err(PlayoutError::Timing);
        }
        Ok(Self {
            joints,
            capacity,
            delay_s,
            settle_s,
            rows: vec![0.; capacity * (2 + 2 * joints)],
            head: 0,
            len: 0,
        })
    }

    /// Samples held.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether no sample is held.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Forget every sample (a new stream, or a clock that went back).
    pub fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }

    /// Add the sample the controller stamped `controller_s`, received at `received_s` on the display's
    /// clock. A full ring drops its oldest. Refused (false): wrong lengths, non-finite values, or a
    /// stamp not newer than the newest held (a duplicate or a reordered delivery).
    pub fn push(&mut self, controller_s: f64, received_s: f64, q: &[f64], qd: &[f64]) -> bool {
        if q.len() != self.joints || qd.len() != self.joints {
            return false;
        }
        if ![controller_s, received_s]
            .iter()
            .chain(q)
            .chain(qd)
            .all(|v| v.is_finite())
        {
            return false;
        }
        if self.len > 0 && controller_s <= self.row(self.len - 1)[0] {
            return false;
        }
        let at = (self.head + self.len) % self.capacity;
        if self.len == self.capacity {
            self.head = (self.head + 1) % self.capacity;
        } else {
            self.len += 1;
        }
        let width = self.width();
        let row = &mut self.rows[at * width..(at + 1) * width];
        row[0] = controller_s;
        row[1] = received_s;
        row[2..2 + self.joints].copy_from_slice(q);
        row[2 + self.joints..].copy_from_slice(qd);
        true
    }

    /// The controller time shown at display time `now_s`: `now_s` less the smallest lag held and
    /// the delay.
    pub fn controller_time(&self, now_s: f64) -> Option<f64> {
        let lag = (0..self.len)
            .map(|i| self.row(i))
            .map(|r| r[1] - r[0])
            .fold(f64::INFINITY, f64::min);
        (self.len > 0 && now_s.is_finite()).then(|| now_s - lag - self.delay_s)
    }

    /// Positions at display time `now_s` into `out` (one per joint).
    pub fn sample(&self, now_s: f64, out: &mut [f64]) -> Playout {
        let Some(t) = self
            .controller_time(now_s)
            .filter(|_| out.len() == self.joints)
        else {
            return Playout::Empty;
        };
        let n = self.joints;
        let oldest = self.row(0);
        if t <= oldest[0] {
            out.copy_from_slice(&oldest[2..2 + n]);
            return Playout::Held;
        }
        let newest = self.row(self.len - 1);
        if t >= newest[0] {
            let ahead_s = t - newest[0];
            let reach = self.settle_s * (1. - (-ahead_s / self.settle_s).exp());
            for (j, value) in out.iter_mut().enumerate() {
                *value = newest[2 + j] + newest[2 + n + j] * reach;
            }
            return Playout::Extrapolated { ahead_s };
        }
        // The bracketing pair, searched from the newest: playout reads near the ring's end.
        let mut i = self.len - 2;
        while self.row(i)[0] > t {
            i -= 1;
        }
        let (a, b) = (self.row(i), self.row(i + 1));
        // Held samples have equal lengths and increasing finite stamps: the segment is well formed.
        let read = interpolate_into(
            (a[0], &a[2..2 + n], &a[2 + n..]),
            (b[0], &b[2..2 + n], &b[2 + n..]),
            t,
            out,
        );
        debug_assert!(read.is_ok());
        Playout::Interpolated
    }

    fn width(&self) -> usize {
        2 + 2 * self.joints
    }

    /// The `i`-th held sample, oldest first.
    fn row(&self, i: usize) -> &[f64] {
        let at = (self.head + i) % self.capacity;
        let width = self.width();
        &self.rows[at * width..(at + 1) * width]
    }
}
