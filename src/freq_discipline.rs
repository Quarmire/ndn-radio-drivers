//! Periodic **frequency** discipline: push the time layer's skew estimate into a radio's clock.
//!
//! `ndn-time` already estimates local frequency skew (`Correction.freq_skew_ppb`, a regression over
//! recent offset points) but its `Discipline` action is deliberately software-only — it moves the
//! *wall estimate* and never touches a counter. That is the right default for a radio with no trim.
//! When a radio DOES advertise [`RadioTime::clock_steering`], the same estimate can be spent on the
//! hardware instead, so a correction holds rather than re-accumulating between fixes.
//!
//! Layering is deliberate: `ndn-time` computes, this actuates. The core crate keeps no dependency
//! on the radio HAL, and a radio without a trim simply never constructs one of these.
//!
//! ## Why this is a loop and not a calibration
//!
//! MEASURED on two RTL8733BUs: the relative rate between two chips at their factory caps read
//! -0.338, -1.095 and -0.838 ppm across three sessions. The offset WANDERS, so a one-shot
//! calibration decays. Re-measure and re-apply.
//!
//! ## Why there is a deadband
//!
//! Also measured: the trim resolves ~0.64 ppm per step while common view resolves 0.0034 ppm — the
//! actuator is ~94x coarser than the sensor. Without a deadband the loop sees a residual it can
//! always measure and never express, and dithers between two adjacent caps forever.
//!
//! The deadband is **one full step**, not half of one. Half-a-step minimises the instantaneous
//! error on paper, but MEASURED on hardware it limit-cycles: with the deadband at 0.32 ppm,
//! residuals of 0.323 / 0.325 / 0.392 ppm each cleared it, and the smallest available correction
//! (0.67 ppm) overshot them and flipped the sign — cap 70 -> 71 -> 70. A residual smaller than one
//! step is not correctable, so the loop must decline to try. This costs up to one step of standing
//! error and buys a clock that stops moving.
use ndn_radio_hal::{ClockSteering, FaceError, RadioTime};
use std::sync::Arc;

/// What a discipline tick did — returned so a caller can log or export it rather than guess.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FreqAction {
    /// Residual is inside the deadband; the hardware cannot express a smaller correction.
    Held { residual_ppm: f32 },
    /// Steered. `applied_ppm` is what the radio reported it actually did, not what was asked.
    Steered {
        requested_ppm: f32,
        applied_ppm: f32,
    },
    /// The correction needed exceeds the radio's advertised range; applied what was possible.
    Saturated { wanted_ppm: f32, applied_ppm: f32 },
}

/// Closed-loop frequency discipline over one radio's clock trim.
pub struct FreqDiscipline {
    radio: Arc<dyn RadioTime>,
    limits: ClockSteering,
    /// Cumulative correction currently commanded, ppm relative to the factory calibration.
    applied_ppm: f32,
    /// Residuals below this are not chased. Defaults to half a trim step.
    deadband_ppm: f32,
}

impl FreqDiscipline {
    /// Build a loop for `radio`, or `None` if it cannot steer its clock — the honest way for a
    /// caller to discover that this radio simply is not disciplinable.
    pub fn new(radio: Arc<dyn RadioTime>) -> Option<Self> {
        let limits = radio.clock_steering()?;
        Some(Self {
            radio,
            limits,
            applied_ppm: 0.0,
            deadband_ppm: limits.resolution_ppm,
        })
    }

    /// Seed the loop with a correction the HARDWARE is already expressing.
    ///
    /// ⚠ Needed because a trim is a hardware register that survives process restarts. A fresh
    /// process that assumed `applied_ppm = 0` while the radio was already steered would compute its
    /// next correction from a false zero — the same defect that shipped in `steer_clock_ppm`'s
    /// first version, where the base latched from "the cap as found" instead of from efuse.
    pub fn with_applied_ppm(mut self, ppm: f32) -> Self {
        self.applied_ppm = ppm;
        self
    }

    /// Override the deadband (ppm). Below ONE trim step the loop will limit-cycle — see the module
    /// docs for the measurement.
    pub fn with_deadband_ppm(mut self, ppm: f32) -> Self {
        self.deadband_ppm = ppm;
        self
    }

    /// Total correction currently commanded, ppm relative to factory.
    pub fn applied_ppm(&self) -> f32 {
        self.applied_ppm
    }

    /// The radio's advertised steering limits.
    pub fn limits(&self) -> ClockSteering {
        self.limits
    }

    /// One tick. `measured_skew_ppm` is how fast THIS clock runs relative to the reference
    /// (positive = we are fast), e.g. from a common-view comparison or `Correction.freq_skew_ppb`.
    ///
    /// Corrects by the *residual*, not the raw measurement: the measurement already includes
    /// whatever this loop applied earlier, so feeding it back undivided would double-count and
    /// overshoot.
    pub fn update(&mut self, measured_skew_ppm: f32) -> Result<FreqAction, FaceError> {
        if measured_skew_ppm.abs() <= self.deadband_ppm {
            return Ok(FreqAction::Held {
                residual_ppm: measured_skew_ppm,
            });
        }
        let wanted = self.applied_ppm - measured_skew_ppm;
        let clamped = wanted.clamp(-self.limits.range_ppm, self.limits.range_ppm);
        let applied = self.radio.steer_clock_ppm(clamped)?;
        self.applied_ppm = applied;
        Ok(if (clamped - wanted).abs() > f32::EPSILON {
            FreqAction::Saturated {
                wanted_ppm: wanted,
                applied_ppm: applied,
            }
        } else {
            FreqAction::Steered {
                requested_ppm: clamped,
                applied_ppm: applied,
            }
        })
    }

    /// Convenience for the `ndn-time` unit: skew in parts-per-BILLION.
    pub fn update_ppb(&mut self, measured_skew_ppb: i64) -> Result<FreqAction, FaceError> {
        self.update(measured_skew_ppb as f32 / 1000.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ndn_radio_hal::{ClockDomainId, RadioTimeSource};
    use std::sync::Mutex;

    /// A radio with a quantised, bounded trim — the properties that actually shape the control law.
    struct FakeRadio {
        step: f32,
        range: f32,
        cap: Mutex<f32>,
    }
    impl RadioTime for FakeRadio {
        fn time_sources(&self) -> Vec<RadioTimeSource> {
            vec![]
        }
        fn clock_steering(&self) -> Option<ClockSteering> {
            Some(ClockSteering {
                range_ppm: self.range,
                resolution_ppm: self.step,
            })
        }
        fn steer_clock_ppm(&self, ppm: f32) -> Result<f32, FaceError> {
            let q = (ppm / self.step).round() * self.step;
            let q = q.clamp(-self.range, self.range);
            *self.cap.lock().unwrap() = q;
            Ok(q)
        }
        fn read_clock(&self, _d: ClockDomainId) -> Result<Option<u64>, FaceError> {
            Ok(None)
        }
    }

    fn loop_for(step: f32, range: f32) -> FreqDiscipline {
        FreqDiscipline::new(Arc::new(FakeRadio {
            step,
            range,
            cap: Mutex::new(0.0),
        }))
        .unwrap()
    }

    #[test]
    fn converges_and_then_holds() {
        let mut d = loop_for(0.64, 9.0);
        // The radio runs 5 ppm fast; simulate re-measuring after each correction.
        let mut truth = 5.0f32;
        for _ in 0..6 {
            let a = d.update(truth).unwrap();
            if let FreqAction::Steered { applied_ppm, .. } = a {
                truth = 5.0 + applied_ppm; // hardware now offsets the real skew
            }
        }
        assert!(truth.abs() <= 0.64, "did not converge: residual {truth}");
        // Once inside the deadband it must STOP, not dither.
        assert!(matches!(d.update(truth).unwrap(), FreqAction::Held { .. }));
    }

    #[test]
    fn does_not_chase_below_one_step() {
        let mut d = loop_for(0.64, 9.0);
        // A residual the hardware cannot express must be held, not acted on.
        assert!(matches!(d.update(0.2).unwrap(), FreqAction::Held { .. }));
        assert_eq!(d.applied_ppm(), 0.0);
    }

    /// Regression for a limit cycle seen ON HARDWARE with a half-step deadband: residuals of
    /// 0.323 / 0.325 / 0.392 ppm each cleared it, and the only available correction (0.67 ppm)
    /// overshot and flipped the sign, toggling the trim between two adjacent caps forever.
    #[test]
    fn does_not_limit_cycle_near_the_threshold() {
        let mut d = loop_for(0.64, 9.0);
        for r in [0.323f32, -0.325, 0.392, -0.35, 0.34] {
            let a = d.update(r).unwrap();
            assert!(
                matches!(a, FreqAction::Held { .. }),
                "acted on an uncorrectable residual {r}: {a:?}"
            );
        }
        assert_eq!(d.applied_ppm(), 0.0, "trim moved while limit-cycling");
    }

    #[test]
    fn reports_saturation_instead_of_pretending() {
        let mut d = loop_for(0.64, 9.0);
        let a = d.update(30.0).unwrap();
        match a {
            FreqAction::Saturated { applied_ppm, .. } => assert!(applied_ppm <= -8.9),
            other => panic!("expected saturation, got {other:?}"),
        }
    }

    #[test]
    fn corrects_the_residual_not_the_raw_measurement() {
        // After a first correction the measurement already contains it; a loop that fed the raw
        // value back would double-count and oscillate.
        let mut d = loop_for(0.64, 9.0);
        d.update(5.0).unwrap();
        let first = d.applied_ppm();
        d.update(0.1_f32.max(0.0)).unwrap(); // residual now tiny -> hold
        assert_eq!(d.applied_ppm(), first);
    }
}
