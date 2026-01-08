//! Simple NLMS-based acoustic echo canceller.

const DEFAULT_EPSILON: f32 = 1e-3;

/// Adaptive filter implementing a Normalized Least Mean Squares echo canceller.
pub struct NlmsCanceller {
    taps: Vec<f32>,
    history: Vec<f32>,
    history_pos: usize,
    energy: f32,
    mu: f32,
    epsilon: f32,
}

impl NlmsCanceller {
    /// Creates a new canceller that tracks `tap_len` samples of the render path.
    pub fn new(tap_len: usize, mu: f32) -> Self {
        assert!(tap_len > 0, "tap_len must be positive");
        Self {
            taps: vec![0.0; tap_len],
            history: vec![0.0; tap_len],
            history_pos: 0,
            energy: 1e-6,
            mu,
            epsilon: DEFAULT_EPSILON,
        }
    }

    /// Processes a capture block using the provided render block, writing the
    /// residual echo-reduced samples into `output`.
    ///
    /// Each slice must share the same length. Internally we iterate sample by
    /// sample to update the adaptive filter.
    pub fn process_block(
        &mut self,
        render: &[i16],
        capture: &[i16],
        output: &mut [i16],
        adapt: bool,
    ) {
        assert_eq!(
            render.len(),
            capture.len(),
            "render and capture chunks must match"
        );
        assert_eq!(
            capture.len(),
            output.len(),
            "output buffer length must match capture chunk"
        );

        let limit_min = i16::MIN as f32;
        let limit_max = i16::MAX as f32;

        for idx in 0..render.len() {
            let new_sample = render[idx] as f32;
            let old_sample = self.history[self.history_pos];

            self.history[self.history_pos] = new_sample;
            self.energy += new_sample * new_sample - old_sample * old_sample;
            if self.energy < self.epsilon {
                self.energy = self.epsilon;
            }

            self.history_pos = (self.history_pos + 1) % self.history.len();

            let estimate = self.estimate_echo();
            let error = capture[idx] as f32 - estimate;
            output[idx] = error.clamp(limit_min, limit_max) as i16;

            if adapt {
                self.update_taps(error);
            }
        }
    }

    fn estimate_echo(&self) -> f32 {
        let len = self.history.len();
        let mut idx = self.history_pos;
        let mut acc = 0.0;
        for weight in &self.taps {
            idx = dec_idx(len, idx);
            acc += weight * self.history[idx];
        }
        acc
    }

    fn update_taps(&mut self, error: f32) {
        let norm = self.energy + self.epsilon;
        let scale = self.mu * error / norm;

        let len = self.history.len();
        let mut idx = self.history_pos;
        for weight in &mut self.taps {
            idx = dec_idx(len, idx);
            *weight += scale * self.history[idx];
        }
    }
}

fn dec_idx(len: usize, idx: usize) -> usize {
    if idx == 0 {
        len - 1
    } else {
        idx - 1
    }
}
