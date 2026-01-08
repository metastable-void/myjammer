use std::cmp::Ordering;
use std::f32::consts::{PI, SQRT_2};

use alsa::nix::errno::Errno;
use alsa::pcm::{Access, Format, Frames, HwParams, IO, PCM};
use alsa::{Direction, ValueOr};
use anyhow::{Context, Result};

const SAMPLE_RATE: u32 = 48_000;
const CHUNK_SIZE: usize = 4096;
const MIN_FREQ: f32 = 60.0;
const MAX_FREQ: f32 = 1000.0;
const MAX_OUTPUT_GAIN: f32 = 1.0;
const GAIN_SMOOTHING: f32 = 0.15;
const MIN_DETECTION_LEVEL: f32 = 0.01;
const MAX_VOICES: usize = 3;
const MIN_CORRELATION: f32 = 0.35;
const HOLD_FRAMES: usize = 6;

fn main() -> Result<()> {
    run()
}

fn run() -> Result<()> {
    let capture = open_pcm(Direction::Capture).context("failed to open capture PCM")?;
    let playback = open_pcm(Direction::Playback).context("failed to open playback PCM")?;

    let capture_io = capture.io_i16().context("capture IO handle")?;
    let playback_io = playback.io_i16().context("playback IO handle")?;

    let mut input = [0i16; CHUNK_SIZE];
    let mut output = [0i16; CHUNK_SIZE];
    let mut phases = [0.0f32; MAX_VOICES];
    let mut last_reported = [0.0f32; MAX_VOICES];
    let mut current_gain = 0.0f32;
    let mut active_freqs = [0.0f32; MAX_VOICES];
    let mut active_count = 0usize;
    let mut frames_since_detection = HOLD_FRAMES;

    loop {
        read_chunk(&capture_io, &capture, &mut input)?;
        let level = rms_level(&input);
        let mut pitches = if level >= MIN_DETECTION_LEVEL {
            detect_pitches(
                &input,
                SAMPLE_RATE,
                MIN_FREQ,
                MAX_FREQ,
                MAX_VOICES,
                MIN_CORRELATION,
            )
        } else {
            Vec::new()
        };

        if !pitches.is_empty() {
            if pitches.len() > MAX_VOICES {
                pitches.truncate(MAX_VOICES);
            }

            active_count = pitches.len();
            active_freqs[..active_count].copy_from_slice(&pitches);
            for idx in active_count..MAX_VOICES {
                active_freqs[idx] = 0.0;
                last_reported[idx] = 0.0;
            }

            for (idx, &freq) in active_freqs.iter().take(active_count).enumerate() {
                if (freq - last_reported[idx]).abs() > 3.0 {
                    println!(
                        "Voice {}: {:.1} Hz -> {:.1} Hz",
                        idx + 1,
                        freq,
                        freq * SQRT_2
                    );
                    last_reported[idx] = freq;
                }
            }

            frames_since_detection = 0;
        } else if active_count > 0 {
            if frames_since_detection < HOLD_FRAMES {
                frames_since_detection += 1;
            } else {
                active_count = 0;
                active_freqs.fill(0.0);
                last_reported.fill(0.0);
                phases.fill(0.0);
                frames_since_detection = HOLD_FRAMES;
            }
        } else {
            frames_since_detection = HOLD_FRAMES;
        }

        let target_gain = (level * MAX_OUTPUT_GAIN).min(MAX_OUTPUT_GAIN);
        current_gain += (target_gain - current_gain) * GAIN_SMOOTHING;

        let playback_freqs: Vec<f32> = active_freqs
            .iter()
            .take(active_count)
            .map(|f| f * SQRT_2)
            .collect();
        synthesize_chunk(&mut output, &playback_freqs, &mut phases, current_gain);
        write_chunk(&playback_io, &playback, &output)?;
    }
}

fn open_pcm(direction: Direction) -> Result<PCM> {
    let pcm = PCM::new("default", direction, false)
        .with_context(|| format!("open {:?} PCM", direction))?;

    {
        let hwp = HwParams::any(&pcm)?;
        hwp.set_access(Access::RWInterleaved)?;
        hwp.set_format(Format::s16())?;
        hwp.set_channels(1)?;
        hwp.set_rate(SAMPLE_RATE, ValueOr::Nearest)?;
        hwp.set_period_size_near(CHUNK_SIZE as Frames, ValueOr::Nearest)?;
        hwp.set_buffer_size_near((CHUNK_SIZE * 2) as Frames)?;
        pcm.hw_params(&hwp)?;
    }

    pcm.prepare()?;
    Ok(pcm)
}

fn read_chunk(io: &IO<i16>, pcm: &PCM, buffer: &mut [i16]) -> Result<()> {
    let mut offset = 0;
    while offset < buffer.len() {
        match io.readi(&mut buffer[offset..]) {
            Ok(frames) => offset += frames,
            Err(err) if err.errno() == Errno::EPIPE => {
                pcm.prepare()?;
            }
            Err(err) if err.errno() == Errno::EAGAIN => continue,
            Err(err) => return Err(err.into()),
        }
    }
    Ok(())
}

fn write_chunk(io: &IO<i16>, pcm: &PCM, buffer: &[i16]) -> Result<()> {
    let mut offset = 0;
    while offset < buffer.len() {
        match io.writei(&buffer[offset..]) {
            Ok(frames) => offset += frames,
            Err(err) if err.errno() == Errno::EPIPE => {
                pcm.prepare()?;
            }
            Err(err) if err.errno() == Errno::EAGAIN => continue,
            Err(err) => return Err(err.into()),
        }
    }
    Ok(())
}

fn synthesize_chunk(buffer: &mut [i16], freqs: &[f32], phases: &mut [f32], gain: f32) {
    if freqs.is_empty() {
        buffer.fill(0);
        phases.fill(0.0);
        return;
    }

    let normalized_gain = gain.clamp(0.0, 1.0);
    let amplitude = i16::MAX as f32 * (normalized_gain / freqs.len() as f32);

    for sample in buffer.iter_mut() {
        let mut acc = 0.0f32;
        for (idx, freq) in freqs.iter().enumerate() {
            let phase = &mut phases[idx];
            acc += (*phase).sin();
            let phase_step = 2.0 * PI * freq / SAMPLE_RATE as f32;
            *phase += phase_step;
            if *phase > 2.0 * PI {
                *phase -= 2.0 * PI;
            }
        }
        let value = (acc * amplitude).clamp(i16::MIN as f32, i16::MAX as f32);
        *sample = value as i16;
    }

    for idx in freqs.len()..phases.len() {
        phases[idx] = 0.0;
    }
}

fn detect_pitches(
    samples: &[i16],
    sample_rate: u32,
    min_hz: f32,
    max_hz: f32,
    max_results: usize,
    min_correlation: f32,
) -> Vec<f32> {
    if samples.is_empty() || max_results == 0 {
        return Vec::new();
    }

    let mut floated: Vec<f32> = samples.iter().map(|&s| s as f32).collect();
    let mean = floated.iter().sum::<f32>() / floated.len() as f32;

    for sample in &mut floated {
        *sample -= mean;
    }

    apply_hann_window(&mut floated);

    let len = floated.len();
    if len < 2 {
        return Vec::new();
    }

    let min_period = ((sample_rate as f32) / max_hz).floor() as usize;
    let max_period = ((sample_rate as f32) / min_hz).ceil() as usize;

    if min_period < 2 || max_period >= len || min_period >= max_period {
        return Vec::new();
    }

    let mut energy_prefix = vec![0.0f32; len + 1];
    for (idx, sample) in floated.iter().enumerate() {
        energy_prefix[idx + 1] = energy_prefix[idx] + sample * sample;
    }

    let mut correlations: Vec<(usize, f32)> = Vec::with_capacity(max_period - min_period + 1);

    for lag in min_period..=max_period {
        let segment_len = len - lag;
        if segment_len < 2 {
            continue;
        }

        let energy_a = energy_prefix[segment_len] - energy_prefix[0];
        let energy_b = energy_prefix[len] - energy_prefix[lag];
        let denom = (energy_a * energy_b).sqrt();
        if denom <= 1e-9 {
            continue;
        }

        let mut sum = 0.0;
        for i in 0..segment_len {
            sum += floated[i] * floated[i + lag];
        }
        let normalized = (sum / denom).clamp(-1.0, 1.0);
        correlations.push((lag, normalized));
    }

    correlations.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(Ordering::Equal));

    let mut results: Vec<f32> = Vec::new();
    for (lag, corr) in correlations {
        if corr < min_correlation {
            continue;
        }

        let freq = sample_rate as f32 / lag as f32;
        let is_distinct = results
            .iter()
            .all(|&existing| (existing - freq).abs() > 5.0f32);
        if is_distinct {
            results.push(freq);
        }

        if results.len() == max_results {
            break;
        }
    }

    results
}

fn apply_hann_window(samples: &mut [f32]) {
    if samples.len() < 2 {
        return;
    }

    let denom = (samples.len() - 1) as f32;
    for (n, sample) in samples.iter_mut().enumerate() {
        let weight = 0.5 - 0.5 * (2.0 * PI * n as f32 / denom).cos();
        *sample *= weight;
    }
}

fn rms_level(samples: &[i16]) -> f32 {
    if samples.is_empty() {
        return 0.0;
    }

    let sum = samples
        .iter()
        .map(|&s| {
            let v = s as f32;
            v * v
        })
        .sum::<f32>();
    let rms = (sum / samples.len() as f32).sqrt();
    (rms / i16::MAX as f32).min(1.0)
}
