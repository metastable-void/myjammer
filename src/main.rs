use std::f32::consts::{PI, SQRT_2};

use alsa::nix::errno::Errno;
use alsa::pcm::{Access, Format, Frames, HwParams, IO, PCM};
use alsa::{Direction, ValueOr};
use anyhow::{Context, Result};

const SAMPLE_RATE: u32 = 48_000;
const CHUNK_SIZE: usize = 4096;
const MIN_FREQ: f32 = 60.0;
const MAX_FREQ: f32 = 1000.0;
const MIN_OUTPUT_GAIN: f32 = 0.05;
const MAX_OUTPUT_GAIN: f32 = 0.8;
const GAIN_SMOOTHING: f32 = 0.15;

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
    let mut phase = 0.0f32;
    let mut last_reported = 0.0f32;
    let mut current_gain = MIN_OUTPUT_GAIN;

    loop {
        read_chunk(&capture_io, &capture, &mut input)?;
        let pitch = detect_pitch(&input, SAMPLE_RATE, MIN_FREQ, MAX_FREQ);

        if let Some(freq) = pitch {
            if (freq - last_reported).abs() > 3.0 {
                println!("Detected {:.1} Hz -> playing {:.1} Hz", freq, freq * SQRT_2);
                last_reported = freq;
            }
        }

        let level = rms_level(&input);
        let target_gain = MIN_OUTPUT_GAIN + level * (MAX_OUTPUT_GAIN - MIN_OUTPUT_GAIN);
        current_gain += (target_gain - current_gain) * GAIN_SMOOTHING;

        let target_freq = pitch.map(|f| f * SQRT_2).unwrap_or(0.0);
        synthesize_chunk(&mut output, target_freq, &mut phase, current_gain);
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

fn synthesize_chunk(buffer: &mut [i16], frequency: f32, phase: &mut f32, gain: f32) {
    if frequency <= 0.0 {
        buffer.fill(0);
        return;
    }

    let amplitude = i16::MAX as f32 * gain.clamp(0.0, 1.0);
    let phase_step = 2.0 * PI * frequency / SAMPLE_RATE as f32;

    for sample in buffer.iter_mut() {
        *sample = (amplitude * (*phase).sin()) as i16;
        *phase += phase_step;
        if *phase > 2.0 * PI {
            *phase -= 2.0 * PI;
        }
    }
}

fn detect_pitch(samples: &[i16], sample_rate: u32, min_hz: f32, max_hz: f32) -> Option<f32> {
    if samples.is_empty() {
        return None;
    }

    let mut floated: Vec<f32> = samples.iter().map(|&s| s as f32).collect();
    let mean = floated.iter().sum::<f32>() / floated.len() as f32;

    for sample in &mut floated {
        *sample -= mean;
    }

    apply_hann_window(&mut floated);

    let len = floated.len();
    if len < 2 {
        return None;
    }

    let min_period = ((sample_rate as f32) / max_hz).floor() as usize;
    let max_period = ((sample_rate as f32) / min_hz).ceil() as usize;

    if min_period < 2 || max_period >= len || min_period >= max_period {
        return None;
    }

    let mut best_lag = 0usize;
    let mut best_corr = f32::MIN;

    for lag in min_period..=max_period {
        let mut sum = 0.0;
        for i in 0..(len - lag) {
            sum += floated[i] * floated[i + lag];
        }
        let normalized = sum / (len - lag) as f32;
        if normalized > best_corr {
            best_corr = normalized;
            best_lag = lag;
        }
    }

    if best_lag == 0 || best_corr <= 0.0 {
        return None;
    }

    Some(sample_rate as f32 / best_lag as f32)
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
