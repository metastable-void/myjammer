use alsa::nix::errno::Errno;
use alsa::pcm::{Access, Format, Frames, HwParams, IO, PCM};
use alsa::{Direction, ValueOr};
use anyhow::{Context, Result};
use clap::Parser;
use echo_nlms::NlmsCanceller;

const SAMPLE_RATE: u32 = 48_000;
const CHUNK_SIZE: usize = 4096;
const DELAY_MS: u32 = 150;
const AEC_TAPS: usize = 2048;
const NLMS_STEP_SIZE: f32 = 0.1;
const MIN_RENDER_LEVEL: f32 = 0.002;
const DOUBLE_TALK_RATIO: f32 = 2.5;

#[derive(Parser, Debug)]
#[command(name = "delay-jammer")]
struct Args {
    /// Disable adaptive echo suppression (use when monitoring via headphones).
    #[arg(long)]
    disable_echo: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();
    run(args.disable_echo)
}

fn run(disable_echo: bool) -> Result<()> {
    let capture = open_pcm(Direction::Capture).context("failed to open capture PCM")?;
    let playback = open_pcm(Direction::Playback).context("failed to open playback PCM")?;

    let capture_io = capture.io_i16().context("capture IO handle")?;
    let playback_io = playback.io_i16().context("playback IO handle")?;

    let mut input = [0i16; CHUNK_SIZE];
    let mut cleaned = [0i16; CHUNK_SIZE];
    let mut output = [0i16; CHUNK_SIZE];
    let mut render_history = [0i16; CHUNK_SIZE];

    let delay_frames = ((SAMPLE_RATE as u64 * DELAY_MS as u64) / 1000).max(1) as usize;
    let mut delay_line = vec![0i16; delay_frames];
    let mut delay_pos = 0usize;

    let mut canceller = if disable_echo {
        None
    } else {
        Some(NlmsCanceller::new(AEC_TAPS, NLMS_STEP_SIZE))
    };

    loop {
        read_chunk(&capture_io, &capture, &mut input)?;

        if let Some(canceller) = canceller.as_mut() {
            let render_level = rms_level(&render_history);
            let capture_level = rms_level(&input);
            let adapt = render_level > MIN_RENDER_LEVEL
                && capture_level <= render_level * DOUBLE_TALK_RATIO;
            canceller.process_block(&render_history, &input, &mut cleaned, adapt);
        } else {
            cleaned.copy_from_slice(&input);
        }

        process_delay(&cleaned, &mut output, &mut delay_line, &mut delay_pos);
        write_chunk(&playback_io, &playback, &output)?;
        render_history.copy_from_slice(&output);
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

fn process_delay(input: &[i16], output: &mut [i16], delay_line: &mut [i16], delay_pos: &mut usize) {
    for (idx, &sample) in input.iter().enumerate() {
        let delayed = delay_line[*delay_pos];
        delay_line[*delay_pos] = sample;
        *delay_pos += 1;
        if *delay_pos == delay_line.len() {
            *delay_pos = 0;
        }

        output[idx] = delayed;
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
