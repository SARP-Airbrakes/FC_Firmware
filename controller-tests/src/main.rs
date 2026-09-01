
use indicatif::MultiProgress;
use indicatif_log_bridge::LogWrapper;
use log::*;
use clap::{Parser, Subcommand, Args};

use crate::flight::DataFormat;

mod flight;
mod filter;

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Cli {
    /// Enables verbose logging.
    #[arg(short = 'v', default_value_t = false)]
    verbose: bool,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Runs the Kalman filter on given data.
    Filter {
        /// Where to write the filtered output to.
        #[arg(short, long, default_value = "filter_out.csv")]
        output: String,

        #[command(flatten)]
        data: DataFlags,
    }
}

#[derive(Args)]
pub(crate) struct DataFlags {
    /// Format of the input .csv data.
    #[arg(short = 'f', long = "input-format", value_enum, default_value_t = DataFormat::Detect)]
    input_format: DataFormat,

    /// Only process packets from after this time.
    #[arg(short, long)]
    after: Option<f64>,

    /// Flight data file to parse.
    file: String
}

fn main() {
    let cli = Cli::parse();

    let logger = env_logger::builder()
        .filter_level(if cli.verbose { LevelFilter::Trace } else { LevelFilter::Debug })
        .build();
    let level = logger.filter();
    let multi = MultiProgress::new();
    
    LogWrapper::new(multi.clone(), logger)
        .try_init()
        .unwrap();
    log::set_max_level(level);

    if let Err(e) = match cli.command {
        Commands::Filter { output, data } => crate::filter::run(multi, output, data),
    } {
        log::error!("Got error: {}", e);
    }
}
