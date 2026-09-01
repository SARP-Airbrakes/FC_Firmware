use std::error::Error;

use controller::{filter::Filter, stage::FlightStage};
use indicatif::{MultiProgress, ProgressBar, ProgressStyle};

use crate::{DataFlags, flight::FlightData};

pub fn run(multi: MultiProgress, output: String, flags: DataFlags) -> Result<(), Box<dyn Error>> {
    let mut data = FlightData::from_flags(flags)?;

    let (ref_alt, mut filter) = {
        let mut iter = data.packets();
        let alt: f32;
        let time_ms: u64;
        let packet = iter.next().unwrap()?;
        alt = controller::estimated_altitude(packet.pressure_pa);
        time_ms = (packet.time_s * 1000.0) as u64;

        log::debug!("Initializing filter with altitude {} m and initial time {} ms", alt, time_ms);
        (alt, Filter::new(alt, time_ms))
    };
    let mut stage = FlightStage::Idle;

    data.reset()?;
    let pg = multi.add(ProgressBar::new(data.count() as u64));

    pg.set_style(ProgressStyle::with_template("{spinner:.green} [{elapsed_precise}] [{wide_bar:.cyan/blue}] {pos}/{len} ({eta})")
        .unwrap()
        .progress_chars("#>-"));

    let mut wtr = csv::Writer::from_path(output)?;
    wtr.write_record(vec![
        "packet_id",
        "time_s",
        "accel_x_mps2",
        "accel_y_mps2",
        "accel_z_mps2",
        "ang_vel_x_ds",
        "ang_vel_y_ds",
        "ang_vel_z_ds",
        "baro_altitude_m",
        "reference_altitude_m",
        "agl_altitude_m",
        "estimated_accel_x_mps2",
        "estimated_accel_y_mps2",
        "estimated_accel_z_mps2",
        "estimated_altitude_m",
        "estimated_upward_velocity_mps",
        "pressure_pascals",
        "temperature_c",
        "current_state",
        "motor_target_degrees",
        "motor_actual_degrees",
        "motor_commanded_power",
        "flap_target_degrees",
    ])?;

    // for tracking the active time
    let mut start_time_s: f64 = 0.0;
    let mut end_time_s: f64 = 0.0;

    let mut apogee_baro: f32 = 0.0;
    let mut apogee_filtered: f32 = 0.0;

    let mut apogee_stage: f32 = 0.0;

    let mut i = 0;
    data.reset()?;
    for packet in data.packets() {
        let packet = packet?;
        let time_ms = (packet.time_s * 1000.0) as u64;
        log::trace!("pre acc update cov: {}", filter.covariance());

        let _ = filter.update_acceleration(
            time_ms, 
            packet.accel_z_mps2, 
            0.0,
            stage,
        );

        log::trace!("post acc update cov: {}", filter.covariance());

        let _ = filter.update_pressure(time_ms, packet.pressure_pa);

        log::trace!("post press update cov: {}", filter.covariance());

        if !filter.altitude().is_finite() {
            log::error!("Numerical stability lost");
            pg.abandon_with_message("Numerical stability lost");
            multi.remove(&pg);
            return Ok(())
        }

        if let Some(new_stage) = stage.next(
            time_ms, 
            filter.upward_acceleration(packet.accel_z_mps2),
            filter.upward_velocity()
        ) {
            match new_stage {
                FlightStage::ActiveCoast => { start_time_s = packet.time_s; },
                FlightStage::Recovery => { 
                    apogee_stage = filter.altitude();
                    end_time_s = packet.time_s; 
                },
                _ => {}
            }
            // Switch stages if a new stage is detected.
            stage = new_stage;
        }

        log::trace!("packet time={} alt={} vel={}", packet.time_s, filter.altitude(), filter.upward_velocity());

        let est_alt = controller::estimated_altitude(packet.pressure_pa);

        apogee_baro = f32::max(apogee_baro, est_alt);
        apogee_filtered = f32::max(apogee_filtered, filter.altitude());

        wtr.write_record(vec![
            i.to_string(), // "packet_id",
            packet.time_s.to_string(), // "time_s",
            packet.accel_x_mps2.to_string(), // "accel_x_mps2",
            packet.accel_y_mps2.to_string(), // "accel_y_mps2",
            packet.accel_z_mps2.to_string(), // "accel_z_mps2",
            "0.0".to_string(), // "ang_vel_x_ds",
            "0.0".to_string(), // "ang_vel_y_ds",
            "0.0".to_string(), // "ang_vel_z_ds",
            est_alt.to_string(), // "baro_altitude_m",
            ref_alt.to_string(), // "reference_altitude_m",
            (est_alt - ref_alt).to_string(), // "agl_altitude_m"
            packet.accel_x_mps2.to_string(), // "estimated_accel_x_mps2",
            packet.accel_y_mps2.to_string(), // "estimated_accel_y_mps2",
            filter.upward_acceleration(packet.accel_z_mps2).to_string(), // "estimated_accel_z_mps2",
            filter.altitude().to_string(), // "estimated_altitude_m",
            filter.upward_velocity().to_string(), // "estimated_upward_velocity_mps",
            packet.pressure_pa.to_string(), // "pressure_pascals",
            "15.0".to_string(), // "temperature_c",
            (match stage {
                FlightStage::Idle => "0",
                FlightStage::Boost { .. } => "1",
                FlightStage::InactiveCoast => "1",
                FlightStage::ActiveCoast => "2",
                FlightStage::Recovery => "3",
                _ => "0"
            }).to_string(), // "current_state",
            "0.0".to_string(), // "motor_target_degrees",
            "0.0".to_string(), // "motor_actual_degrees",
            "0.0".to_string(), // "motor_commanded_power",
            "0.0".to_string(), // "flap_target_degrees",
        ])?;
        i += 1;

        pg.inc(1);
    }

    log::info!("In active flight for {:.2}s ({:.2}s -> {:.2}s)", end_time_s - start_time_s, start_time_s, end_time_s);
    log::info!("Apogee measured (barometer): {:.2} m ASL, {:.2} m AGL", apogee_baro, apogee_baro - ref_alt);
    log::info!("Apogee filtered: {:.2} m ASL, {:.2} m AGL", apogee_filtered, apogee_filtered - ref_alt);
    log::info!("Apogee as detected by FSM: {:.2} m ASL, {:.2} m AGL", apogee_stage, apogee_stage - ref_alt);

    pg.finish();
    multi.remove(&pg);

    wtr.flush()?;

    Ok(())

}