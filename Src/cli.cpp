
#include <cli.h>
#include <airbrakes.h>
#include <selfcheck.h>
#include <cmsis_os.h>

#include <sdk/usbtransport.hpp>
#include <sdk/cli.hpp>

sdk::usb_transport<> cli_transport;
sdk::cli *global_cli;

static constexpr sdk::cli::command cli_commands[] = {
    {
        "status",
        [](sdk::cli &c, int, char *[]) {
            c.println("Hello bro");
            return 0;
        },
        "Report status of airbrakes system."
    },
    {
        "erase",
        [](sdk::cli &c, int, char *[]) {
            c.println("Erasing the flash chip. This may take a while...");
            auto status = state_handle->flash.erase();
            if (status.is_ok()) {
                c.println("Erased flash chip");
            } else {
                c.println("Try again");
            }

            return 0;
        },
        "Erases the flash chip."
    },
    {
        "dump",
        [](sdk::cli &c, int, char *[]) {
            c.println("packet_id,time_s,accel_x_mps2,accel_y_mps2,accel_z_mps2,ang_vel_x_ds,ang_vel_y_ds,ang_vel_z_ds,acc_altitude_m,baro_altitude_m,reference_altitude_m,agl_altitude_m,acc_velocity_mps,baro_velocity_mps,fused_velocity_mps,pressure_pascals,temperature_c,gps_altitude_m,current_state,motor_target_degrees,motor_actual_degrees,motor_commanded_power,flap_target_degrees,fix_status");
            int i = 0;
            for (;;) {
                airbrakes_state::flight_packet packet = state_handle->read_packet(i++);
                if (packet.packet_id != i - 1)
                    break;
                printf(
                    "%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%d\r\n",
                    packet.packet_id,
                    packet.time_s,
                    packet.accel_x_mps2,
                    packet.accel_y_mps2,
                    packet.accel_z_mps2,
                    packet.ang_vel_x_ds,
                    packet.ang_vel_y_ds,
                    packet.ang_vel_z_ds,
                    packet.acc_altitude_m,
                    packet.baro_altitude_m,
                    packet.reference_altitude_m,
                    packet.agl_altitude_m,
                    packet.acc_velocity_mps,
                    packet.baro_velocity_mps,
                    packet.fused_velocity_mps,
                    packet.pressure_pascals,
                    packet.temperature_c,
                    packet.gps_altitude_m,
                    (int) packet.current_state,
                    packet.motor_target_degrees,
                    packet.motor_actual_degrees,
                    packet.motor_commanded_power,
                    packet.flap_target_degrees,
                    packet.fix_status
                );
                osDelay(10);
            }
            return 0;
        },
        "Prints flight log data in CSV format."
    },
    {
        "check",
        [](sdk::cli &, int, char *[]) {
            selfcheck_test_and_print();
            return 0;
        },
        "Runs diagnostic tests."
    },
    {
        "set_target",
        [](sdk::cli &c, int argc, char *argv[]) {
            if (argc != 2) {
                c.println("set_target [target]");
                return 1;
            }
            float target_degrees = atoff(argv[1]);
            printf("setting target to %f\r\n", target_degrees);
            state_handle->servo.set_target_degrees(target_degrees);
            return 0;
        },
        "Set target degrees of motor controller."
    },
};

extern "C" {

int __io_putchar(int ch)
{
    uint8_t b = static_cast<uint8_t>(ch);
    global_cli->write_bytes(&b, 1);
    return ch;
}

void cli_init(void)
{
    static sdk::cli cli_(cli_transport);

    global_cli = &cli_;
    global_cli->set_commands(cli_commands, sizeof(cli_commands) /
            sizeof(sdk::cli::command));
    global_cli->begin();
}

void cli_receive(uint8_t *buf, uint32_t len)
{
    cli_transport.on_rx_isr(buf, len);
}

void cli_transmit_completed(void)
{
    cli_transport.on_tx_complete_isr();
}

void cli_poll(void)
{
    global_cli->poll();
}

void cli_process_tx(void)
{
    cli_transport.process_tx();
}

}
