
#include <cli.h>
#include <airbrakes.h>
#include <selfcheck.h>
#include <cmsis_os.h>

#include <sdk/usbtransport.hpp>
#include <sdk/cli.hpp>

bool cli_disabled = false;

sdk::usb_transport<> cli_transport;
sdk::cli *global_cli;

static constexpr sdk::cli::command cli_commands[] = {
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
        [](sdk::cli &, int, char *[]) {
            airbrakes_state::flight_packet::print_packet_header();
            int i = 0;
            for (;;) {
                airbrakes_state::flight_packet packet = state_handle->read_packet(i++);
                if (packet.packet_id != i - 1)
                    break;
                packet.print_packet();
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
    {
        "forcelog",
        [](sdk::cli &, int, char *[]) {
            airbrakes_state::flight_packet::print_packet_header();
            state_handle->force_log = true;
            cli_disabled = true;
            return 0;
        },
        "Turns off CLI and switches to just printing flight packets. Too epic for normal use."
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
    if (!cli_disabled)
        cli_transport.on_rx_isr(buf, len);
}

void cli_transmit_completed(void)
{
    cli_transport.on_tx_complete_isr();
}

void cli_poll(void)
{
    if (!cli_disabled)
        global_cli->poll();
}

void cli_process_tx(void)
{
    cli_transport.process_tx();
}

}
