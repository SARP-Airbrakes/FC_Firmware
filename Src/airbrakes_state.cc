
#include <airbrakes_state.h>

airbrakes_state::airbrakes_state(sdk::i2c_master i2c1) : imu(i2c1), baro(i2c1)
{
}

void airbrakes_state::update()
{

}

void airbrakes_state::refresh_imu()
{
    imu.update();
}

void airbrakes_state::refresh_baro()
{
    baro.update();
}
