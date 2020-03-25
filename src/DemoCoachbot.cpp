#include "Coachbot.h"

namespace Kilosim
{
class DemoCoachbot : public Coachbot
{

    void setup()
    {
        drive_robot(50, 40);
        set_led(100, 100, 100);
    };

    void loop(){};
};
} // namespace Kilosim
