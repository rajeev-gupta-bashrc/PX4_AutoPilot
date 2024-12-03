#include <uORB/uORB.h>
#include <uORB/topics/actuator_motors.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/app.h>

#include <px4_platform_common/log.h>
#include <stdlib.h> // For atof
#include <string.h> // For strlen


class MotorCommandPublisher : public ModuleBase<MotorCommandPublisher>
{
public:
    MotorCommandPublisher();

    void publish(float motor1, float motor2, float motor3, float motor4);
    void loop(float motor1, float motor2, float motor3, float motor4);

    static px4::AppState appState; /* track requests to terminate app */


private:
    orb_advert_t _orb_pub; // Publisher handle
    struct actuator_motors_s _actuator_motors; // Define the actuator_motors topic structure
};
