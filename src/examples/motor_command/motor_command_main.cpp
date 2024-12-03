#include <uORB/uORB.h>
#include <uORB/topics/actuator_motors.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/app.h>

#include <px4_platform_common/log.h>
#include <px4_platform_common/init.h>
#include <stdlib.h> // For atof
#include <string.h> // For strlen

#include "motor_command_main.h"

static int daemon_motor_command_pub;


int daemon_motor_command(int argc, char *argv[])
{
	printf("Starting motor command publisher \n");


	MotorCommandPublisher publisher;
    publisher.appState.setRunning(true);
    float motor1, motor2, motor3, motor4;

    // for (int i = 0; i < argc; i++)
    // {
    //     printf("%s\n", argv[i]);
    // }
    /*
    argv passed here are:
        "Motor Command" "motor_command" "stop" : length = 3
    */


    if(argc == 2){
        motor1 = 0.5;
        motor2 = 0.5;
        motor3 = 0.5;
        motor4 = 0.5;
    }
    else if(argc == 3 && !strcmp(argv[2], "stop")){
        publisher.appState.requestExit();
    }
    else if (argc == 6) {
        motor2 = atof(argv[2]);
        motor3 = atof(argv[3]);
        motor4 = atof(argv[4]);
        motor4 = atof(argv[5]);
    }
    else{
        PX4_WARN("number of arguments passed to daemon = %d", argc);
        PX4_WARN("Usage: motor_command");
        PX4_WARN("Usage: motor_command <motor1> <motor2> <motor3> <motor4>");
        PX4_WARN("Usage: motor_command stop");
        return 1;
    }
    publisher.loop(motor1, motor2, motor3, motor4);

    return 0;
}





MotorCommandPublisher::MotorCommandPublisher(){
    _actuator_motors = {};  // Zero-initialized
    _orb_pub = orb_advertise(ORB_ID(actuator_motors), &_actuator_motors);
}

void MotorCommandPublisher::publish(float motor1=0.0, float motor2=0.0, float motor3=0.0, float motor4=0.0)
{
    // Set motor command values
    _actuator_motors.control[0] = motor1; // Set motor 1 command
    _actuator_motors.control[1] = motor2; // Set motor 2 command
    _actuator_motors.control[2] = motor3; // Set motor 3 command
    _actuator_motors.control[3] = motor4; // Set motor 4 command

    // Publish the actuator_motors message
    orb_publish(ORB_ID(actuator_motors), _orb_pub, &_actuator_motors);
}

void MotorCommandPublisher::loop(float motor1=0.0, float motor2=0.0, float motor3=0.0, float motor4=0.0){
    // Publish the motor command values
    // PX4_INFO("motor commands published %f, %f, %f, %f \n", double(motor1), double(motor2), double(motor3), double(motor4));
    while (!appState.exitRequested()) {
        usleep(100);
        publish(motor1, motor2, motor3, motor4);
    }
}

px4::AppState MotorCommandPublisher::appState;  /* track requests to terminate app */


extern "C" __EXPORT int motor_command_main(int argc, char *argv[])
{
    char *motor_command_args[argc+1];
    for (int i = 0; i < argc; i++)
    {
        motor_command_args[i] = argv[i];
    }
    motor_command_args[argc] = nullptr;

    daemon_motor_command_pub =  px4_task_spawn_cmd("Motor Command",
						 	    SCHED_DEFAULT,
						        SCHED_PRIORITY_MAX - 5,
								2000,
						 		daemon_motor_command,
						 		motor_command_args);


    return 0;
}
