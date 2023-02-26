#include "main.h"
#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/api/odometry/odomState.hpp"
#include "okapi/api/odometry/point.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/impl/device/controllerUtil.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "HolonomicLib/API.hpp"

std::shared_ptr<OdomChassisController> chassis;
std::shared_ptr<okapi::XDriveModel> driveTrain;
std::shared_ptr<AsyncHolonomicChassisController> Holo_controller;

lv_obj_t * myButton;
lv_obj_t * myButtonLabel;
lv_obj_t * myLabel;

lv_style_t myButtonStyleREL; //relesed style
lv_style_t myButtonStylePR; //pressed style


//all the variables
#define index_mtr_prt 6
#define fly_mtr_prt 9
#define intake_prt 10
#define expansion_prt 19


float current_fly_pct=0;
float intake_mtr_speed = 0;
float index_mtr_speed = 0;
float indexer_pos=0;
int start_pos=2;

static lv_res_t btn_click_action(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn); //id usefull when there are multiple buttons

    if(id == 0)
    {
        char buffer[100];
		//sprintf(buffer, "button was clicked %i milliseconds from start", pros::millis());
        if (start_pos==1){
            start_pos=2;
        } else if (start_pos==2){
            start_pos=1;
        }
        sprintf(buffer, "starting position is %i, options 1,2", start_pos);
		lv_label_set_text(myLabel, buffer);
    }

    return LV_RES_OK;
}

// void disk_indexer(){ //indexes one disk

// }


void change_fly_speed(float fly_pct){
    pros::Motor fly_mtr(9);
    fly_mtr.move_velocity(fly_pct*6); //Max +-600
}

void ShootThree(bool auton) {
    pros::Motor index_mtr(6);
    if (auton == false){
        change_fly_speed(70);
    }else {
        change_fly_speed(35);
    }
    pros::delay(500);
    for(int i=0;i<3;i++) {
        indexer_pos-=180;
        index_mtr.move_absolute(indexer_pos,180);
        pros::delay(500);
    }
    change_fly_speed(0);
}


pros::Controller controller_master (pros::E_CONTROLLER_MASTER);

void expand(void){
    pros::Motor expansion_mtr(expansion_prt);
    expansion_mtr.move_relative(-180,100);
}
std::uint32_t RTOStime = pros::millis();
void PrintController(void* tmp) {
    while (true) {
        okapi::OdomState current_state = chassis->getState();
        //controller_master.print(0, 0, std::to_string(pros::c::motor_get_actual_velocity(fly_mtr_prt)).c_str()); //note: .c_str converts str to char
        int battery = round(pros::battery::get_capacity());
        std::string tmp1, tmp2, tmp3;
        tmp1 = std::to_string(battery);
        tmp2 = "Bat: "+tmp1.substr(0,2)+"(P)";
        controller_master.print(0,0,tmp2.c_str()); //"Battery: xx(P)"
        pros::delay(70);

        tmp1 = current_state.str(); 
        tmp2 = tmp1.substr(12,4)+", "+tmp1.substr(22,4)+", "+tmp1.substr(36,4);

        controller_master.print(1,0,tmp2.c_str());
        pros::delay(70);

        tmp1 = std::to_string(round(pros::c::motor_get_actual_velocity(fly_mtr_prt)/6)).c_str();
        tmp2 = std::to_string(round(pros::c::motor_get_temperature(fly_mtr_prt))).c_str();
        tmp3 = "V:"+tmp1.substr(0,3)+";T:"+tmp2.substr(0,3)+"°C";
        controller_master.print(2, 0,tmp3.c_str()); //note: .c_str converts str to char
        pros::Task::delay_until(&RTOStime, 300); //Every 1s regardless of pros::delay
    }
}
// void PrintTerminal(void* tmp) {
//     std::cout << pros::c::motor_get_actual_velocity(fly_mtr_prt) << std::endl;
//     std::cout << pros::c::motor_get_voltage(fly_mtr_prt) << std::endl;
//     std::cout << pros::c::motor_get_temperature(fly_mtr_prt) << std::endl;
// }
/*
Flywheel Testing
12° 45 Power 34 cm from left wall to original C-channel limit, touching red barrier

*/

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

// void myPID(void *tmp) {
//     float error, integral, derivative, prevError, power;
//     const float kP = 10, kI = 10, kD = 10;
//     while (!controller_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
//         error = current_fly_pct*6 - pros::c::motor_get_actual_velocity(fly_mtr_prt) ;
//         integral = integral + error;
//         derivative = error - prevError;
//         prevError = error;
//         power = error*kP + integral*kI + derivative*kD;
//         if (power > 100) {
//             power = 100;
//         }
//         change_fly_speed(power);
//         pros::Task::delay_until(&RTOStime, 15);
//     } 
// }

bool started = false;
void Roller(void *tmp) {
    driveTrain->forward(20);
    pros::Task::delay_until(&RTOStime, 1000);
}

void initialize() {
    pros::task_t my_task = pros::c::task_create(PrintController,NULL,TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PrintController");
    //pros::task_t task_2 = pros::c::task_create(myPID,NULL,TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "FlyWheelPID");

	lv_style_copy(&myButtonStyleREL, &lv_style_plain);
    myButtonStyleREL.body.main_color = LV_COLOR_MAKE(150, 0, 0);
    myButtonStyleREL.body.grad_color = LV_COLOR_MAKE(0, 0, 150);
    myButtonStyleREL.body.radius = 0;
    myButtonStyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);

    lv_style_copy(&myButtonStylePR, &lv_style_plain);
    myButtonStylePR.body.main_color = LV_COLOR_MAKE(255, 0, 0);
    myButtonStylePR.body.grad_color = LV_COLOR_MAKE(0, 0, 255);
    myButtonStylePR.body.radius = 0;
    myButtonStylePR.text.color = LV_COLOR_MAKE(255, 255, 255);

    myButton = lv_btn_create(lv_scr_act(), NULL); //create button, lv_scr_act() is deafult screen object
    lv_obj_set_free_num(myButton, 0); //set button is to 0
    lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action); //set function to be called on button click
    lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL); //set the relesed style
    lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR); //set the pressed style
    lv_obj_set_size(myButton, 200, 50); //set the button size
    lv_obj_align(myButton, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10); //set the position to top mid

    myButtonLabel = lv_label_create(myButton, NULL); //create label and puts it inside of the button
    lv_label_set_text(myButtonLabel, "--"); //sets label text

    myLabel = lv_label_create(lv_scr_act(), NULL); //create label and puts it on the screen
    lv_label_set_text(myLabel, "Button has not been clicked yet"); //sets label text
    lv_obj_align(myLabel, NULL, LV_ALIGN_IN_LEFT_MID, 10, 0); //set the position to center

	//motor init
	pros::Motor fly_mtr_initializer(fly_mtr_prt,pros::E_MOTOR_GEARSET_06,true, pros::E_MOTOR_ENCODER_DEGREES);

    //motor init for intake
    pros::Motor intake_mtr_initializer(intake_prt, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);

    //motor for indexer
    pros::Motor index_mtr_initializer(index_mtr_prt, pros::E_MOTOR_GEARSET_18,true, pros::E_MOTOR_ENCODER_DEGREES);

    pros::Motor expansion_mtr_initalizer(expansion_prt, pros::E_MOTOR_GEARSET_36,true, pros::E_MOTOR_ENCODER_DEGREES);

    //x-drive init
    chassis = //drive is inited as a global var so it can be used everywhere
        ChassisControllerBuilder() 
            //.withMotors(3,-2,-20,11) //Top Left, Top Right, Bottom Right, Bottom Left
            /*
            //config,FR, Turn, Strife
            0000 No, No, No
            0001 Yes?, Yes?, Yes?
            0010 No, No, No
            0011 NO, No, No
            0100 No
            0101 No, No, No
            0110 Invertedm Inverted, Inverted - so lets flip it
            0111
            1000
            1001 Yes, Yes, Yes
            1010
            1011
            1100 No, No, No
            1101
            1110
            1111
            */
            //old correct 25/02
            //.withMotors(-20,11,3,-2) //Top Left, Top Right, Bottom Right, Bottom Left //16 combos because 2^4 
            
            .withMotors(-3,2,20,-11) //Top Left, Top Right, Bottom Right, Bottom Left //16 combos because 2^4 
            .withDimensions({AbstractMotor::gearset::green,(60.0/84.0)}, {{4_in, 26.5_in}, imev5GreenTPR}) //Track length, Gearing
            .withMaxVelocity(100)
            .withSensors(
                ADIEncoder{'A','B', true},
                ADIEncoder{'C','D', true},
                ADIEncoder{'E','F',true}
            )
            //.withOdometry({{2.75_in, 7.5_in, 9_in, 2.75_in}, quadEncoderTPR}) //quadEncoderTPR=fixed variable representing the ticks per rotation of the red v1 potentiometers 
            .withOdometry({{2.75_in, 6.5_in, 9.5_in, 2.75_in}, quadEncoderTPR}) //quadEncoderTPR=fixed variable representing the ticks per rotation of the red v1 potentiometers 
            .buildOdometry();



    driveTrain = std::dynamic_pointer_cast<XDriveModel>(chassis->getModel()); //switch to ThreeEncoderXDriveModel???
    // assigning the chassis to a Three Encoder X-drive model
    driveTrain->setBrakeMode(AbstractMotor::brakeMode::hold);

    Holo_controller = AsyncHolonomicChassisControllerBuilder(chassis)
        // PID gains (must be tuned for your robot)
        .withDistGains(
            {0.02, 0.0, 0.00025, 0.0} // Translation gains
        )
        .withTurnGains(
            {0.02, 0.0, 0.00025, 0.0} // Turn gains
        )
        //.withDistGains(const okapi::IterativePosPIDController::Gains &idistGains)
        //.withTurnGains(const okapi::IterativePosPIDController::Gains &iturnGains)
        // Tolerance (how close the chassis must be to the target before stopping)
        /*.withDistSettleParameters(
            0.5_in, // Max error
            2.0_in / 1_s, // Max derivative
            100_ms // Wait time
        )
        .withTurnSettleParameters(
            1.0_in, // Max error
            2.0_in / 1_s, // Max derivative
            100_ms // Wait time
        )*/
        .build();
    
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

void index(){
    pros::Motor index_mtr(index_mtr_prt); //port 6
    indexer_pos-=180;
    index_mtr.move_absolute(indexer_pos,180);
}
/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    pros::Motor fly_mtr(9);
    pros::Motor intake_mtr(10);
    pros::Motor index_mtr(index_mtr_prt); //port 6
    pros::Motor expansion_mtr(expansion_prt);

    if (start_pos==1){
        chassis->setState({0.36_m, 0.92_m, 180_deg}); //todo: to be changed to a configurable value depending on the starting position on the pitch 
    } else if (start_pos==2){
        chassis->setState({2.12_m, 3.35_m, 340_deg}); //todo: to be changed to a configurable value depending on the starting position on the pitch 
    }
    // std::cout << "in auton" << " , ";
    // Holo_controller->setTarget({3_m, 3_m, 0_deg}, true);
    //chassis->driveToPoint({0_ft,3_ft});
    //Holo_controller->waitUntilSettled();
    // std::cout << "ran auton" << " , ";

    if (start_pos==1){
        //auton routine 1
        //Holo_controller->setTarget({1_m, 0_m, 0_deg}, false);
        //chassis->driveToPoint({-0.9_m,-0.3_m});
        //driveTrain->setBrakeMode(MOTOR_BRAKE_HOLD);
        //driveTrain->forward(1);
        //intake_mtr.move_velocity(50);
        //pros::delay(500);
        //driveTrain->stop();
        // driveTrain->xArcade(0,10,0);
        // pros::delay(2000);

        //intake_mtr.move_velocity(50);
        intake_mtr.move_relative(-500,50);
        //inch forward
        pros::task_t roller_inch_forward = pros::c::task_create(Roller,NULL,TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "RollerInchForward");
        //move intake async 
        pros::delay(1000);
        pros::c::task_suspend(roller_inch_forward);
        //intake_mtr.move_velocity(0);
        //stop the roller inch forward task (self stopping after x ms)
        //driveToPoint
        // chassis->turnToPoint({0.95_m,1.53_m});
        // chassis->driveToPoint({1.65_m, 1.4_m});
        // //turn to point 
        // chassis->turnToPoint({0.5_m, 3.15_m});
        // pros::delay(5000);
        //fly wheel 70%
        // ShootThree(true);
        //launch 2 discs
        //move to point 1.5,0.8
        // chassis->driveToPoint({1.54_m, 0.95_m});
        // pros::delay(5000);
        // chassis->moveDistance(0.1_m);
        // pros::delay(1000);
        // chassis->moveDistance(-0.1_m);
        // chassis->driveToPoint({1.65_m, 1.4_m});
        // chassis->turnToPoint({0.5_m, 3.15_m});
        // ShootThree(true);
    } else if(start_pos==2){
        //atuon routine 2
        //move intake async 
        //chassis->driveToPoint({2.96_m,3.34_m});
        //chassis->turnToPoint({3.1_m, 3.9_m});
        
        /* auton v1 11am
        intake_mtr.move_velocity(130);
        pros::task_t roller_inch_forward = pros::c::task_create(Roller,NULL,TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "RollerInchForward");
        pros::delay(2000);
        pros::c::task_suspend(roller_inch_forward); */ 
        
        
        //chassis->driveToPoint({1.36_m, 1.62_m});
        //chassis->turnToPoint({3.13_m, 0.52_m});
        // pros::delay(5000);
        // ShootThree(true);
        // chassis->driveToPoint({0.94_m, 1.52_m});
        // pros::delay(1000);
        // chassis->driveToPoint({1.25_m, 1.83_m});
        // pros::delay(1000);
        // chassis->driveToPoint({1.56_m, 2.15_m});
        // chassis->turnToPoint({3.13_m,0.52_m});
        // ShootThree(true);


        //auton v2 12pm
        change_fly_speed(60);
        chassis->moveDistance(0.7_m);
        pros::delay(500);
        chassis->turnAngle(30_deg);
        pros::delay(4000);
        index();
        pros::delay(1300);
        index();
        pros::delay(1300);
        index();
        pros::delay(72000);
        expand();

        
    }

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

/*float round_2dp(float var){
        // 37.66666 * 100 =3766.66
        // 3766.66 + .5 =3767.16    for rounding off value
        // then type cast to int so value is 3767
        // then divided by 100 so the value converted into 37.67
        float value = (int)(var * 100 + .5);
        return (float)value / 100;
    }*/
        
void opcontrol() {
    std::ios_base::sync_with_stdio(0);
    std::cout.tie(0);
    pros::c::motor_set_brake_mode(9, pros::E_MOTOR_BRAKE_COAST);

	// pros::Controller master(pros::E_CONTROLLER_MASTER);
	// pros::Motor left_mtr(1);
	// pros::Motor right_mtr(2);
	pros::Motor fly_mtr(9);
    pros::Motor intake_mtr(10);
    pros::Motor index_mtr(index_mtr_prt); //port 6
    pros::Motor expansion_mtr(expansion_prt);

    //Controller controller;
    okapi::Controller controller_okapi = Controller();

    // change_fly_speed(10);

    //std::cout << "in fake auton" << " , ";
    //Holo_controller->setTarget({0.5_m, 0_m, 0_deg}, true);
    //std::cout << "ran fake auton" << " , ";

    while (true) {
        //Tank drive with left and right sticks
        //drive->setState({0_in,0_in,0_deg});
        //chassis->driveToPoint({0_ft,3_ft});

        //read the input from the controller sticks
        //int controller_turn = controller_master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X); //this uses PROS
        double controller_turn = controller_okapi.getAnalog(ControllerAnalog::leftX); //this uses okapilib - not sure about the differences
        double controller_forward = controller_okapi.getAnalog(ControllerAnalog::rightY); //todo: implement s curve for the controller
        double controller_strife = controller_okapi.getAnalog(ControllerAnalog::rightX);
        /*double controller_forward = -controller_okapi.getAnalog(ControllerAnalog::rightY); //todo: implement s curve for the controller
        double controller_strife = -controller_okapi.getAnalog(ControllerAnalog::rightX);*/
        driveTrain->xArcade(controller_strife,controller_forward, controller_turn); //should be used for degraded control when odom is off - voltage mode, note - +1 =1 is max min
        //double QAngle = controller_okapi->getPose().theta.convert(degree);
        //lv_label_set_text(myLabel, "controller turn %d",controller_turn);
        // char to_be_printed[100];
        //std::string to_be_printed = ("forward input %f,turn input %f, strife input %f",controller_forward,controller_turn,controller_strife);
        //sprintf(to_be_printed,"forward input %f,turn input %f, strife input %f",controller_forward,controller_turn,controller_strife);
        //lv_label_set_text(myLabel, to_be_printed);
        //driveTrain->fieldOrientedXArcade(controller_strife,controller_forward, controller_turn,chassis->getState().theta);// you can use an IMU as an alternative for the QAngle //chassis->getPose().theta.convert(degree)
        //driveTrain->fieldOrientedXArcade(controller_strife,controller_forward, controller_turn,chassis->getState().theta);// you can use an IMU as an alternative for the QAngle //chassis->getPose().theta.convert(degree)
        
        okapi::OdomState current_state = chassis->getState();
        //controller.setText(0, 0, "x %f,y %f, theta %f",current_state.x,current_state.y,current_state.theta);
        //controller_master.print(0, 0, "x %f,y %f, theta %f",current_state.x,current_state.y,current_state.theta);
        //controller_master.print(0, 0, current_state.str().c_str());
        //std::string to_be_printed = ("x %f,y %f, theta %f",current_state.x,current_state.y,current_state.theta);
        
        //good debugging - just swapping the screen output to debug something else
        lv_label_set_text(myLabel, current_state.str().c_str());
        //controller_master.print(3, 3, "theta %f",current_state.theta);
        
        /*chassis->setMaxVelocity(20);
        chassis->driveToPoint({50_cm,0_cm});
        chassis->driveToPoint({50_cm,50_cm});
        chassis->driveToPoint({0_cm,50_cm});
        chassis->driveToPoint({0_cm,0_cm});*/

        //index_mtr_speed += controller_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)*50; 
        //index_mtr_speed -= controller_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)*50;
        intake_mtr_speed -= controller_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)*50;
        intake_mtr_speed += controller_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)*50;
        intake_mtr.move_velocity(intake_mtr_speed);
        
        current_fly_pct+=controller_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)*5; //increase fly speed by 5%
        current_fly_pct-=controller_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)*5; //decrease fly speed by 5%
        change_fly_speed(current_fly_pct);
        std::cout << pros::c::motor_get_actual_velocity(fly_mtr_prt) << " , ";
        //std::cout << pros::c::motor_get_temperature(fly_mtr_prt) << " , ";
        //std::cout << pros::c::motor_get_voltage_limit(fly_mtr_prt) << std::endl;
        if (controller_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            chassis->turnToPoint({3.0_m, 0.65_m});
        }
        //if (controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_A)
        //controller_master.print(0, 0, current_state.str().c_str());


        //indexer
        if(controller_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
            indexer_pos-=180;
            index_mtr.move_absolute(indexer_pos,180);
        }
        if (controller_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            ShootThree(false);
        }
        if (controller_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            change_fly_speed(45);
        }
        if (controller_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            change_fly_speed(0);
        }

        //expansion
        if(controller_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
            expand();
        }


        pros::delay(30); //x ms delay
    }
}
