#include "main.h"
#include "okapi/api/odometry/odomState.hpp"
#include "okapi/impl/device/controllerUtil.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

std::shared_ptr<OdomChassisController> chassis;
std::shared_ptr<okapi::XDriveModel> driveTrain;

lv_obj_t * myButton;
lv_obj_t * myButtonLabel;
lv_obj_t * myLabel;

lv_style_t myButtonStyleREL; //relesed style
lv_style_t myButtonStylePR; //pressed style

static lv_res_t btn_click_action(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn); //id usefull when there are multiple buttons

    if(id == 0)
    {
        char buffer[100];
		sprintf(buffer, "button was clicked %i milliseconds from start", pros::millis());
		lv_label_set_text(myLabel, buffer);
    }

    return LV_RES_OK;
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
#define fly_mtr_prt 1
void initialize() {

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
            .withMotors(-20,11,3,-2) //Top Left, Top Right, Bottom Right, Bottom Left //16 combos because 2^4 
            .withDimensions({AbstractMotor::gearset::green,(60.0/84.0)}, {{4_in, 26.5_in}, imev5GreenTPR}) //Track length, Gearing
            .withMaxVelocity(100)
            .withSensors(
                ADIEncoder{'A','B'},
                ADIEncoder{'C','D', false},
                ADIEncoder{'E','F'}
            )
            .withOdometry({{2.75_in, 7.5_in, 9_in, 2.75_in}, quadEncoderTPR}) //quadEncoderTPR=fixed variable representing the ticks per rotation of the red v1 potentiometers 
            .buildOdometry();

    chassis->setState({0_in, 0_in, 0_deg}); //todo: to be changed to a configurable value depending on the starting position on the pitch 

    driveTrain = std::dynamic_pointer_cast<XDriveModel>(chassis->getModel()); //switch to ThreeEncoderXDriveModel???
    // assigning the chassis to a Three Encoder X-drive model
    driveTrain->setBrakeMode(AbstractMotor::brakeMode::hold);
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

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
	// pros::Controller master(pros::E_CONTROLLER_MASTER);
	// pros::Motor left_mtr(1);
	// pros::Motor right_mtr(2);
	// pros::Motor fly_mtr(9);

    //Controller controller;
    pros::Controller controller_master (pros::E_CONTROLLER_MASTER);
    okapi::Controller controller_okapi = Controller();

    
    while (true) {
        //Tank drive with left and right sticks
        //drive->setState({0_in,0_in,0_deg});
        //drive->driveToPoint({0_ft,3_ft});

        //read the input from the controller sticks
        //int controller_turn = controller_master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X); //this uses PROS
        double controller_turn = controller_okapi.getAnalog(ControllerAnalog::leftX); //this uses okapilib - not sure about the differences
        double controller_forward = controller_okapi.getAnalog(ControllerAnalog::rightY); //todo: implement s curve for the controller
        double controller_strife = controller_okapi.getAnalog(ControllerAnalog::rightX);
        driveTrain->xArcade(controller_strife,controller_forward, controller_turn); //should be used for degraded control when odom is off - voltage mode, note - +1 =1 is max min
        //double QAngle = controller_okapi->getPose().theta.convert(degree);
        //lv_label_set_text(myLabel, "controller turn %d",controller_turn);
        char to_be_printed[100];
        //std::string to_be_printed = ("forward input %f,turn input %f, strife input %f",controller_forward,controller_turn,controller_strife);
        sprintf(to_be_printed,"forward input %f,turn input %f, strife input %f",controller_forward,controller_turn,controller_strife);
        lv_label_set_text(myLabel, to_be_printed);
        //driveTrain->fieldOrientedXArcade(controller_strife,controller_forward, controller_turn,chassis->getState().theta);// you can use an IMU as an alternative for the QAngle //chassis->getPose().theta.convert(degree)
        //driveTrain->fieldOrientedXArcade(controller_strife,controller_forward, controller_turn,chassis->getState().theta);// you can use an IMU as an alternative for the QAngle //chassis->getPose().theta.convert(degree)
        
        okapi::OdomState current_state = chassis->getState();
        //controller.setText(0, 0, "x %f,y %f, theta %f",current_state.x,current_state.y,current_state.theta);
        //controller_master.print(0, 0, "x %f,y %f, theta %f",current_state.x,current_state.y,current_state.theta);
        controller_master.print(0, 0, current_state.str().c_str());
        //std::string to_be_printed = ("x %f,y %f, theta %f",current_state.x,current_state.y,current_state.theta);
        
        //good debugging - just swapping the screen output to debug something else
        //lv_label_set_text(myLabel, current_state.str().c_str());
        //controller_master.print(3, 3, "theta %f",current_state.theta);
        
        /*chassis->setMaxVelocity(20);
        chassis->driveToPoint({50_cm,0_cm});
        chassis->driveToPoint({50_cm,50_cm});
        chassis->driveToPoint({0_cm,50_cm});
        chassis->driveToPoint({0_cm,0_cm});*/

        pros::delay(20); //x ms delay
    }
}
