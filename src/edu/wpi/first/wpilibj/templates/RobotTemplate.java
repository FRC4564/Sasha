/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationEnhancedIO;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Dashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Encoder;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends SimpleRobot {

    /*** Declare Constants ***/
    
    // Right Joystick Button assignments
    public static final int JB_WINCH_UP   = 3;
    public static final int JB_WINCH_DOWN = 2;
    public static final int JB_WINCH_RESET = 6;
    
    public static final int JB_THROWER_FAST = 4;
    public static final int JB_THROWER_SLOW = 5;
    
    public static final int JB_CAM_FORWARD  = 1;
    public static final int JB_CAM_BACKWARD = 7;
    
    public static final int JB_SOLENOID_1 = 4;
    public static final int JB_SOLENOID_2 = 3;
    public static final int JB_SOLENOID_3 = 5;
    
    // Left Joystick Buttons
    public static final int JB_TUBE_LEFT = 8;
    public static final int JB_TUBE_RIGHT = 9;
    // PWM assingments
    public static final int PWM_DRIVE_FR = 1;
    public static final int PWM_DRIVE_RR = 2;
    public static final int PWM_DRIVE_RL = 3;
    public static final int PWM_DRIVE_FL = 4;
    public static final int PWM_THROWER  = 5;
    public static final int PWM_CAM      = 6;

    public static final int RLY_WINCH    = 8;
    public static final int RLY_CMP_PUMP = 7;
    public static final int RLY_TUBE_LEFT = 5;
    public static final int RLY_TUBE_RIGHT = 6;
    
    // Motor speed constants
    public static final double SPD_THROWER_FAST = 1.0;
    //public static final double SPD_THROWER_SLOW = 0.8;
    public static final double SPD_THROWER_SLOW = 0.5;

    //public static final double SPD_CAM_FORWARD  = -0.6;
    public static final double SPD_CAM_FORWARD  = -1.0;
    public static final double SPD_CAM_BACKWARD =  0.6;
    
    // DIO assignments
    public static final int DIO_WINCH_LOWER_LIMIT = 1;
    public static final int DIO_WINCH_UPPER_LIMIT = 2;
    public static final int DIO_CAM_LIMIT = 3;
    public static final int DIO_CMP_SWITCH = 5;  
    public static final int DIO_ROTARY_A = 6;
    public static final int DIO_ROTARY_B = 7;
    // limit switches are 'false' when reached
    public static final boolean WINCH_LIMIT_REACHED = false;
    public static final boolean CAM_LIMIT_REACHED = false;
    
    // main loop frequency control in seconds
    public static final double LOOP_DELAY_SECS = 0.01;
    
    /*** Public Variables ***/
    
    // Initiates the state of the cam switch
    public boolean prevState = false;
    // Tracks state fo cam
    public boolean camRunning;
    // contains the speed for the cam to run at
    public double camSpeed;
    // States if the winch is setting to angle
    public boolean winchReseting = false;
    // States if the winch is going to the angle
    public boolean winchDown = false;
    //
    public double winchTimer = 0;
    
   public double motorVal = 0;
    
    /*** Initialize robot components ***/
    RobotDrive chassis = new RobotDrive(PWM_DRIVE_FL,PWM_DRIVE_RL,PWM_DRIVE_FR,PWM_DRIVE_RR);
    
    Joystick leftstick  = new Joystick(1);
    Joystick rightstick = new Joystick(2);
    
    //SpeedController winch   = new Victor(PWM_WINCH);
    SpeedController thrower = new Jaguar(PWM_THROWER);
    SpeedController cam     = new Jaguar(PWM_CAM);
    Relay winch = new Relay(RLY_WINCH);
    //winch.setDirection(Relay.Direction.kBothDirections);
    
    //Relays for t-shirt tube launchers
    Relay tube1 = new Relay(RLY_TUBE_LEFT);
    Relay tube2 = new Relay(RLY_TUBE_RIGHT);
    
    DigitalInput upperLimit = new DigitalInput(DIO_WINCH_UPPER_LIMIT);
    DigitalInput lowerLimit = new DigitalInput(DIO_WINCH_LOWER_LIMIT);
    DigitalInput camLimit   = new DigitalInput(DIO_CAM_LIMIT);
    
    Encoder encoder = new Encoder(DIO_ROTARY_A, DIO_ROTARY_B);
    
    Solenoid[] solenoids = new Solenoid[8];
    
    AnalogChannel potentiometer = new AnalogChannel(1);
    
    /*** Attach to the driver station ***/
    DriverStation ds = DriverStation.getInstance();
    DriverStationEnhancedIO dseio = DriverStation.getInstance().getEnhancedIO();
    SmartDashboard DashData = new SmartDashboard();
    
    Compressor compressor = new Compressor(DIO_CMP_SWITCH, RLY_CMP_PUMP);
    
    
    /**
     * This code is called once, when the robot is powered on.
     */
    protected void robotInit() {
        // Motor inversion is used by the arcadeDrive method
        // This does not impact the drive method
        chassis.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        chassis.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
        chassis.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        chassis.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
        compressor.start();
        for (int i = 0; i < 8; i++) {
                solenoids[i] = new Solenoid(i + 1);
        }
        solenoids[0].set(true);
        encoder.start();
        encoder.reset();


    }
    
    
     public void fireTube1() {
        if (leftstick.getRawButton(JB_TUBE_LEFT)){
            tube1.set(Relay.Value.kForward);
        } else {
            tube1.set(Relay.Value.kOff);
        }
    }
     
     public void fireTube2() {
        if (leftstick.getRawButton(JB_TUBE_RIGHT)){
            tube2.set(Relay.Value.kForward);
        } else {
            tube2.set(Relay.Value.kOff);
        }
    }

     
     /** If limit switch is clear, move winch at speed provided. Use SPD_WINCH_*
     * constants, which incorporate speed and direction for motor.
     */

    public void winchUp() {
        if (upperLimit.get() != WINCH_LIMIT_REACHED) {
            winch.set(Relay.Value.kReverse);
        } else {
            winch.set(Relay.Value.kOff);
        }
    }
    
    public void winchDown() {
        if (lowerLimit.get() != WINCH_LIMIT_REACHED) {
            winch.set(Relay.Value.kForward);
        } else {
            winch.set(Relay.Value.kOff);
        }
    }
    /** Read joystick buttons and stop or move winch, as appropriate,
     *  and with regard to limit switches
     */
    public void controlWinch() {
        if (rightstick.getRawButton(JB_WINCH_UP)) {
            winchUp();
            winchReseting = false;
        } else if (rightstick.getRawButton(JB_WINCH_DOWN)) {
            winchDown();
            winchReseting = false;
        } else if (rightstick.getRawButton(JB_WINCH_RESET)) {
            winchReseting = true;
        } else {
            winch.set(Relay.Value.kOff);  //stop winch
        }
        if (winchReseting == true) {
            if (upperLimit.get() != WINCH_LIMIT_REACHED 
               && winchDown == false
               && lowerLimit.get() != WINCH_LIMIT_REACHED) {
                winchUp();
            } else if (upperLimit.get() == WINCH_LIMIT_REACHED
                      && winchDown == false
                      && lowerLimit.get() != WINCH_LIMIT_REACHED) {
                winchDown = true;
                winchTimer = ds.getAnalogIn(1);
            } else if (winchDown == true
                      && winchTimer > 0
                      && lowerLimit.get() != WINCH_LIMIT_REACHED) {
                winchDown();
                winchTimer = winchTimer - LOOP_DELAY_SECS;
                if (winchTimer <= 0) {
                    winchReseting = false;
                    winchDown = false;
                }
            }
        }
    }

    /** Read joystick buttons and stop or spin thrower motor at selected speed */
    public void controlThrower() {
        if (rightstick.getRawButton(JB_THROWER_SLOW)) {
            thrower.set(SPD_THROWER_SLOW);
        } else if (rightstick.getRawButton(JB_THROWER_FAST)) {
            //thrower.set(SPD_THROWER_FAST);
            thrower.set(((-1.0 * rightstick.getZ()) + 1.0 )/ 2.0);
            //motorVal = (potentiometer.getValue() - 12) / 956.0;
        } else {
            thrower.set(0);
        }
}
    
    /** Read joystick buttons and initiate or re-initiate cam rotation.  Cam will
     * be set to run for the number of seconds that are read from 'analog input 1' on
     * the driver station.  When timer reaches zero (cam_Timer), the cam is stopped.
     * Each call to this routine will decrement the cam_Timer based on the main loop
     * frequency.
     */ 
    public void controlCam() {
        if (rightstick.getRawButton(JB_CAM_FORWARD)) {
            camRunning = true;
            camSpeed = SPD_CAM_FORWARD;
        } else if (rightstick.getRawButton(JB_CAM_BACKWARD)) {
            camRunning = true;
            camSpeed = SPD_CAM_BACKWARD;
        }
        
        if (camRunning == true) {
            cam.set(camSpeed);
            SmartDashboard.putNumber("Cam Velocity", camSpeed);
        } else {
            cam.set(0);
            SmartDashboard.putNumber("Cam Velocity", 0);
        }
        if (prevState != CAM_LIMIT_REACHED && camLimit.get() == CAM_LIMIT_REACHED) {
            camRunning = false;
        }
        prevState = camLimit.get();

    }
    public void solenoidTest() {

        if (leftstick.getRawButton(3)) {
            solenoids[0].set(true);
        } else {
            solenoids[0].set(false);
            }
        if (leftstick.getRawButton(4)) {
            solenoids[1].set(true);
        } else {
            solenoids[1].set(false);
            }
        if (leftstick.getRawButton(5)) {
            solenoids[2].set(true);
        } else {
            solenoids[2].set(false);
            }
        }
    
    public void encoderTest() {
        if (rightstick.getRawButton(8)) {
            encoder.start();
            encoder.reset();
        } else if (rightstick.getRawButton(9)) {
            encoder.stop();
        }
        
    }
    /**
     * This function is called each time autonomous mode is enabled from driver station.
     */
    public void autonomous() {
        chassis.setSafetyEnabled(false);
            //thrower.set(SPD_THROWER_FAST);
            //cam.set(SPD_CAM_FORWARD);
        compressor.start();
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        chassis.setSafetyEnabled(false);
        while(isOperatorControl() && isEnabled()) {
            chassis.arcadeDrive(leftstick);
            //chassis.arcadeDrive(leftstick.getY()*0.7,leftstick.getX()*0.8);
            controlWinch();
            controlThrower();
            controlCam();
            SmartDashboard.putNumber("Potentiometer", potentiometer.getValue());
            SmartDashboard.putNumber("Motor value", motorVal);
            SmartDashboard.putNumber("Encoder", encoder.get());
            solenoidTest();
            encoderTest();
            fireTube1();
            fireTube2();
            Timer.delay(LOOP_DELAY_SECS);
        }
    }
    
    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
    
    }
}
