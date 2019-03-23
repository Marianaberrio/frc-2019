/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Calendar;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.sun.tools.javadoc.main.Start;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * TEAM FORCE 4707 PRACTICE ROBOT
 */
public class Robot extends TimedRobot implements RobotMap {
  private static OI oi;

  private DifferentialDrive drive;

  private long time1;
  private double Kp, Ki;
  private double eK = 0;
  private double setPoint;
  private boolean pidEnabled = false;
  private boolean handToggle = false;
  private boolean shotBallToggle = false;
  private boolean compresorToggle = false;
  private Ultrasonic ultrasonicSonar;
  private Timer myTimer;
  private Gyro gyro;

  private static final double MAX_VELOCITY = 0.5;

  @Override
  public void robotInit() {
    oi = OI.getInstace();
    oi.initSetup();
    drive = new DifferentialDrive(oi.leftFrontTalon, oi.rightFrontTalon);
    gyro = new AnalogGyro(1);
    gyro.calibrate();
    // DigitalInput input = new DigitalInput(2);
    // DigitalOutput output = new DigitalOutput(2);
    // ultrasonicSonar = new Ultrasonic(output, input);
    // ultrasonicSonar.setAutomaticMode(true);
  }

  @Override
  public void teleopInit() {
    eK = 0;
    Ki = 0.005;
    Kp = 0.025;
    setPoint = 0.0001;
    gyro.reset();
  }

  @Override
  public void autonomousInit() {
    myTimer = new Timer();
    myTimer.reset();
    myTimer.start();
  }

  @Override
  public void teleopPeriodic() {
    double velocity = oi.driverJoystick.getRawAxis(1) * 0.7;
    double rotation = oi.driverJoystick.getRawAxis(4) * 0.7;
    double upArmVelocity = oi.driverJoystick.getRawAxis(2) * MAX_VELOCITY;
    double downArmVelocity = oi.driverJoystick.getRawAxis(3) * MAX_VELOCITY;
    double armVelocity = 0;
    smoothDrive(velocity * (-1), rotation);
    SmartDashboard.putNumber("upArmVelocity ", upArmVelocity);
    SmartDashboard.putNumber("downArmVelocity ", downArmVelocity);
    // SmartDashboard.putNumber("RangeInces ", ultrasonicSonar.getRangeInches());

    long time2 = Calendar.getInstance().getTimeInMillis();
    double delta = (time2 - time1) / 1000.0;
    time1 = time2;
    double v = 0;

    int count = oi.armEncoder.get();
    double raw = oi.armEncoder.getRaw();
    double distance = oi.armEncoder.getDistance();
    double rate = oi.armEncoder.getRate();

    SmartDashboard.putNumber("count ", count);
    SmartDashboard.putNumber("Distance ", distance);
    SmartDashboard.putNumber("Raw value ", raw);
    SmartDashboard.putNumber("Rater ", rate);
    SmartDashboard.putBoolean("PID ", pidEnabled);
    SmartDashboard.putNumber("Angle", gyro.getAngle());

    if (oi.driverJoystick.getRawButtonPressed(BTN_BACK_AXIS)) {
      oi.armEncoder.reset();
      eK = 0;
    }

    if (oi.driverJoystick.getRawButtonPressed(BTN_Y_AXIS)) {
      compresorToggle = !compresorToggle;
      if (compresorToggle && !oi.mainCompressor.enabled())
        oi.initNeumatics();
      else
        oi.stopCompressor();
    }

    if (oi.driverJoystick.getRawButtonPressed(BTN_START_AXIS)) {
      initPIDValues();
    }

    if (oi.driverJoystick.getRawButtonPressed(BTN_R1_AXIS)) {
      moveHand(-0.7);
    } else if ((oi.driverJoystick.getRawButtonReleased(BTN_R1_AXIS))) {
      moveHand(0.0);
    }

    if (oi.driverJoystick.getRawButtonPressed(BTN_L1_AXIS)) {
      moveHand(0.7);
    } else if (oi.driverJoystick.getRawButtonReleased(BTN_L1_AXIS)) {
      moveHand(0.0);
    }

    if (upArmVelocity > 0d && (count > 20 || pidEnabled)) {
      moveArm(-upArmVelocity);
      armVelocity = -upArmVelocity;
    } else if (downArmVelocity > 0 && (count < 603 || pidEnabled)) {
      moveArm(downArmVelocity);
      armVelocity = downArmVelocity;
    } else {
      armVelocity = 0d;
      moveArm(armVelocity);
    }

    SmartDashboard.putNumber("ARM Velocity ", armVelocity);

    if (oi.driverJoystick.getRawButtonPressed(BTN_A_AXIS)) {
      toogleHand();
    }

    if (oi.driverJoystick.getRawButtonPressed(BTN_X_AXIS)) {
      shotBall();
    }
  }

  @Override
  public void autonomousPeriodic() {
    while (myTimer.get() < 3.0) {
      smoothDrive(0.6, 0.0);
    }
  }

  private void moveArm(double direction) {
    oi.armTalon.set(ControlMode.PercentOutput, direction);
    SmartDashboard.putNumber("ARM_direction ", direction);
  }

  private void moveHand(double direction) {
    oi.handTalon.set(ControlMode.PercentOutput, direction);
    SmartDashboard.putNumber("HAND_direction ", direction);
  }

  private void initPIDValues() {
    pidEnabled = !pidEnabled;
  }

  private void toogleHand() {
    handToggle = !handToggle;
    if (handToggle) {
      oi.hanSolenoid.set(Value.kForward);
    } else {
      oi.hanSolenoid.set(Value.kReverse);
    }
  }

  private void shotBall() {
    if (oi.shooterSolenoid.get() == Value.kForward) {
      oi.shooterSolenoid.set(Value.kReverse);
    } else {
      oi.shooterSolenoid.set(Value.kForward);
    }
  }

  private void smoothDrive(double velocity, double rotation) {
    if (Math.abs(velocity) < 0.05) {
      velocity = 0;
    }
    if (Math.abs(rotation) < 0.05) {
      rotation = 0;
    }

    if (velocity != 0 && rotation == 0) {
      drive.tankDrive(velocity, velocity);
    } else {
      if (rotation != 0 && velocity == 0) {
        drive.tankDrive(-rotation, rotation);
      } else {
        drive.curvatureDrive(velocity, -rotation, false);
      }
    }
  }

}
