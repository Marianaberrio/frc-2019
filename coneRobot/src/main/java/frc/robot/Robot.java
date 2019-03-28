/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Calendar;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * TEAM FORCE 4707.
 */
public class Robot extends TimedRobot implements RobotMap {
  private static OI oi;
  private boolean elevateRobotToggle, wheelsUpToggle, balanceHelperToggle, gearChangeToggle;

  private SpeedControllerGroup speedControllerDriveRight;
  private SpeedControllerGroup speedControllerDriveLeft;
  private DifferentialDrive drive;

  private long time1;
  private double Kp, Ki;
  private double eK = 0;
  private double setPoint;
  private boolean pidEnabled = false;

  private static final double MAX_VELOCITY = 0.8;

  @Override
  public void robotInit() {
    oi = OI.getInstace();
    oi.initTalons();
    oi.initNeumatics();
    oi.initEncoders();
  }

  @Override
  public void teleopInit() {
    speedControllerDriveRight = new SpeedControllerGroup(oi.rightFrontTalon, oi.rightSlaveTalon);
    speedControllerDriveLeft = new SpeedControllerGroup(oi.leftFrontTalon, oi.leftSlaveTalon);
    drive = new DifferentialDrive(speedControllerDriveLeft, speedControllerDriveRight);

    eK = 0;
    Ki = 0.005;
    Kp = 0.025;
    setPoint = 0.0001;
    setDriveWheels(true);
  }

  @Override
  public void testPeriodic() {
    super.testPeriodic();
  }

  @Override
  public void teleopPeriodic() {
    double velocity = oi.driverJoystick.getRawAxis(1);
    double rotation = oi.driverJoystick.getRawAxis(4);
    // drive.tankDrive(leftVelovity, rightVelocity);
    smoothDrive(velocity, rotation);

    if (oi.driverJoystick.getRawButtonPressed(BTN_A_AXIS)) {
      elevateRobot();
    }

    if (oi.driverJoystick.getRawButtonPressed(BTN_B_AXIS)) {
      wheelsUp();
    }

    if (oi.driverJoystick.getRawButtonPressed(BTN_X_AXIS))
      launchBalanceHelper();

    // if (velocity == 0.0 && rotation == 0.0){
    if (oi.driverJoystick.getRawButtonPressed(BTN_Y_AXIS))
      changeGear();
    // }

    SmartDashboard.putNumber("ARM Velocity ", oi.asistantJoystick.getRawAxis(1));
    moveArm(-oi.asistantJoystick.getRawAxis(1));
    moveHand(oi.asistantJoystick.getRawAxis(5));

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

    if (oi.asistantJoystick.getRawButtonPressed(BTN_BACK_AXIS)) {
      oi.armEncoder.reset();
      eK = 0;
    }
    boolean limitSwitchValue = oi.limitSwitchArm.get();
    SmartDashboard.putBoolean("LimitSwitch ", limitSwitchValue);

    if (oi.asistantJoystick.getRawButtonPressed(BTN_START_AXIS)) {
      initPIDValues();
    }

    if (pidEnabled) {
      // PID
      // Leer el setpoint
      double e_k = setPoint - raw;
      double e_P = e_k;
      double e_I = eK + e_k * delta;

      v = Kp * e_P + Ki * e_I;

      v = Math.min(Math.max(-MAX_VELOCITY, v), MAX_VELOCITY);
      eK = e_I;
      oi.armTalon.set(ControlMode.PercentOutput, v);
      SmartDashboard.putNumber("Velocity ", v);
      SmartDashboard.putNumber("Integrator-e ", eK);
    }

  }

  private void elevateRobot() {
    elevateRobotToggle = !elevateRobotToggle;
    if (elevateRobotToggle)
      oi.elevateRobotSolenoid.set(Value.kForward);
    else
      oi.elevateRobotSolenoid.set(Value.kReverse);
  }

  private void wheelsUp() {
    wheelsUpToggle = !wheelsUpToggle;
    if (wheelsUpToggle)
      oi.wheelSolenoid.set(Value.kForward);
    else
      oi.wheelSolenoid.set(Value.kReverse);
  }

  private void launchBalanceHelper() {
    balanceHelperToggle = !balanceHelperToggle;
    if (balanceHelperToggle)
      oi.balanceHelperSolenoid.set(Value.kForward);
    else
      oi.balanceHelperSolenoid.set(Value.kReverse);
  }

  private void changeGear() {
    gearChangeToggle = !gearChangeToggle;
    if (gearChangeToggle)
      oi.gearChangeSolenoid.set(Value.kReverse);
    else
      oi.gearChangeSolenoid.set(Value.kForward);
  }

  private void moveArm(double direction) {
    oi.armTalon.set(ControlMode.PercentOutput, direction);
  }

  private void moveHand(double direction) {
    oi.handTalon.set(ControlMode.PercentOutput, direction);
  }

  private void initPIDValues() {
    pidEnabled = !pidEnabled;
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
      setDriveWheels(true);
    } else {
      if (rotation != 0 || velocity != 0) {
        setDriveWheels(false);
      }
      if (rotation != 0 && velocity == 0) {
        drive.tankDrive(-rotation, rotation);
      } else {
        drive.curvatureDrive(velocity, -rotation, false);
      }
    }
  }

  private void setDriveWheels(boolean wheelsUp) {
    if (wheelsUp)
      oi.wheelSolenoid.set(Value.kForward);
    else
      oi.wheelSolenoid.set(Value.kReverse);
    wheelsUpToggle = wheelsUp;
  }

}
