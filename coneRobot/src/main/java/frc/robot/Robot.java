/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * TEAM FORCE 4707.
 */
public class Robot extends TimedRobot implements RobotMap {
  private static IO oi;
  private boolean elevateRobotToggle, wheelsUpToggle, balanceHelperToggle, gearChangeToggle;

  private SpeedControllerGroup speedControllerDriveRight;
  private SpeedControllerGroup speedControllerDriveLeft;
  private DifferentialDrive drive;

  private boolean isManualArmEnabled = false;
  private boolean asistanceEnabled = false;
  private boolean handToggle = false;
  private boolean compresorToggle = false;
  private boolean isDiskLevelEnable = false;
  private int armEncoderCount;
  private int currentLevel;
  
  private static final double MAX_VELOCITY = 0.8;
  private static final double MAX_VELOCITY_ARM = 0.3;

  private static final int grabLevelBall = 5;
  private static final int firstLevelBall = 202;
  private static final int secondLevelBall = 455;
  private static final int thirdLevelBall = 604;

  private static final int grabLevelDisk = 20;
  private static final int firstLevelDisk = 47;
  private static final int secondLevelDisk = 290;
  private static final int thirdLevelDisk = 565;

  private static final int LEVEL_0 = 0;
  private static final int LEVEL_1 = 1;
  private static final int LEVEL_2 = 2;
  private static final int LEVEL_3 = 3;
  
  @Override
  public void robotInit() {
    oi = IO.getInstace();
    oi.initTalons();
    oi.initNeumatics();
    oi.initEncoders();
  }

  @Override
  public void teleopInit() {
    speedControllerDriveRight = new SpeedControllerGroup(oi.rightFrontTalon, oi.rightSlaveTalon);
    speedControllerDriveLeft = new SpeedControllerGroup(oi.leftFrontTalon, oi.leftSlaveTalon);
    drive = new DifferentialDrive(speedControllerDriveLeft, speedControllerDriveRight);
    setDriveWheels(true);
  }

  @Override
  public void testPeriodic() {
    super.testPeriodic();
  }

  @Override
  public void teleopPeriodic() {
    double velocity = oi.driverJoystick.getRawAxis(1) * MAX_VELOCITY;
    double rotation = oi.driverJoystick.getRawAxis(4) * MAX_VELOCITY;
    driveRobot(velocity, rotation);
    double upArmVelocity = oi.driverJoystick.getRawAxis(2) * MAX_VELOCITY_ARM;
    double downArmVelocity = oi.driverJoystick.getRawAxis(3) * MAX_VELOCITY;
    double armVelocity = 0;
    boolean armLimitSwitchActivated = oi.limitSwitchArm.get();
    armEncoderCount = oi.armEncoder.get();

    if (oi.driverJoystick.getRawButtonPressed(BTN_A_AXIS)) {
      elevateRobot();
    }

    if (oi.driverJoystick.getRawButtonPressed(BTN_B_AXIS)) {
      wheelsUp();
    }

    if (oi.driverJoystick.getRawButtonPressed(BTN_Y_AXIS)){
      changeGear();
    }

    if (oi.driverJoystick.getRawButtonPressed(BTN_R1_AXIS)) {
      moveHand(0.7);
    } else if ((oi.driverJoystick.getRawButtonReleased(BTN_R1_AXIS))) {
      moveHand(0.0);
    }

    if (oi.driverJoystick.getRawButtonPressed(BTN_L1_AXIS)) {
      moveHand(0.0);
    } else if ((oi.driverJoystick.getRawButtonReleased(BTN_L1_AXIS))) {
      moveHand(-0.7);
    }

    if (upArmVelocity > 0d && (armEncoderCount > 15 || asistanceEnabled)) {
      moveArm(-upArmVelocity);
      armVelocity = -upArmVelocity;
    } else if (downArmVelocity > 0 && (armEncoderCount < 603 || asistanceEnabled)) {
      moveArm(downArmVelocity);
      armVelocity = downArmVelocity;
    } else {
      armVelocity = 0d;
      moveArm(armVelocity);
    }
    
    // Set level of the arm base on the X btn and the values
    if (oi.driverJoystick.getRawButtonPressed(BTN_X_AXIS)) {
      if (isDiskLevelEnable) {
        oi.hanSolenoid.set(Value.kReverse);
        moveHand(0.0);
      } else {
        oi.hanSolenoid.set(Value.kForward);
        moveHand(-0.7);
      }
      setArmBallToLevel(LEVEL_0);
    }
    //Assistance controls
    // take the disk from the cargo
    if (oi.asistantJoystick.getRawButtonPressed(BTN_BACK_AXIS)) {
      grabDisk();
    }
    if (oi.asistantJoystick.getRawButtonPressed(BTN_START_AXIS)) {
      putDisk();
    }

    if (oi.asistantJoystick.getRawButtonPressed(BTN_A_AXIS)) {
      isDiskLevelEnable = !isDiskLevelEnable;
      if (!isDiskLevelEnable)
        oi.hanSolenoid.set(Value.kForward);

    }
    if (oi.asistantJoystick.getRawButtonPressed(BTN_X_AXIS)) {
      setArmBallToLevel(LEVEL_1);
    }
    if (oi.asistantJoystick.getRawButtonPressed(BTN_Y_AXIS)) {
      setArmBallToLevel(LEVEL_2);
    }
    if (oi.asistantJoystick.getRawButtonPressed(BTN_B_AXIS)) {
      setArmBallToLevel(LEVEL_3);
    }

    SmartDashboard.putString("LevelActivated", isDiskLevelEnable ? "Disk" : "Ball");
    SmartDashboard.putBoolean("LimitSwitch ", armLimitSwitchActivated);
    SmartDashboard.putNumber("count ", armEncoderCount);
    SmartDashboard.putBoolean("ARM Manual ", isManualArmEnabled);
    SmartDashboard.putNumber("ARM Velocity ", oi.asistantJoystick.getRawAxis(1));
    SmartDashboard.putNumber("distance Inches ", getDistanceSonar());
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
      oi.raptorWheelSolenoid.set(Value.kForward);
    else
      oi.raptorWheelSolenoid.set(Value.kReverse);
  }

  private void changeGear() {
    gearChangeToggle = !gearChangeToggle;
    if (gearChangeToggle)
      oi.changeGearSolenoid.set(Value.kReverse);
    else
      oi.changeGearSolenoid.set(Value.kForward);
  }

  private void moveArm(double direction) {
    oi.armTalon.set(ControlMode.PercentOutput, direction);
  }

  private void moveHand(double direction) {
    oi.handTalon.set(ControlMode.PercentOutput, direction);
  }

  private void toggleManualArm() {
    isManualArmEnabled = !isManualArmEnabled;
  }

  private void driveRobot(double velocity, double rotation) {
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
      oi.raptorWheelSolenoid.set(Value.kForward);
    else
      oi.raptorWheelSolenoid.set(Value.kReverse);
    wheelsUpToggle = wheelsUp;
  }

  private void toogleHand() {
    handToggle = !handToggle;
    if (handToggle) {
      oi.hanSolenoid.set(Value.kForward);
    } else {
      oi.hanSolenoid.set(Value.kReverse);
    }
  }

  private void resetArmEncoder() {
    oi.armEncoder.reset();
  }

  private void setInitialPosition() {
    while (!oi.limitSwitchArm.get()) {
      moveArm(-0.4);
    }
    if (oi.limitSwitchArm.get()) {
      resetArmEncoder();
    }
  }

  private void setArmBallToLevel(int level) {
    int count;
    switch (level) {
    case LEVEL_0: {
      count = isDiskLevelEnable ? grabLevelDisk : grabLevelBall;
      break;
    }
    case LEVEL_1: {
      count = isDiskLevelEnable ? firstLevelDisk : firstLevelBall;
      break;
    }
    case LEVEL_2: {
      count = isDiskLevelEnable ? secondLevelDisk : secondLevelBall;
      break;
    }
    case LEVEL_3: {
      count = isDiskLevelEnable ? thirdLevelDisk : thirdLevelBall;
      break;
    }
    default:
      count = 10;
    }
    currentLevel = count;
    setArmToPosition(count);
  }

  private void setArmToPosition(int count) {
    // int lastCounter = 0;
    Timer movementTimout = new Timer();
    movementTimout.start();
    if (armEncoderCount < count) {
      while (armEncoderCount < count) {
        armEncoderCount = oi.armEncoder.get();
        moveArm(0.5);
        // if (movementTimout.get() > 2.0 && lastCounter == armEncoderCount) {
        // setArmToPosition(lastCounter - 5);
        // break;
        // }
        // lastCounter = armEncoderCount;
      }
    } else {
      while (armEncoderCount > count) {
        armEncoderCount = oi.armEncoder.get();
        moveArm(-0.4);
        // lastCounter = armEncoderCount;
      }
    }
  }

  private void grabDisk() {
    setArmToPosition(currentLevel + 15);
    oi.hanSolenoid.set(Value.kForward);
  }

  private void putDisk() {
    int additionalFactor;
    if (currentLevel > 500)
      additionalFactor = 5;
    else if (currentLevel > 200)
      additionalFactor = 3;
    else
      additionalFactor = 1;
    oi.hanSolenoid.set(Value.kReverse);
    setArmToPosition(currentLevel - (15 + additionalFactor));
  }

  private double getDistanceSonar(){
    double valueToInches_2 = 1 / 20.5;
    double distanceX = oi.frontSonar.getAverageValue();
    double distance = (distanceX - 237) * valueToInches_2 + 12;
    int distanceInt=(int) distance;
    return distanceInt;
  }
}
