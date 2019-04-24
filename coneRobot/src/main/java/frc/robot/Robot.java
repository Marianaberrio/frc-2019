/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.OpenHand;
import frc.robot.commands.SetArmSetPoint;
import frc.robot.subsystems.Driver;

/**
 * TEAM FORCE 4707.
 */
public class Robot extends TimedRobot implements RobotMap {
  private static OI oi;
  private boolean elevateRobotToggle, wheelsUpToggle, balanceHelperToggle, gearChangeToggle;

  // private SpeedControllerGroup xspeedControllerDriveRight;
  // private SpeedControllerGroup speedControllerDriveLeft;
  // private DifferentialDrive drive;

  private boolean isManualArmEnabled = false;
  private boolean asistanceEnabled = false;
  private boolean handToggle = false;
  private boolean isDiskLevelEnable = false;
  private int armEncoderCount;
  private int currentLevel;

  DriveWithJoysticks driveWithJoysticks;
  Command m_teleOpCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

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
    oi = OI.getInstace();
    oi.initSetup();
    // oi.initTalons();
    // oi.initNeumatics();
    // oi.initEncoders();
    m_chooser.setDefaultOption("Default Auto", new DriveWithJoysticks());
    SmartDashboard.putData("Auto mode", m_chooser);
  }

  @Override
  public void autonomousInit() {
    if (m_teleOpCommand != null) {
      m_teleOpCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {
    m_teleOpCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_teleOpCommand != null) {
      m_teleOpCommand.start();
    }
    Scheduler.getInstance().add(new SetArmSetPoint());
  }

  @Override
  public void teleopPeriodic() {
    double velocity = oi.driverJoystick.getRawAxis(1);
    double rotation = oi.driverJoystick.getRawAxis(4);
    // driveRobot(velocity, rotation);
    Scheduler.getInstance().run();
    // if (oi.asistantJoystick.getRawButton(BTN_A_AXIS)){
    //   Scheduler.getInstance().add(new SetArmSetPoint());
    // }
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
      oi.drive.tankDrive(velocity, velocity);
      setDriveWheels(true);
    } else {
      if (rotation != 0 || velocity != 0) {
        setDriveWheels(false);
      }
      if (rotation != 0 && velocity == 0) {
        oi.drive.tankDrive(-rotation, rotation);
      } else {
        oi.drive.curvatureDrive(velocity, -rotation, false);
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

  private void toogleHandFWD() {
    handToggle = !handToggle;
    if (handToggle) {
      moveHand(0.7);
    } else {
      moveHand(0.0);
    }
  }

  private void toogleHandRVS() {
    handToggle = !handToggle;
    if (handToggle) {
      moveHand(-0.7);
    } else {
      moveHand(0.0);
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
      }
    } else {
      while (armEncoderCount > count) {
        armEncoderCount = oi.armEncoder.get();
        moveArm(-0.4);
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

  private double getDistanceSonar() {
    double valueToInches_2 = 1 / 20.5;
    double distanceX = oi.frontSonar.getAverageValue();
    double distance = (distanceX - 237) * valueToInches_2 + 12;
    int distanceInt = (int) distance;
    return distanceInt;
  }
}
