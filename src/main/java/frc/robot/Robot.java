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
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * TEAM FORCE 4707 PRACTICE ROBOT
 */
public class Robot extends TimedRobot implements RobotMap {
  private static OI oi;

  private DifferentialDrive drive;
  private AnalogInput frontSonar;

  private boolean asistanceEnabled = false;
  private boolean handToggle = false;
  private boolean compresorToggle = false;
  private boolean isDiskLevelEnable = false;
  private Timer robotTimer;
  private int armEncoderCount;
  private int currentLevel;

  private static final double MAX_VELOCITY = 0.5;
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
    drive = new DifferentialDrive(oi.leftFrontTalon, oi.rightFrontTalon);
    frontSonar = new AnalogInput(4);
    frontSonar.setAccumulatorInitialValue(0);

    robotTimer = new Timer();
    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(640, 480);
      
      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
      
      Mat source = new Mat();
      Mat output = new Mat();
      
      while(!Thread.interrupted()) {
          cvSink.grabFrame(source);
          Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
          outputStream.putFrame(output);
      }
  }).start();
  }

  @Override
  public void teleopInit() {
    setInitialPosition();
    robotTimer.reset();
    robotTimer.start();
  }

  @Override
  public void autonomousInit() {
    setInitialPosition();
    robotTimer.reset();
    robotTimer.start();
  }

  @Override
  public void autonomousPeriodic() {
    while (robotTimer.get() < 5.0) {
      smoothDrive(0.4, 0.0);
    }
  }

  @Override
  public void teleopPeriodic() {
    double velocity = oi.driverJoystick.getRawAxis(1) * 0.7;
    double rotation = oi.driverJoystick.getRawAxis(4) * 0.7;
    double upArmVelocity = oi.driverJoystick.getRawAxis(2) * MAX_VELOCITY_ARM;
    double downArmVelocity = oi.driverJoystick.getRawAxis(3) * MAX_VELOCITY;
    double armVelocity = 0;
    smoothDrive(velocity * (-1), rotation);
    SmartDashboard.putBoolean("LimitSwitchActice", oi.limitSwitch.get());

    armEncoderCount = oi.armEncoder.get();

    SmartDashboard.putNumber("count ", oi.armEncoder.get());
    SmartDashboard.putNumber("Distance ", oi.armEncoder.getDistance());
    SmartDashboard.putBoolean("ArmControllerEnabled ", asistanceEnabled);

    SmartDashboard.putNumber("count Right ", oi.rightEncoder.get());
    SmartDashboard.putNumber("Distance Right ", oi.rightEncoder.getDistance());

    SmartDashboard.putNumber("count Left ", oi.leftEncoder.get());
    SmartDashboard.putNumber("Distance Left ", oi.leftEncoder.getDistance());

    SmartDashboard.putNumber("Sonar volts ", frontSonar.getVoltage());
    SmartDashboard.putNumber("Sonar avg Volt ", frontSonar.getAverageVoltage());
    SmartDashboard.putNumber("Raw avg sonar ", frontSonar.getAverageValue());
    SmartDashboard.putNumber("Raw value sonar ", frontSonar.getValue());
    // SmartDashboard.putNumber("distance in cm", (frontSonar.getAverageVoltage()/1024d));
    SmartDashboard.putNumber("distance Inches ", getDistanceSonar());

    if (oi.driverJoystick.getRawButtonPressed(BTN_BACK_AXIS)) {
      resetArmEncoder();
    }

    if (oi.driverJoystick.getRawButtonPressed(BTN_Y_AXIS)) {
      compresorToggle = !compresorToggle;
      if (compresorToggle && !oi.mainCompressor.enabled())
        oi.initNeumatics();
      else
        oi.stopCompressor();
    }
    SmartDashboard.putBoolean("LimitSwitchActice", oi.limitSwitch.get());

    if (oi.driverJoystick.getRawButtonPressed(BTN_START_AXIS)) {
      asistanceEnabled = !asistanceEnabled;
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

    SmartDashboard.putNumber("ARM Velocity ", armVelocity);

    if (oi.driverJoystick.getRawButtonPressed(BTN_A_AXIS)) {
      toogleHand();
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
    SmartDashboard.putString("LevelActivated", isDiskLevelEnable ? "Disk" : "Ball");
    if (oi.asistantJoystick.getRawButtonPressed(BTN_X_AXIS)) {
      setArmBallToLevel(LEVEL_1);
    }
    if (oi.asistantJoystick.getRawButtonPressed(BTN_Y_AXIS)) {
      setArmBallToLevel(LEVEL_2);
    }
    if (oi.asistantJoystick.getRawButtonPressed(BTN_B_AXIS)) {
      setArmBallToLevel(LEVEL_3);
    }

    if ((oi.limitSwitch.get() && upArmVelocity > 0d)) {
      moveArm(0.0);
      resetArmEncoder();
    } else if (oi.limitSwitch.get()) {
      resetArmEncoder();
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

  private void toogleHand() {
    handToggle = !handToggle;
    if (handToggle) {
      oi.hanSolenoid.set(Value.kForward);
    } else {
      oi.hanSolenoid.set(Value.kReverse);
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

  private void resetArmEncoder() {
    oi.armEncoder.reset();
  }

  private void setInitialPosition() {
    while (!oi.limitSwitch.get()) {
      moveArm(-0.4);
    }
    if (oi.limitSwitch.get()) {
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
    double valueToInches_2 = 1 / 20.5;// 14.45
    double distanceX = frontSonar.getAverageValue();
    double distance = (distanceX - 237) * valueToInches_2 + 12;
    int distanceInt=(int) distance;
    return distanceInt;
  }
}
