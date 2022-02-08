// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {
  /* doesn't work
  private final PWMTalonSRX m_frontLeft = new PWMTalonSRX(DriveConstants.kFrontLeftMotorPort);
  private final PWMTalonSRX m_rearLeft = new PWMTalonSRX(DriveConstants.kRearLeftMotorPort);
  private final PWMTalonSRX m_frontRight = new PWMTalonSRX(DriveConstants.kFrontRightMotorPort);
  private final PWMTalonSRX m_rearRight = new PWMTalonSRX(DriveConstants.kRearRightMotorPort);
  */
  
  private final WPI_TalonSRX motorFL = new WPI_TalonSRX(DriveConstants.kFrontLeftMotorID);
  private final WPI_TalonSRX motorRL = new WPI_TalonSRX(DriveConstants.kRearLeftMotorID);
  private final WPI_TalonSRX motorFR = new WPI_TalonSRX(DriveConstants.kFrontRightMotorID);
  private final WPI_TalonSRX motorRR = new WPI_TalonSRX(DriveConstants.kRearRightMotorID);

  private final MecanumDrive m_drive =
      new MecanumDrive(motorFL, motorRL, motorFR, motorRR);
  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    // DO WE NEED TO DO THIS?

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_drive.setSafetyEnabled(false);
    motorFR.setInverted(true);
    motorRR.setInverted(true);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new MecanumDriveWheelSpeeds(
            motorFL.getSelectedSensorVelocity()* (10.0 / 4096) * DriveConstants.kWheelCircumference,
            motorFL.getSelectedSensorVelocity()* (10.0 / 4096) * DriveConstants.kWheelCircumference,
            motorFL.getSelectedSensorVelocity()* (10.0 / 4096) * DriveConstants.kWheelCircumference,
            motorFL.getSelectedSensorVelocity()* (10.0 / 4096) * DriveConstants.kWheelCircumference));
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      m_drive.driveCartesian(ySpeed, xSpeed, rot, -m_gyro.getAngle());
    } else {
      m_drive.driveCartesian(ySpeed, xSpeed, rot);
    }
  }

  /** Sets the front left drive MotorController to a voltage. */
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    motorFL.setVoltage(volts.frontLeftVoltage);
    motorRL.setVoltage(volts.rearLeftVoltage);
    motorFR.setVoltage(volts.frontRightVoltage);
    motorRR.setVoltage(volts.rearRightVoltage);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    motorFL.setSelectedSensorPosition(0);
    motorRL.setSelectedSensorPosition(0);
    motorFR.setSelectedSensorPosition(0);
    motorRR.setSelectedSensorPosition(0);
  }

  /**
   * Gets the front left drive encoder.
   *
   * @return the front left drive encoder
   */
  /*
   public Encoder getFrontLeftEncoder() {
    return m_frontLeftEncoder;
  }
  */

  /**
   * Gets the rear left drive encoder.
   *
   * @return the rear left drive encoder
   */
  /*
  public Encoder getRearLeftEncoder() {
    return m_rearLeftEncoder;
  }
  */

  /**
   * Gets the front right drive encoder.
   *
   * @return the front right drive encoder
   */
  /*
  public Encoder getFrontRightEncoder() {
    return m_frontRightEncoder;
  }
  */

  /**
   * Gets the rear right drive encoder.
   *
   * @return the rear right encoder
   */
  /*
  public Encoder getRearRightEncoder() {
    return m_rearRightEncoder;
  }
  */

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      motorFL.getSelectedSensorVelocity()* (10.0 / 4096) * DriveConstants.kWheelCircumference,
      motorFL.getSelectedSensorVelocity()* (10.0 / 4096) * DriveConstants.kWheelCircumference,
      motorFL.getSelectedSensorVelocity()* (10.0 / 4096) * DriveConstants.kWheelCircumference,
      motorFL.getSelectedSensorVelocity()* (10.0 / 4096) * DriveConstants.kWheelCircumference);
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
  public void testMotor(){
    m_drive.driveCartesian(1, 0, 0);
  }
}