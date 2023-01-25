// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  CANSparkMax m_leftMotor = new CANSparkMax(DriveConstants.kLeftMotorPort, MotorType.kBrushless);
  CANSparkMax m_rightMotor = new CANSparkMax(DriveConstants.kRightMotorPort, MotorType.kBrushless);

  private final RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);
    m_leftEncoder.setPositionConversionFactor(1);
    m_rightEncoder.setPositionConversionFactor(1);
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void resetEncoders(){
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getLeftEncoderInches() {
    return m_leftEncoder.getPosition() * PhysicalConstants.DRIVE_WHEEL_CIRCUM / PhysicalConstants.DRIVE_GEAR_RATIO;
  }

  public double getRightEncoderInches() {
    return m_rightEncoder.getPosition() * PhysicalConstants.DRIVE_WHEEL_CIRCUM / PhysicalConstants.DRIVE_GEAR_RATIO;
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderInches() + getRightEncoderInches()) / 2;
  }
  public double getAbsAverageEncoderDistance() {
    return (Math.abs(getLeftEncoderInches()) + Math.abs(getRightEncoderInches())) / 2;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  @Override
    public void periodic() {
      SmartDashboard.putNumber("Encoder Abs Avg", getAbsAverageEncoderDistance());
      // SmartDashboard.putNumber("Encoder Abs Avg", getAbsAverageEncoderDistance());
      SmartDashboard.putNumber("Encoder Position", getAverageEncoderDistance());
      SmartDashboard.putNumber("Encoder Ticks", m_leftEncoder.getPosition());//log();
    }
}
