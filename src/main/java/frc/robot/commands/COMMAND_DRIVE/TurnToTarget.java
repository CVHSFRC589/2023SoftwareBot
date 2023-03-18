// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DRIVE;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToTarget extends CommandBase {
  /** Creates a new TurnToTarget. */
  private DriveSubsystem m_drive;
  private Timer m_timer;

  public TurnToTarget(DriveSubsystem drive) {
    m_timer = new Timer();
    m_drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setSafetyPID(false);
    m_drive.setPipeline(1);
    m_timer.reset();
    m_timer.stop();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!(m_drive.getLimeDegToTarget() < .5 && m_drive.getLimeDegToTarget() > -.5)) {
      m_drive.setVelocityLeftMotor(m_drive.limeAdjustLeftRightVelocity());
      m_drive.setVelocityRightMotor(-m_drive.limeAdjustLeftRightVelocity());
      m_timer.stop();
      m_timer.reset();
    } else {
      m_drive.setVelocityLeftMotor(0);
      m_drive.setVelocityRightMotor(0);
      m_timer.start();

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_timer.reset();
    m_drive.setVelocityLeftMotor(0);
    m_drive.setVelocityRightMotor(0);
    m_drive.setSafetyPID(true);
    
    m_drive.setPipeline(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.hasElapsed(.5)) {
      return true;
    }

    else {
      return false;
    }
  }
}