// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto_Pattern;

import frc.robot.Constants;
import frc.robot.commands.TurnDeg;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.Pause;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class ReverseRealComplexAuto extends SequentialCommandGroup {
  /**
   * Creates a new ComplexAuto.
   *
   * @param drive The drive subsystem this command will run on
   * @param hatch The hatch subsystem this command will run on
   */
  public ReverseRealComplexAuto(DriveSubsystem drive) {
    addCommands(
        new DriveDistance(5, .5, drive), 
    
        new TurnDeg(90, -0.5, drive),

        new Pause(1),

        new DriveDistance(35, .5, drive),  

        new TurnDeg(90, 0.5, drive),

        new DriveDistance(80, .5, drive), 
        
        new TurnDeg(90, 0.5, drive),

        new DriveDistance(40, .5, drive)
        );
  }
}
