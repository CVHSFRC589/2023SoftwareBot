// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto_Pattern;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPhysicalConstants;
import frc.robot.commands.COMMAND_ARM.ArmSetPosVelocity;
import frc.robot.commands.COMMAND_ARM.ArmZeroEncoder;
import frc.robot.commands.COMMAND_DRIVE.DriveDistance;
import frc.robot.commands.COMMAND_GRIP.CloseGripper;
import frc.robot.commands.COMMAND_GRIP.OpenGripper;
import frc.robot.commands.COMMAND_MISC.Pause;
import frc.robot.commands.COMMAND_WRIST.WristSetSpeedTime;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score3Cube extends SequentialCommandGroup {
  /** Creates a new Score3Cube. */
  public Score3Cube(ArmSubsystem arm, DriveSubsystem drive, GripperSubsystem grip, WristSubsystem wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new ArmZeroEncoder(arm),
      new CloseGripper(grip),
      new WristSetSpeedTime(wrist, () -> -0.5, 1),
      new ArmSetPosVelocity(arm, ArmPhysicalConstants.level2),
      new DriveDistance(24, 0.5, drive),
      new WristSetSpeedTime(wrist, () -> 0.7, 1.5),
      new Pause(0.5),
      new OpenGripper(grip),
      new Pause(2),
      new DriveDistance(24, -0.5, drive),
      new WristSetSpeedTime(wrist, () -> -0.5, 1),
      new ArmSetPosVelocity(arm, 0)
    );
  }
}
