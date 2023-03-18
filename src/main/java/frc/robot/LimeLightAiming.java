// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimeLightConstants;


public class LimeLightAiming extends LimeLight{
  /** Creates a new LimelightSubsystem. */
  private static LimeLight m_Limelight = new LimeLight();


  public double estimateTargetDistance(){
    double finheight = LimeLightConstants.HUB_HEIGHT - LimeLightConstants.LIMELIGHT_HEIGHT;
    double finangle = LimeLightConstants.LIMELIGHT_MOUNT_ANGLE + m_Limelight.getdegVerticalToTarget();
    finangle = Math.toRadians(finangle);
    double finratio = Math.tan(finangle); //Math.tan takes in radians
    // SmartDashboard.putNumber("FinHeight", finheight);
    // SmartDashboard.putNumber("Finratio", finratio);
    // SmartDashboard.putNumber("Finangle", finangle);
    if(getIsTargetFound()){
      return (finheight)/finratio;
    }
   return 0;
  }
}