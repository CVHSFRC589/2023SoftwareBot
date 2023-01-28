// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants.*;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class DrivePIDSubsystem extends SubsystemBase {

//   CANSparkMax m_leftMotor = new CANSparkMax(DriveConstants.kLeftMotorPort, MotorType.kBrushless);
//   CANSparkMax m_rightMotor = new CANSparkMax(DriveConstants.kRightMotorPort, MotorType.kBrushless);

//   private SparkMaxPIDController m_leftPIDController, m_rightPIDController;
//   private final RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
//   private final RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();

//   // The robot's drive
//   private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

//   /** Creates a new DriveSubsystem. */
//   public DrivePIDSubsystem() {
//     /**
//      * The RestoreFactoryDefaults method can be used to reset the configuration parameters
//      * in the SPARK MAX to their factory default state. If no argument is passed, these
//      * parameters will not persist between power cycles
//      */

//     m_rightMotor.setInverted(true);
//     // initialze PID controller and encoder objects
//     m_leftPIDController = m_leftMotor.getPIDController();
//     m_rightPIDController = m_rightMotor.getPIDController();
//     m_leftEncoder.setPosition(0);
//     m_rightEncoder.setPosition(0);
//     m_leftEncoder.setPositionConversionFactor(1.76);
//     m_rightEncoder.setPositionConversionFactor(1.76);
//     //exact measurements from circumference
//     // PID coefficients
    

//     // set PID coefficients
//     m_leftPIDController.setP(PIDConstants.kP);
//     m_leftPIDController.setI(PIDConstants.kI);
//     m_leftPIDController.setD(PIDConstants.kD);
//     m_leftPIDController.setIZone(PIDConstants.kIz);
//     m_leftPIDController.setFF(PIDConstants.kFF);
//     m_leftPIDController.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);

//     //pid 2
//     m_rightPIDController.setP(PIDConstants.kP);
//     m_rightPIDController.setI(PIDConstants.kI);
//     m_rightPIDController.setD(PIDConstants.kD);
//     m_rightPIDController.setIZone(PIDConstants.kIz);
//     m_rightPIDController.setFF(PIDConstants.kFF);
//     m_rightPIDController.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);
//   }

//   /**
//    * Sets the max output of the drive. Useful for scaling the drive to drive more
//    * slowly.
//    *
//    * @param maxOutput the maximum output to which the drive will be constrained
//    */
//   public void setMaxOutput(double maxOutput) {
//     m_drive.setMaxOutput(maxOutput);
//   }

//   public void setVelocityLeftMotor(double velocity) { 
//     m_leftPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
//   }

//   public void setVelocityRightMotor(double velocity) { 
//     m_rightPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
//   }

//   public void setPositionLeftMotor(double position) {
//     m_leftPIDController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
//   }

//   public void setPositionRightMotor(double position) {
//     m_rightPIDController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
//   }


//   @Override
//     public void periodic() {
//       // SmartDashboard.putNumber("Encoder Abs Avg", getAbsAverageEncoderDistance());
//       // // SmartDashboard.putNumber("Encoder Abs Avg", getAbsAverageEncoderDistance());
//       // SmartDashboard.putNumber("Encoder Position", getAverageEncoderDistance());
//       // SmartDashboard.putNumber("Encoder Ticks", m_leftEncoder.getPosition());//log();

      
//     }
// }
