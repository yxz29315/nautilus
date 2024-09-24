// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveDistance extends CommandBase {
  private final DriveTrain m_drive;
  private final double m_distance;
  private final double m_speed;

  /**
   * Creates a new DriveDistance.
   *
   * @param inches The number of inches the robot will drive
   * @param speed The speed at which the robot will drive
   * @param drive The drive subsystem on which this command will run
   */
  public DriveDistance(double inches, double speed, DriveTrain drive) {
    m_distance = inches / 1.25;
    m_speed = speed;
    m_drive = drive;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_drive.resetEncoders();
  }

  @Override
  public void execute() {
    m_drive.tankDrive(-1 * m_speed, 0, 0, 0);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.voltageDrive(0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_drive.getLeftEncoderDistance()) >= m_distance;
  }
}