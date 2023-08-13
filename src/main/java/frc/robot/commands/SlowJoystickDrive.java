// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class SlowJoystickDrive extends CommandBase {
  private DriveTrain driveTrain;

  /** Creates a new SlowDrive. */
  public SlowJoystickDrive(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.arcadeDrive(RobotContainer.joystick.getY() * 0.75,
        erf(RobotContainer.joystick.getX() / 1.5d) * 0.75d);
    // sin(x) - sin(x / 2) seems to not be half bad as well
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Error function, look it up to see what is looks like graphed
   * (https://en.wikipedia.org/wiki/Error_function)
   * I copied from here
   * (https://introcs.cs.princeton.edu/java/21function/ErrorFunction.java.html)
   * 
   * @param t input to error function
   * @return value at 't'
   */
  private double erf(double z) {
    double t = 1.0 / (1.0 + 0.47047 * Math.abs(z));
    double poly = t * (0.3480242 + t * (-0.0958798 + t * (0.7478556)));
    double ans = 1.0 - poly * Math.exp(-z * z);
    if (z >= 0)
      return ans;
    else
      return -ans;
  }
}
