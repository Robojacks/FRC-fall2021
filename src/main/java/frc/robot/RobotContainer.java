/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//testing
package frc.robot;

import frc.robot.drive.RevDrivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import static edu.wpi.first.wpilibj.XboxController.Axis.*;

import static frc.robot.Constants.*;



/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Drive Controller
  private XboxController xbox = new XboxController(kXboxPort);

  // Drive Subsystem
  private final RevDrivetrain rDrive = new RevDrivetrain();

  // Drive with Controller 
  private Command manualDrive = new RunCommand(
    () -> rDrive.getDifferentialDrive().
    tankDrive((xbox.getRawAxis(kLeftY.value)), 
    (xbox.getRawAxis(kLeftX.value))), 
    (rDrive)
    );

  public RobotContainer() {

    rDrive.setDefaultCommand(manualDrive);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
