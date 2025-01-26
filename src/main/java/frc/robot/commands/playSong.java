// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class playSong extends Command {
  private SwerveSubsystem swerveSubsystem;
  private String songNameFunc;
  private Boolean playSong;

  public playSong(SwerveSubsystem swerveSubsystem, String songNameFunc, Boolean playSong) {
    this.swerveSubsystem = swerveSubsystem;
    this.songNameFunc = songNameFunc;
    this.playSong = playSong;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {swerveSubsystem.loadSong(songNameFunc);}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    swerveSubsystem.loadSong(songNameFunc);
    swerveSubsystem.playSong();
    
   
     
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.pauseSong();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
