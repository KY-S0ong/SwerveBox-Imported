package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.playSong;
import frc.robot.commands.coral.intakeCoral;
import frc.robot.commands.coral.outCoral;
import frc.robot.commands.driving.FOCdrivingCommand;
import frc.robot.commands.driving.resetEncodersCommnad;
import frc.robot.commands.driving.zeroHeading;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.coral_intake;
import frc.robot.subsystems.visionSystems;

public class RobotContainer {

    //private final XboxController xc = new XboxController(0);
    //private final PS4Controller ps = new PS4Controller(0);
    private final Joystick drivejoystick = new Joystick(0);
    private Joystick sysjoystick = new Joystick(1);

    private SendableChooser<Command> autoChoser;
    private String songChoice = "WiiChannel.chrp";
    private boolean playFunc = false;

    private final visionSystems Cameras = new visionSystems();

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(songChoice);
    private final coral_intake cIntake = new coral_intake();
    
    private final FOCdrivingCommand focDrive = new FOCdrivingCommand(
        swerveSubsystem, 
        ()-> -drivejoystick.getRawAxis(1), 
        ()-> drivejoystick.getRawAxis(0), 
        ()-> drivejoystick.getRawAxis(2), 
        ()-> !drivejoystick.getRawButton(1) 
    );
 
    public RobotContainer() {
        
        swerveSubsystem.setDefaultCommand(focDrive);
        configureButtonBindings();
        registerCommands();


        
        autoChoser = AutoBuilder.buildAutoChooser();
        

        //SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Auto Chooser", autoChoser);
        
        
        
    }

    private void registerCommands() {
        NamedCommands.registerCommand("zero", new zeroHeading(swerveSubsystem));
    }

    private void configureButtonBindings() {
        //new JoystickButton(xc, Constants.buttonA).whileTrue(new zeroHeading(swerveSubsystem));
        //new JoystickButton(xc, Constants.buttonB).whileTrue(new resetEncodersCommnad(swerveSubsystem));
        //new JoystickButton(xc, Constants.buttonY).toggleOnTrue(new playSong(swerveSubsystem, playFunc));
        
        new JoystickButton(drivejoystick, 5).toggleOnTrue(new playSong(swerveSubsystem, playFunc));
        new JoystickButton(drivejoystick, 6).whileTrue(new zeroHeading(swerveSubsystem));
        new JoystickButton(drivejoystick, 7).whileTrue(new resetEncodersCommnad(swerveSubsystem));
        
        new POVButton(sysjoystick, 0).whileTrue(new outCoral(cIntake));
        new POVButton(sysjoystick, 180).whileTrue(new intakeCoral(cIntake));
        
    }

    public Command getAutonomousCommand() {
        /*try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(autoChooser.getSelected());
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }*/

        try {
           //return new PathPlannerAuto(autoChoser.getSelected());
           return autoChoser.getSelected();
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
       
    }

}