package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.playSong;
import frc.robot.commands.zeroHeading;
import frc.robot.commands.driving.FOCdrivingCommand;
import frc.robot.commands.driving.resetEncodersCommnad;
import frc.robot.commands.driving.zeroStates;
import frc.robot.subsystems.PDPSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.visionSystems;

public class RobotContainer {

    private final XboxController xc = new XboxController(0);

    private final visionSystems Cameras = new visionSystems();
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    
    
    

    private final FOCdrivingCommand focDrive = new FOCdrivingCommand(
            swerveSubsystem,
            () -> xc.getLeftX(),
            () -> xc.getLeftY(),
            () -> xc.getRightX(),
            () -> !xc.getLeftBumperButton(),
            () -> xc.getRightBumperButton());
        
    /*private final FOCdrivingCommand driveCommand = new FOCdrivingCommand(
        swerveSubsystem, 
        ()-> joystick.getX(), 
        ()-> joystick.getY(), 
        ()-> joystick.getZ(), 
        null, 
        null);*/ 

    private SendableChooser<String> autoChooser;
    private SendableChooser<String> songChooser;
    private SendableChooser<String> playSong;
    
    private boolean playFunc = false;

    public RobotContainer() {
        
        swerveSubsystem.setDefaultCommand(focDrive);
        configureButtonBindings();
        registerCommands();



        // Build an auto chooser. This will use Commands.none() as the default option.
        // As an example, this will only show autos that start with "comp" while at
        // competition as defined by the programmer
        autoChooser = new SendableChooser<String>();
        
        autoChooser.addOption("TestRun", "TestRun");
        autoChooser.addOption("Calibration", "Calibration");
        autoChooser.setDefaultOption("New Path", "New Path");
    

        songChooser = new SendableChooser<String>();
        songChooser.addOption("Hehe", "eeeaaaooo.chrp");
        songChooser.addOption("Black Betty", "blackbetty.chrp");
        songChooser.addOption("Wii Channel", "Wii Channel.chrp");

        playSong = new SendableChooser<String>();
        
        playSong.addOption("Play", "true");
        playSong.addOption("Pause", "false");

        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        //SmartDashboard.putData("Song Chooser", songChooser);
        //SmartDashboard.putData("Play Song", playSong);

        if (playSong.getSelected() == "true"){playFunc = true;}
        else{playFunc = false;}
        
        
    }

    private void registerCommands() {
        NamedCommands.registerCommand("zero", new zeroHeading(swerveSubsystem));
        
    }

    private void configureButtonBindings() {
        new JoystickButton(xc, Constants.buttonA).whileTrue(new zeroHeading(swerveSubsystem));
        new JoystickButton(xc, Constants.buttonB).whileTrue(new resetEncodersCommnad(swerveSubsystem));
        new JoystickButton(xc, Constants.buttonX).onTrue(new zeroStates(swerveSubsystem));
        
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
             return swerveSubsystem.followPathCommand(autoChooser.getSelected());
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
       
    }

}