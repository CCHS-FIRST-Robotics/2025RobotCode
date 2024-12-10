package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;

public class AutoBuilderManager {
    public AutoBuilderManager() {
        AutoBuilder.configureLTV(
            Supplier<Pose2d> poseSupplier, 
            Consumer<Pose2d> resetPose, 
            Supplier<ChassisSpeeds> speedsSupplier, 
            Consumer<ChassisSpeeds> output, 
            double dt, 
            ReplanningConfig replanningConfig, 
            BooleanSupplier shouldFlipPath, 
            Subsystem driveSubsystem
        );
        // //   RobotConfig config;
        // // try{
        // //   config = RobotConfig.fromGUISettings();
        // // } catch (Exception e) {
        // //   // Handle exception as needed
        // //   e.printStackTrace();
        // // }
        

        // // Configure AutoBuilder last
        //     AutoBuilder.configure(
        //         // this::getPose, // Robot pose supplier
        //         // this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        //         // this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //         // (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        //         // new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
        //         // config, // The robot configuration
        //         // () -> {
        //         //   // Boolean supplier that controls when the path will be mirrored for the red alliance
        //         //   // This will flip the path being followed to the red side of the field.
        //         //   // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //         //   var alliance = DriverStation.getAlliance();
        //         //   if (alliance.isPresent()) {
        //         //     return alliance.get() == DriverStation.Alliance.Red;
        //         //   }
        //         //   return false;
        //         // },
        //         // this // Reference to this subsystem to set requirements
        //     );
    }
}