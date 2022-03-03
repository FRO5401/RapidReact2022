package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTables extends SubsystemBase {
  
  NetworkTable ballTable, targetTable, robotTable;
  NetworkTableInstance inst;
  NetworkTableEntry ballXEntry, ballYEntry, ballDEntry, ballREntry;
  NetworkTableEntry targetXEntry, targetYEntry, targetDEntry;
  NetworkTableEntry robotXEntry, robotYEntry, robotDEntry;
  private static double ballX, ballY, ballDistance;
  public double ballRadius;
  public double targetX, targetY, targetDistance;
  public double robotX, robotY, robotDistance;


  public NetworkTables() {
    inst = NetworkTableInstance.getDefault();
    ballTable = inst.getTable("Ball");
    targetTable = inst.getTable("Targets");
    robotTable = inst.getTable("Robot");
    ballXEntry = ballTable.getEntry("ballX");
    ballYEntry = ballTable.getEntry("ballY");
    targetXEntry = ballTable.getEntry("targetX");
    targetYEntry = ballTable.getEntry("targetY");
    robotXEntry = ballTable.getEntry("robotX");
    robotYEntry = ballTable.getEntry("robotY");
    ballDEntry = ballTable.getEntry("ballDistance");
    targetDEntry = ballTable.getEntry("targetDistance");
    robotDEntry = ballTable.getEntry("robotDistance");
    ballREntry = ballTable.getEntry("ballRadius");
    ballX = 0.0;
    ballY = 0.0;
    targetX = 0.0;
    targetY = 0.0;
    robotX = 0.0;
    robotY = 0.0;
    targetDistance = 0.0;
    robotDistance = 0.0;
    ballDistance = 0.0;
    ballRadius = 0.0;

    inst.startClientTeam(5401); // where TEAM=190, 294, etc, or use inst.
    inst.startDSClient();

    Shuffleboard.getTab("SmartDashboard").add("Current Ball X", getBallXValue()).withWidget(BuiltInWidgets.kGraph);
    Shuffleboard.getTab("SmartDashboard").add("Current Ball Y", getBallYValue()).withWidget(BuiltInWidgets.kGraph);
    Shuffleboard.getTab("SmartDashboard").add("Current Ball radius", getBallRadius()).withWidget(BuiltInWidgets.kGraph);
    Shuffleboard.getTab("SmartDashboard").add("Ball Distance", getBallDistance()).withWidget(BuiltInWidgets.kGraph);
  }

  @Override
  public void periodic() {
    updateValue();
    reportValues();
    //odometry.update(navxGyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  public void updateValue() {
     // recommended if running on DS computer; this gets the robot

    ballX = ballXEntry.getDouble((ballX != 0) ? ballX : 0);
    ballY = ballYEntry.getDouble((ballY != 0) ? ballY : 0);
    ballRadius = (ballX == 0 && ballY == 0) ? 0 : ballREntry.getDouble((ballRadius != 0) ? ballRadius : 0);
    ballDistance = ballDEntry.getDouble((ballDistance != 0) ? ballDistance : 0);
    targetX = targetXEntry.getDouble((targetX != 0) ? targetX : 0);
    targetY = targetYEntry.getDouble((targetY != 0) ? targetY : 0);
    targetDistance = ballDEntry.getDouble((targetDistance != 0) ? targetDistance : 0);
    robotX = robotXEntry.getDouble((robotX != 0) ? robotX : 0);
    robotY = robotYEntry.getDouble((robotY != 0) ? robotY : 0);
    robotDistance = robotDEntry.getDouble((robotDistance != 0) ? robotDistance : 0);
    //powerPortX = powerPortXEntry.getDouble(0.0);
    //powerPortY = powerPortYEntry.getDouble(0.0);
    //System.out.println("The Ball coordinates are: " + "X: " + ballX + " Y: " + ballY);
    //System.out.println("The Ball is " + ballDistance + " away");
    //System.out.println("The radius is " + radius);
    //System.out.println("The Power Port coordinates are: " + "X: " + powerPortY + " Y: " + powerPortY);
  }

  public double getBallXValue() {
    return ballX;
  }

  public double getBallYValue() {
    return ballY;
  }

  public double getBallRadius(){
    return ballRadius;
  }

  public double getBallDistance(){
    return ballDistance;
  }

  public double getTargetXValue() {
    return targetX;
  }

  public double getTargetYValue(){
    return targetY;
  }

  public double getRobotXValue() {
    return robotX;
  }

  public double getRobotYValue() {
    return robotY;
  }

  public double getTargetDistance(){
    return targetDistance;
  }

  public double getRobotDistance() {
    return robotDistance;
  }

  
  public void resetValues(){
    ballX = 0;
    ballY = 0;
    targetY = 0;
    targetX = 0;
    robotX = 0;
    robotY = 0;
    robotDistance = 0;
    targetDistance = 0;
    ballDistance = 0;
    ballRadius = 0;
  }

  public boolean checkCentered(){

    if(getBallXValue() >= 300 && getBallXValue() <= 340){
      return true;
    }
    
    else if(((getBallXValue() < 300) & (getBallXValue() >= 0)) || ((getBallXValue() > 340) & (getBallXValue() <= 640))){
      return false;
    }
    return false;
  }



  public void reportValues()
  {
   
  }
}