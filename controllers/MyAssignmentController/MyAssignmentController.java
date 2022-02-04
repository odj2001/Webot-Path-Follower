// File:          MyAssignmentController.java
// Date:
// Description:
// Author: Stuart Groom- 201435331
// Modifications:

// ==============================================================
// COMP329 2021 Programming Assignment
// ==============================================================
// 
// The aim of the assignment is to move the robot around the arena in such a way
// as to generate an occupancy grid map of the arena itself.  Full details can be
// found on CANVAS for COMP329
//
// Only add code to the controller file - do not modify the other java files in this project.
// You can add code (such as constants, instance variables, initialisation etc) anywhere in 
// the file, but the navigation itself that occurs in the main loop shoudl be done after checking
// the current pose, and having updated the two displays.
//
// Note that the size of the occup[ancy grid can be changed (see below) as well as the update
// frequency of the map, adn whether or not a map is generated.  Changing these values may be
// useful during the debugging phase, but ensure that the solution you submit generates an
// occupancy grid map of size 100x100 cells (with a recommended update frquency of 2).
//
// ==============================================================

import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Display;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.PositionSensor;

public class MyAssignmentController {

  // ---------------------------------------------------------------------------
  // Dimensions of the Robot
  // ---------------------------------------------------------------------------
  // Note that the dimensions of the robot are not strictly circular, as 
  // according to the data sheet the length is 485mm, and width is 381mm
  // so we assume for now the aprox average of the two (i.e. 430mm)
  private final static double ROBOT_RADIUS = 0.215;  // in meters
  public static double wheelRadius = 0.0975;         // in meters
  public static double axelLength = 0.31;            // Distance (in m) between the two wheels
  public static int MAX_NUM_SENSORS = 16;            // Number of sensors on the robot
  public enum MoveState { FORWARD, ROTATE };
 

  // ---------------------------------------------------------------------------
  // Assignment Parameters
  // ---------------------------------------------------------------------------
  // Note that ideally the number of cells in the occupancy grid should be a multiple of the
  // display size (which is 500x500).  So smaller values such as 50x50 or 25x25 could be used
  // to initialise a map with fewer, but larger grid cells 
  private final static int NUMBER_OF_ROWCELLS = 100;   // How many cells across the occupancy grid
  private final static int NUMBER_OF_COLCELLS = 100;   // How many cells down the occupancy grid
  
  // This is the frequency that the map is updated (i.e. the map is updated every GRID_UPDATE_FREQUENCY
  // times the loop is iterated.  Increasing it may make the simulator run faster, but fewer samples
  // will be taken
  private final static int GRID_UPDATE_FREQUENCY = 2;  // How frequently do we sample the world 
  
  // This boolean switches on (or off) the generation of the occupancy grid.  It may be useful to
  // make this false whilst working on the navigation code to speed things up, but any final solution
  // should verify that a valid occupancy grid map is generated.
  private final static boolean GENERATE_OCCUPANCY_GRID = true;

  // ---------------------------------------------------------------------------
  // Robot instance
  // ---------------------------------------------------------------------------
  public static Supervisor robot;
  public static Node robotNode;

  // ==================================================================================
  // Static Methods  
  // ==================================================================================
  // getLocalisedPos()
  //   returns the real position of the robot without the need for localisation through
  //   particle filters etc.  The supervisor mode is used to facilitate this.
  public static Pose getLocalisedPos() {
    double[] realPos = robotNode.getPosition();
    double[] rot = robotNode.getOrientation(); // 3x3 Rotation matrix as vector of length 9
    double theta1 = Math.atan2(rot[2], rot[8]);
    double halfPi = Math.PI/2;
    double theta2 = theta1 + halfPi;
    if (theta1 > halfPi)
        theta2 = -(3*halfPi)+theta1;
    
    return new Pose(realPos[0], -realPos[2], theta2);
  }


  // ==================================================================================
  // Main Methods 
  // ==================================================================================  
  public static void main(String[] args) {

    // Define Robot Parameters
    long loopCounter = 0;           // Used to count the number of main loop iterations                
    // ---------------------------------------------------------------------------
    // create the Supervised Robot instance.
    // ---------------------------------------------------------------------------
    robot = new Supervisor();
    robotNode = robot.getSelf(); // Get a handle to the Robot Node in supervisor mode
    
    // get the time step of the current world.
    int timeStep = (int) Math.round(robot.getBasicTimeStep());    
    
    // ---------------------------------------------------------------------------
    // Set up motor devices
    // ---------------------------------------------------------------------------
    Motor leftMotor = robot.getMotor("left wheel");
    Motor rightMotor = robot.getMotor("right wheel");
    

    // Set the target positions of the motors
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);


    // Initialise motor velocity
    double av = 5.0;
    leftMotor.setVelocity(av);
    rightMotor.setVelocity(av);

    // ---------------------------------------------------------------------------
    // set up proximity detectors
    // ---------------------------------------------------------------------------
       // -------------------------
    // set up proximity detectors
    // -------------------------
    int MAX_NUM_SENSORS = 16;
    DistanceSensor[] ps = new DistanceSensor[MAX_NUM_SENSORS];
    String[] psNames = {
      "so0", "so1", "so2", "so3", "so4", "so5", "so6", "so7",
      "so8", "so9", "so10", "so11", "so12", "so13", "so14", "so15",     
    };
    double[] psValues = {0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0};

    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
      ps[i] = robot.getDistanceSensor(psNames[i]);
      ps[i].enable(timeStep);
    }
    
    
    // The following array determines the orientation of each sensor, based on the
    // details of the Pioneer Robot Stat sheet.  Note that the positions may be slightly
    // inaccurate as the pioneer is not perfectly round.  Also these values are in degrees
    // and so may require converting to radians.  Finally, we assume that the front of the
    // robot is between so3 and so4.  As the angle between these is 20 deg, we assume that 
    // they are 10 deg each from the robot heading
    double[] psAngleDeg = { 90, 50, 30, 10, -10, -30, -50, -90,
                            -90, -130, -150, -170, 170, 150, 130, 90};

    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
      ps[i] = robot.getDistanceSensor(psNames[i]);
      ps[i].enable(timeStep);
    }

    // ---------------------------------------------------------------------------
    // Set up occupancy grid
    // ---------------------------------------------------------------------------
    OccupancyGrid grid;                              // Instantiate to generate an occupancy grid
    if (GENERATE_OCCUPANCY_GRID == true) {
      grid = new OccupancyGrid(5.0, 5.0,             // Size of the arena
                                NUMBER_OF_ROWCELLS,  // Number of cells along the x-axis
                                NUMBER_OF_COLCELLS,  // Number of cells along the y-axis
                                ps,                  // Array of distance sensors
                                psAngleDeg,          // Orientation of each sensor (array)
                                ROBOT_RADIUS);       // Radius of the robot body (assumes cylindrical)
    } else {
      grid=null;                                     // No occupancy grid will be generated
    }
  
    // ---------------------------------------------------------------------------
    // Set up display devices
    // ---------------------------------------------------------------------------
    // The sensor view from the Labs is included to assist in debugging
    Display sensorDisplay = robot.getDisplay("sensorDisplay");
    SensorView sensorView = new SensorView(sensorDisplay, ps, psAngleDeg, ROBOT_RADIUS);

    // A variant of the Arena view is used to show the robot position in a map.
    // The current display is configured as a 500x500 display attached to the robot.     
    Display occupancyGridDisplay = robot.getDisplay("occupancyGridDisplay");
    ArenaView gridView = new ArenaView(occupancyGridDisplay, getLocalisedPos(), grid, 5.0, 5.0, ROBOT_RADIUS);


    
    double maxVel = leftMotor.getMaxVelocity();
    
    double leftVel, rightVel;
    double targetVel = 0.2*maxVel;          // Our target velocity
    double slowVel = 0.1*maxVel;

    // ---------------------------------------------------------------------------
    // Main loop:
    // ---------------------------------------------------------------------------
    // perform simulation steps until Webots is stopping the controller
    int count = 0; //waypoint counter
    
    Pose waypoint1 = new Pose (0.4, 2.15, 0.0);
    Pose waypoint2 = new Pose (1.12, 1.3, 0);
    Pose waypoint3 = new Pose (2.1, 1.3, 0);
    Pose waypoint4 = new Pose (2.1, -1.95, 0);
    Pose waypoint5 = new Pose (-2.1, -1.95, 0);
    Pose waypoint6 = new Pose (-2.1, -0.6, 0);
    Pose waypoint7 = new Pose (-1.1, -0.6, 0);
    Pose waypoint8 = new Pose (-1.1, 0.45, 0);
    Pose waypoint9 = new Pose (-2.1, 0.45, 0);
    Pose waypoint10 = new Pose (-2.1, 1.25, 0);
    Pose waypoint11 = new Pose (-0.25, 1.25, 0);
    Pose waypoint12 = new Pose (-0.25, -0.5 , 0);
    Pose waypoint13 = new Pose (0.9, -0.5 , 0);
    Pose waypoint14 = new Pose (0.9, 2.1 , 0);
    Pose waypoint15 = new Pose (2.1, 2.1 , 0);
    
    
    Pose[] path = {waypoint1, waypoint2, waypoint3, waypoint4, waypoint5, waypoint6, waypoint7, waypoint8, waypoint9, waypoint10, waypoint11,waypoint12,waypoint13, waypoint14, waypoint15};
    double angle = 0.0; 
    double timer = 0;

    
    boolean inMotion = false;
    MoveState state = MoveState.FORWARD;
    double dist = 0;
    double forwardTime = 0;
    double turnAngle = 0;
    double turnTime = 0;
    
      // ----------------------------
    while (robot.step(timeStep) != -1) {
      // ---------------------------------------------------------------------------
      // Get current pose of the robot        
      // ---------------------------------------------------------------------------

               
               
      Pose p = getLocalisedPos();
      System.out.println(p);
      System.out.println("targetVel");
      System.out.println(targetVel);
      switch (state) {
        case FORWARD:
          System.out.println("forward");
          if (inMotion == true) {
            // We have already started moving   
            // We hardcode 2 seconds here for now, but it would be better to
            // parameterise this based on the linearVelocity parameter   
                 
            if (timer > forwardTime) {
              // We need to stop
              leftMotor.setVelocity(0.0);
              rightMotor.setVelocity(0.0);
              inMotion = false;           // we are no longer in motion
              state = MoveState.ROTATE;   // change state  
              timer = 0;            
            } 
            else {
              timer+=timeStep;      // Increment by the time state
            }
          } 
          else {
            // Start moving forward
            dist = forwardDist(p, path[count]);
            forwardTime = forwardTime(dist, targetVel);
            

            inMotion=true;               // note that we are now in motion
            leftMotor.setVelocity(targetVel);
            rightMotor.setVelocity(targetVel);
                     
          } 
          break;
        
        
        case ROTATE:
          // Note we are clearly not rotating here just yet but going backwards
          System.out.println("rotate");
          if (inMotion == true) {
            // We have already started moving
            
            if (timer > turnTime+96) {
            //+96 as it takes time for the robot to get to the desired vel so wouldnt
            //travel as far, so we give it 3 more time steps
              // We need to stop
              leftMotor.setVelocity(0.0);
              rightMotor.setVelocity(0.0);
              inMotion = false;           // we are no longer in motion
              state = MoveState.FORWARD;   // change state     
              timer = 0;             
            } 
            else {
                       
              timer+=timeStep;      // Increment by the time state
            }
          } 
          else {
            // Start moving backwards
            turnAngle = turnAngle(p, path[count+1]);
            turnTime = turnTime(turnAngle, slowVel);
            
            System.out.println("timer");
            System.out.println(timer); 
            System.out.println("turnAngle");
            System.out.println(turnAngle);
            System.out.println("turnTime");
            System.out.println(turnTime);
            
            
            if (turnAngle < 0){

               leftMotor.setVelocity(slowVel);
               rightMotor.setVelocity(-slowVel);
            }
            else if (turnAngle > 0){ 
            
               leftMotor.setVelocity(-slowVel);
               rightMotor.setVelocity(slowVel);
            }
               
            inMotion=true;               // note that we are now in motion  
            count++;       
          } 
          break;      
        }
        

      // ---------------------------------------------------------------------------
      // Update the grid map and arena display
      // ---------------------------------------------------------------------------
      if (loopCounter++ % GRID_UPDATE_FREQUENCY == 0) {
        if (GENERATE_OCCUPANCY_GRID == true) {
          grid.occupancy_grid_mapping(p);
        }
        gridView.setPose(p);
        gridView.paintView();
      }

      // ---------------------------------------------------------------------------
      // Update the sensor display
      // ---------------------------------------------------------------------------
      sensorView.setPose(p);
      sensorView.paintView();
      
    }
}
    
public static double forwardDist (Pose p, Pose waypoint){
  
  double xDif;
  double yDif;
  double dist;
  
  xDif = p.getX() - waypoint.getX();
  yDif = p.getY() - waypoint.getY();
  
  dist = Math.sqrt(Math.pow(xDif,2) + Math.pow(yDif,2));
  
  return dist;
}

      
public static double forwardTime (double distance, double velocity){
   double targetTime;
   
   targetTime = 1000.0*(distance/(velocity*wheelRadius)); 
                                           
  
   //velocity is in radians/s, need to convert to m. 
   //velocity * wheelradius = m/s
   return targetTime; 

}

public static double turnAngle(Pose p, Pose waypoint){

  double turnAngle;
  double YDIF;
  double XDIF;
  double angle;
  
  XDIF = (waypoint.getX() - p.getX());
  YDIF = (waypoint.getY() - p.getY());
  
  angle = Math.atan2(YDIF, XDIF);
  turnAngle = p.getDeltaTheta(angle);
  
  return turnAngle;
}

public static double turnTime(double turnAngle, double vel){
  
  double turnDist;
  double turnTime;
  
  turnDist = 0.5*axelLength * Math.abs(turnAngle);//turn dist in radians 
  System.out.println("turnDist");
  System.out.println(turnDist);
            
  turnTime = 1000*(turnDist/(vel*wheelRadius)); //1000* for conversion to ms
                                //vel*wheelRadius for radians -> meters 
  return turnTime;
  
}





  
}