package aems;

public class MecanumChassisController {
    private double leftX = 0; //The left-x-axis value
    private double leftY = 0; //The left-y-axis value

    private double rightX = 0; //The right-x-axis value

    private double chassisYaw = 0; //The robot's yaw angle

    private boolean fieldCentric = false; //The robot's field-centric condition

    public MecanumChassisController(boolean fieldCentric) {
        this.fieldCentric = fieldCentric; 
        //If field-centric is true, then the robot's x and y values will be modified to account for robot's yaw
    }

    public void setChassisSpeeds(double lx, double ly, double rx, double robotYaw) {
        this.leftX = lx; //assigns the left-x-axis value to a givin lx value
        this.leftY = -ly; //assigns the left-y-axis value to a givin ly value
        
        this.rightX = rx; //assigns the right-x-axis value to a givin rx value

        this.chassisYaw = robotYaw; //The robot's yaw angle given by the IMU in radians
    }

    public double[] getChassisSpeeds() {
        double x; //x-axis value
        double y; //y-axis value
        double r = rightX; //The rotate value will always be the rightX value no matter what angle the robot is facing

        if(fieldCentric) {
            x = (leftY * Math.sin(chassisYaw)) + (leftX * Math.cos(chassisYaw)); //Assign field-centric values to x value
            y = (leftY * Math.cos(chassisYaw)) + (leftX * Math.sin(chassisYaw)); //Assign field-centric values to y value
        } else {
            x = leftX; //Assigns non field-centric values to x value
            y = leftY; //Assigns non field-centric values to y value
        }

        double[] chassisSpeeds = {
            y + x + r, //front left wheel (index 0)
            y - x + r, //back left wheel (index 1)
            y - x - r, //front right wheel (index 2)
            y + x - r  //back right wheel (index 3)
        };

        return chassisSpeeds; //Returns the chassis speeds
    }
}
