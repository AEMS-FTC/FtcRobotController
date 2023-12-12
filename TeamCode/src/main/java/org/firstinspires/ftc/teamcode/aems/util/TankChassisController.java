package aems;

public class TankChassisController {
    private double leftY = 0; //The left-y-axis value
    
    private double rightY = 0; //The right-y-axis value

    public void setChassisSpeeds(double ly, double ry) {
        this.leftY = ly; //assigns the left-y-axis value to a givin ly value
        this.rightY = ry; //assigns the right-y-axis value to a givin ry value
    }

    public double[] getChassisSpeeds() {
        double l = leftY; //assigns the left-side speed to the left-y-axis value
        double r = rightY; //assigns the right-side speed to the right-y-axis value

        double[] chassisSpeeds = {
            l, //left wheels (index 0)
            r  //right wheels (index 1)
        };

        return chassisSpeeds; //Returns the chassis speeds

    }
}
