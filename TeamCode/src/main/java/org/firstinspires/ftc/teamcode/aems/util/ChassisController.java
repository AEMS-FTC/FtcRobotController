package aems;

class ChassisController {
    double leftX = 0;
    double leftY = 0;
    
    double rightX = 0;
    double rightY = 0;

    double chassisYaw = 0;

    public void setChassisSpeeds(double lx, double ly, double rx, double ry, double robotYaw) {
        this.leftX = lx;
        this.leftY = ly;
        
        this.rightX = rx;
        this.rightY = ry;

        this.chassisYaw = robotYaw;
    }

    public double[] getChassisSpeeds() {
        return null;
    }
}