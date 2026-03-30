package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;

public final class Dimensions {
    // Robot dimensions
    public static final Distance BUMPER_LENGTH = Meters.of(0.837); // 33 inches
    public static final Distance BUMPER_WIDTH = Meters.of(0.853);  // 33.6 inches
    public static final Distance BUMPER_HEIGHT = Meters.of(0.154);   // 36 inches

    public static final Distance BUMPER_COLLISION_DISTANCE = Meters.of(1.16); // 22 inches

    public static final Distance HOPPER_EXTENSION_LENGTH = Meters.of(0.320); // 12 inches

    public static final Distance FUEL_SHOOTER_OFFSET = Meters.of(0.1385); // 4.22 inches

    public static final Distance HUB_HEIGHT = Meters.of(1.8); // 16.4 inches

    public static final Distance BLUE_SHOOT_AREA_X_MIN = Meters.of(0.5);
    public static final Distance BLUE_SHOOT_AREA_X_MAX = Meters.of(3.5);
    public static final Distance BLUE_SHOOT_AREA_Y_MIN = Meters.of(0.5);
    public static final Distance BLUE_SHOOT_AREA_Y_MAX = Meters.of(7.5);

    public static final Distance RED_SHOOT_AREA_X_MIN = Meters.of(13);
    public static final Distance RED_SHOOT_AREA_X_MAX = Meters.of(16);
    public static final Distance RED_SHOOT_AREA_Y_MIN = Meters.of(0.5);
    public static final Distance RED_SHOOT_AREA_Y_MAX = Meters.of(7.5);
    
    public static final Pose3d SHOOTER_POSE =
             new Pose3d(-0.1475, 0.1475, 0.52, new Rotation3d(0, 0, 0));

    public static double[][] getCollisionValuesForX(double y)
    {

        double[][] collisionValuesX = {{-1.0,0.0}, {16.55,17.0}, { -1.0, -1.0 }, { -1.0, -1.0 }, { -1.0, -1.0 }, { -1.0, -1.0 }};

        if(1.3 < y && y < 1.63) // between the tarmac lines
        {
            collisionValuesX[3] = new double[] {4.0, 5.2};
            collisionValuesX[4] = new double[] {11.3, 12.5};
        }
        else if(3.18 < y && y < 3.45) // between the tarmac lines
        {
            collisionValuesX[2] = new double[] {1., 1.1};
        }
        else if(3.45 < y && y < 3.75) // between the tarmac lines
        {
            collisionValuesX[2] = new double[] {1., 1.1};
            collisionValuesX[3] = new double[] {4.0, 5.2};
            collisionValuesX[4] = new double[] {11.3, 12.5};

        }
        else if(3.75 < y && y < 4.37) // between the tarmac lines
        {
            collisionValuesX[2] = new double[] {1., 1.1};
            collisionValuesX[3] = new double[] {4.0, 5.2};
            collisionValuesX[4] = new double[] {11.3, 12.5};
            collisionValuesX[5] = new double[] {15.42, 15.52};
        }
        else if(4.37 < y && y < 4.67) // beyond the outermost tarmac line
        {
            collisionValuesX[3] = new double[] {4.0, 5.2};
            collisionValuesX[4] = new double[] {11.3, 12.5};
            collisionValuesX[5] = new double[] {15.42, 15.52};
        }
        else if(4.67 < y && y < 4.94) // beyond the outermost tarmac line
        {
            collisionValuesX[5] = new double[] {15.42, 15.52};
        }
        else if(6.48 < y && y < 6.82) // beyond the outermost tarmac line
        {
            collisionValuesX[3] = new double[] {4.0, 5.2};
            collisionValuesX[4] = new double[] {11.3, 12.5};
        }

        return collisionValuesX;
    }

    public static double[][] getCollisionValuesForY(double x)
    {

        double[][] collisionValuesY = {{-1.0,0.0}, {8.05,9.0}, { -1.0, -1.0 }, { -1.0, -1.0 }, { -1.0, -1.0 }};

        if(1 < x && x < 1.1) // between the tarmac lines
        {
            collisionValuesY[2] = new double[] {3.18, 4.37};
        }
        else if(4 < x && x < 5.2) // between the tarmac lines
        {
            collisionValuesY[2] = new double[] {1.30, 1.63};
            collisionValuesY[3] = new double[] {3.45, 4.67};
            collisionValuesY[4] = new double[] {6.48, 6.82};
        }
        else if(11.3 < x && x < 12.5) // between the tarmac lines
        {
            collisionValuesY[2] = new double[] {1.30, 1.63};
            collisionValuesY[3] = new double[] {3.45, 4.67};
            collisionValuesY[4] = new double[] {6.48, 6.82};
        }
        else if(15.42 < x && x < 15.52) // beyond the outermost tarmac line
        {
            collisionValuesY[2] = new double[] {3.75, 4.94};
        }

        return collisionValuesY;
    }
}