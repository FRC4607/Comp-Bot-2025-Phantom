package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the constants are needed,
 * to reduce verbosity.
 */
public final class Constants {

    /**
     * Elevator constants.
     */
    public static class ElevatorConstants {
        public static final int kmotor1CanId = 14;
        public static final int kmotor2CanId = 5;
        public static final int kmotor3CanId = 4;
        public static final int kmotor4CanId = 15;
        public static final int kcandiCanId = 55;
        /* Rotations of the motor per inch */
        public static final double kPulleyGearRatio = 1.167;
        public static final int kservoPort = 0;
    }

    /**
     * Windmill constants.
     */
    public static class WindmillConstants {
        public static final int kmotorCanId = 6;
        public static final int kcanCoderCanId = 6;
        public static final double ksensorToMechanismRatio = 1;
        public static final double krotorToSensorRatio = 74.4;
    }

    /**
     * Manipulator Constants.
     */
    public static class ManipulatorConstants {
        public static final int kmotorCanId = 13;
    }

    /**
     * Constants for the LEDs.
     */
    public static final class LEDConstants {
        public static final int kCANdleID = 30;

        /**
         * LED Strip class to define start and length.
         * */
        public static class LEDStrip {
            public final int m_start;
            public final int m_end;
            public final int m_length;

            /**
             * LED strip constructor.
             *
             * @param start the start of the LED strip
             * @param length the length of the LED strip
             */
            public LEDStrip(int start, int length) {
                m_start = start;
                m_length = length;
                m_end = start + length;
            }
        }

        /* Total number of RGB LEDs */
        public static final LEDStrip kRGBCANdle = new LEDStrip(0, 8);
        public static final LEDStrip kRGBSection1 = new LEDStrip(8, 13);
        public static final LEDStrip kRGBSection2 = new LEDStrip(21, 12);
        public static final LEDStrip kRGBSection3 = new LEDStrip(33, 10);
        public static final int kRGBCount = kRGBCANdle.m_length + kRGBSection1.m_length
                                            + kRGBSection2.m_length + kRGBSection3.m_length;

    }
}
