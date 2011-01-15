package resource.can;

import edu.wpi.first.wpilibj.SpeedController;

public class CANJaguar implements JaguarCANProtocol, SpeedController {

    // The internal PID control loop in the Jaguar runs at 1kHz.
    public static final int kControllerRate = 1000;
    public static final double kApproxBusVoltage = 12.0;

    
    /**
     * Determines which sensor to use for position reference.
     */
    public static class PositionReference {

        public final byte value;
        static final byte kEncoder_val = 0;
        static final byte kPotentiometer_val = 1;
        public static final PositionReference kEncoder = new PositionReference(kEncoder_val);
        public static final PositionReference kPotentiometer = new PositionReference(kPotentiometer_val);

        private PositionReference(byte value) {
            this.value = value;
        }
    }

    /**
     * Determines which sensor to use for speed reference.
     * Only one option for now.
     */
    public static class SpeedReference {

        public final byte value;
        static final byte kEncoder_val = 0;
        public static final SpeedReference kEncoder = new SpeedReference(kEncoder_val);

        private SpeedReference(byte value) {
            this.value = value;
        }
    }

    /**
     * Determines how the Jaguar behaves when sending a zero signal.
     */
    public static class NeutralMode {

        public final byte value;
        static final byte kJumper_val = 0;
        static final byte kBrake_val = 1;
        static final byte kCoast_val = 2;
        public static final NeutralMode kJumper = new NeutralMode(kJumper_val);
        public static final NeutralMode kBrake = new NeutralMode(kBrake_val);
        public static final NeutralMode kCoast = new NeutralMode(kCoast_val);

        private NeutralMode(byte value) {
            this.value = value;
        }
    }

    /**
     * Determines which sensor to use for position reference.
     */
    public static class LimitMode {

        public final byte value;
        static final byte kSwitchInputsOnly_val = 0;
        static final byte kSoftPositionLimit_val = 1;
        public static final LimitMode kSwitchInputsOnly = new LimitMode(kSwitchInputsOnly_val);
        public static final LimitMode kSoftPostionLimits = new LimitMode(kSoftPositionLimit_val);

        private LimitMode(byte value) {
            this.value = value;
        }
    }

    /**
     * Mode determines how the Jaguar is controlled
     */
    public static class ControlMode {

        public final int value;
        static final int kPercentVoltage_val = 0;
        static final int kSpeed_val = 1;
        static final int kPosition_val = 2;
        static final int kCurrent_val = 3;
        public static final ControlMode kPercentVoltage = new ControlMode(kPercentVoltage_val);
        public static final ControlMode kSpeed = new ControlMode(kSpeed_val);
        public static final ControlMode kPosition = new ControlMode(kPosition_val);
        public static final ControlMode kCurrent = new ControlMode(kCurrent_val);

        private ControlMode(int value) {
            this.value = value;
        }
    }

    /**
     * Faults reported by the Jaguar
     */
    public static class Faults {

        public final int value;
        static final int kCurrentFault_val = 1;
        static final int kTemperatureFault_val = 2;
        static final int kBusVoltageFault_val = 4;
        public static final Faults kCurrentFault = new Faults(kCurrentFault_val);
        public static final Faults kTemperatureFault = new Faults(kTemperatureFault_val);
        public static final Faults kBusVoltageFault = new Faults(kBusVoltageFault_val);

        private Faults(int value) {
            this.value = value;
        }
    }

    /**
     * Limit switch masks
     */
    public static class Limits {

        public final int value;
        static final int kForwardLimit_val = 1;
        static final int kReverseLimit_val = 2;
        public static final Limits kForwardLimit = new Limits(kForwardLimit_val);
        public static final Limits kReverseLimit = new Limits(kReverseLimit_val);

        private Limits(int value) {
            this.value = value;
        }
    }
    private final Object m_transactionSemaphore = new Object();
    private char m_deviceNumber;
    private ControlMode m_controlMode;
    private double m_maxOutputVoltage;
    private static final byte[] kNoData = new byte[0];

    private final static int swap16(int x) {
        return ((((x) >>> 8) & 0x00FF) | (((x) << 8) & 0xFF00));
    }

    private final static long swap32(long x) {
        return ((((x) >> 24) & 0x000000FF) | (((x) >> 8) & 0x0000FF00) | (((x) << 8) & 0x00FF0000) | (((x) << 24) & 0xFF000000));
    }

    private final static int swap16(int x, byte[] buffer) {
        buffer[0] = (byte) x;
        buffer[1] = (byte) ((x >>> 8) & 0x00FF);
        return ((((x) >>> 8) & 0x00FF) | (((x) << 8) & 0xFF00));
    }

    private final static long swap32(long x, byte[] buffer) {
        buffer[0] = (byte) x;
        buffer[1] = (byte) ((x >>> 8) & 0x00FF);
        buffer[2] = (byte) ((x >>> 16) & 0x00FF);
        buffer[3] = (byte) ((x >>> 24) & 0x00FF);
        return ((((x) >> 24) & 0x000000FF) | (((x) >> 8) & 0x0000FF00) | (((x) << 8) & 0x00FF0000) | (((x) << 24) & 0xFF000000));
    }

    private final static int swap16(byte[] buffer) {
        return ((((buffer[1]) >>> 8) & 0x00FF) | (((buffer[0]) << 8) & 0xFF00));
    }

    private final static long swap32(byte[] buffer) {
        return ((((buffer[3]) >> 24) & 0x000000FF) | (((buffer[2]) >> 8) & 0x0000FF00) | (((buffer[1]) << 8) & 0x00FF0000) | (((buffer[0]) << 24) & 0xFF000000));
    }

    /**
     * Pack 16-bit data in little-endian byte order
     * @param data The data to be packed
     * @param buffer The buffer to pack into
     * @param offset The offset into data to pack the variable
     */
    private static final void pack16(short data, byte[] buffer, int offset) {
        buffer[offset] = (byte) (data & 0xFF);
        buffer[offset + 1] = (byte) ((data >>> 8) & 0xFF);
    }

    /**
     * Pack 32-bit data in little-endian byte order
     * @param data The data to be packed
     * @param buffer The buffer to pack into
     * @param offset The offset into data to pack the variable
     */
    private static final void pack32(int data, byte[] buffer, int offset) {
        buffer[offset] = (byte) (data & 0xFF);
        buffer[offset + 1] = (byte) ((data >>> 8) & 0xFF);
        buffer[offset + 2] = (byte) ((data >>> 16) & 0xFF);
        buffer[offset + 3] = (byte) ((data >>> 24) & 0xFF);
    }

    /**
     * Unpack 16-bit data from a buffer in little-endian byte order
     * @param buffer The buffer to unpack from
     * @param offset The offset into he buffer to unpack
     * @return The data that was unpacked
     */
    private static final short unpack16(byte[] buffer, int offset) {
        return (short) (((int) buffer[offset] & 0xFF) | (short) ((buffer[offset + 1] << 8)) & 0xFF00);
    }

    /**
     * Unpack 32-bit data from a buffer in little-endian byte order
     * @param buffer The buffer to unpack from
     * @param offset The offset into he buffer to unpack
     * @return The data that was unpacked
     */
    private static final int unpack32(byte[] buffer, int offset) {
        return ((int) buffer[offset] & 0xFF) | ((buffer[offset + 1] << 8) & 0xFF00) |
                ((buffer[offset + 2] << 16) & 0xFF0000) | ((buffer[offset + 3] << 24) & 0xFF000000);
    }
    private static final int kFullMessageIDMask = 0x1FFFFFC0;

    /**
     * Common initialization code called by all constructors.
     */
    private void initJaguar() {
        if (m_deviceNumber < 1 || m_deviceNumber > 63) {
            return;
        }

        //long fwVer = getFirmwareVersion();
        //System.out.println("fwVersion[" + m_deviceNumber + "]: " + fwVer);
        //if (fwVer >= 3330 || fwVer < 87)
        //{
        //    System.out.println("ERROR: The Jaguar firmware must be updated!");
        //    return;
        //}

        switch (m_controlMode.value) {
            case ControlMode.kPercentVoltage_val:
                sendMessage(LM_API_VOLT_T_EN | m_deviceNumber, kNoData, 0);
                break;
            case ControlMode.kSpeed_val:
                setSpeedReference(SpeedReference.kEncoder);
                break;
            default:
                return;
        }
    }

    /**
     * Constructor
     * Default to voltage control mode.
     * @param deviceNumber The the address of the Jaguar on the CAN bus.
     */
    public CANJaguar(int deviceNumber) {
        m_deviceNumber = (char) deviceNumber;
        m_controlMode = ControlMode.kPercentVoltage;
        m_maxOutputVoltage = kApproxBusVoltage;
        initJaguar();
    }

    /**
     * Constructor
     * @param deviceNumber The the address of the Jaguar on the CAN bus.
     * @param controlMode The control mode that the Jaguar will run in.
     */
    public CANJaguar(int deviceNumber, ControlMode controlMode) {
        m_deviceNumber = (char) deviceNumber;
        m_controlMode = controlMode;
        m_maxOutputVoltage = kApproxBusVoltage;
        initJaguar();
    }

    /**
     * Set the output set-point value.
     *
     * In PercentVoltage Mode, the input is in the range -1.0 to 1.0
     *
     * @param outputValue The set-point to sent to the motor controller.
     */
    public void set(double outputValue) {
        int messageID = 0;
        byte[] dataBuffer = new byte[8];
        byte dataSize = 0;

        switch (m_controlMode.value) {
            case ControlMode.kPercentVoltage_val:
                messageID = LM_API_VOLT_T_SET;
                packPercentage(dataBuffer, outputValue);
                dataSize = 2;
                break;
            case ControlMode.kSpeed_val: {
                messageID = LM_API_SPD_T_SET;
                dataSize = packFXP16_16(dataBuffer, outputValue);
            }
            break;
            case ControlMode.kPosition_val: {
                messageID = LM_API_POS_T_SET;
                dataSize = packFXP16_16(dataBuffer, outputValue);
            }
            break;
            case ControlMode.kCurrent_val: {
                messageID = LM_API_ICTRL_T_SET;
                dataSize = packFXP8_8(dataBuffer, outputValue);
            }
            break;
            default:
                return;
        }
        setTransaction(messageID, dataBuffer, dataSize);
    }

    public void set(double outputValue, byte syncGroup)
    {
        int messageID = 0;
        byte[] dataBuffer = new byte[8];
        byte dataSize = 0;

        switch (m_controlMode.value) {
            case ControlMode.kPercentVoltage_val:
                messageID = LM_API_VOLT_T_SET;
                packPercentage(dataBuffer, outputValue);
                dataSize = 2;
                break;
            case ControlMode.kSpeed_val: {
                messageID = LM_API_SPD_T_SET;
                dataSize = packFXP16_16(dataBuffer, outputValue);
            }
            break;
            case ControlMode.kPosition_val: {
                messageID = LM_API_POS_T_SET;
                dataSize = packFXP16_16(dataBuffer, outputValue);
            }
            break;
            case ControlMode.kCurrent_val: {
                messageID = LM_API_ICTRL_T_SET;
                dataSize = packFXP8_8(dataBuffer, outputValue);
            }
            break;
            default:
                return;
        }
        setTransaction(messageID, dataBuffer, dataSize);
    }
    public void disable()
    {
        return;
    }

    /**
     * Get the recently set outputValue setpoint.
     *
     * In PercentVoltage Mode, the outputValue is in the range -1.0 to 1.0
     *
     * @return The most recently set outputValue setpoint.
     */
    public double get() {
        byte[] dataBuffer = new byte[8];
        byte dataSize = 0;

        switch (m_controlMode.value) {
            case ControlMode.kPercentVoltage_val:
                dataSize = getTransaction(LM_API_VOLT_SET, dataBuffer);
                if (dataSize == 2) {
                    return unpackPercentage(dataBuffer);
                }
                break;
            case ControlMode.kSpeed_val:
                dataSize = getTransaction(LM_API_SPD_SET, dataBuffer);
                if (dataSize == 4) {
                    return unpackFXP16_16(dataBuffer);
                }
                break;
            case ControlMode.kPosition_val:
                dataSize = getTransaction(LM_API_POS_SET, dataBuffer);
                if (dataSize == 4) {
                    return unpackFXP16_16(dataBuffer);
                }
                break;
            case ControlMode.kCurrent_val:
                dataSize = getTransaction(LM_API_ICTRL_SET, dataBuffer);
                if (dataSize == 2) {
                    return unpackFXP8_8(dataBuffer);
                }
                break;
        }
        return 0.0;

    }

    /**
     * Write out the PID value as seen in the PIDOutput base object.
     *
     * @param output Write out the percentage voltage value as was computed by the PIDController
     */
    public void pidWrite(double output) {
        if (m_controlMode == ControlMode.kPercentVoltage) {
            set(output);
        } else {
            // TODO: Error... only voltage mode supported for PID API
        }
    }

    byte packPercentage(byte[] buffer, double value) {
        short intValue = (short) (value * 32767.0);
        swap16(intValue, buffer);
        return 2;
    }

    byte packFXP8_8(byte[] buffer, double value) {
        short intValue = (short) (value * 256.0);
        swap16(intValue, buffer);
        return 2;
    }

    byte packFXP16_16(byte[] buffer, double value) {
        int intValue = (int) (value * 65536.0);
        swap32(intValue, buffer);
        return 4;
    }

    byte packINT16(byte[] buffer, short value) {
        swap16(value, buffer);
        return 2;
    }

    byte packINT32(byte[] buffer, int value) {
        swap32(value, buffer);
        return 4;
    }

    double unpackPercentage(byte[] buffer) {
        return unpack16(buffer,0) / 32767.0;
    }

    double unpackFXP8_8(byte[] buffer) {
        return unpack16(buffer,0) / 256.0;
    }

    double unpackFXP16_16(byte[] buffer) {
        return unpack32(buffer,0) / 65536.0;
    }

    int unpackINT16(byte[] buffer) {
        return unpack16(buffer,0);
    }

    long unpackINT32(byte[] buffer) {
        return unpack32(buffer,0);
    }
    private static final byte[] sendTrustedDataBuffer = new byte[JaguarCANDriver.kMaxMessageDataSize];

    /**
     * Send a message on the CAN bus through the CAN driver in FRC_NetworkCommunication
     *
     * Trusted messages require a 2-byte token at the beginning of the data payload.
     * If the message being sent is trusted, make space for the token.
     *
     * @param messageID The messageID to be used on the CAN bus
     * @param data The up to 8 bytes of data to be sent with the message
     * @param dataSize Specify how much of the data in "data" to send
     */
    protected void sendMessage(int messageID, byte[] data, int dataSize) {
        final int[] kTrustedMessages = {
            LM_API_VOLT_T_EN, LM_API_VOLT_T_SET, LM_API_SPD_T_EN, LM_API_SPD_T_SET,
            LM_API_POS_T_EN, LM_API_POS_T_SET, LM_API_ICTRL_T_EN, LM_API_ICTRL_T_SET};

        byte i;
        for (i = 0; i < kTrustedMessages.length; i++) {
            if ((kFullMessageIDMask & messageID) == kTrustedMessages[i]) {
                sendTrustedDataBuffer[0] = 0;
                sendTrustedDataBuffer[1] = 0;
                // Make sure the data will still fit after adjusting for the token.
                if (dataSize > JaguarCANDriver.kMaxMessageDataSize - 2) {
                    // TODO: Error
                    return;
                }

                byte j;
                for (j = 0; j <
                        dataSize; j++) {
                    sendTrustedDataBuffer[j + 2] = data[j];
                }

                JaguarCANDriver.sendMessage(messageID, sendTrustedDataBuffer, dataSize + 2);
                return;
            }
        }
        JaguarCANDriver.sendMessage(messageID, data, dataSize);
    }

    /**
     * Receive a message from the CAN bus through the CAN driver in FRC_NetworkCommunication
     *
     * @param messageID The messageID to read from the CAN bus
     * @param data The up to 8 bytes of data that was received with the message
     * @param dataSize Indicates how much data was received
     * @param timeout Specify how long to wait for a message (in seconds)
     */
    protected byte receiveMessage(int messageID, byte[] data, double timeout) {
        JaguarCANDriver canDriver = new JaguarCANDriver();
        byte dataSize = canDriver.receiveMessage(messageID, data, timeout);
        return dataSize;
    }

    protected byte receiveMessage(int messageID, byte[] data) {
        return receiveMessage(messageID, data, 0.1);
    }

    /**
     * Execute a transaction with a Jaguar that sets some property.
     *
     * Jaguar always acks when it receives a message.  If we don't wait for an ack,
     * the message object in the Jaguar could get overwritten before it is handled.
     *
     * @param messageID The messageID to be used on the CAN bus (device number is added internally)
     * @param data The up to 8 bytes of data to be sent with the message
     * @param dataSize Specify how much of the data in "data" to send
     */
    byte setTransaction(int messageID, byte[] data, byte dataSize) {
        int ackMessageID = LM_API_ACK | m_deviceNumber;

        // Make sure we don't have more than one transaction with the same Jaguar outstanding.
        synchronized (m_transactionSemaphore) {
            // Throw away any stale acks.
//            receiveMessage(ackMessageID, kNoData, 10.0);
            // Send the message with the data.
            sendMessage(messageID | m_deviceNumber, data, dataSize);
            // Wait for an ack.
//            dataSize = receiveMessage(ackMessageID, kNoData, 0.0);
        }
        return dataSize;
    }

    /**
     * Execute a transaction with a Jaguar that gets some property.
     *
     * Jaguar always generates a message with the same message ID when replying.
     *
     * @param messageID The messageID to read from the CAN bus (device number is added internally)
     * @param data The up to 8 bytes of data that was received with the message
     * @param dataSize Indicates how much data was received
     */
    byte getTransaction(int messageID, byte[] data) {
        int targetedMessageID = messageID | m_deviceNumber;
        byte dataSize = 0;
        synchronized (m_transactionSemaphore) {
            // Make sure we don't have more than one transaction with the same Jaguar outstanding.

            // Send the message requesting data.
            sendMessage(targetedMessageID, kNoData, 0);
            // Wait for the data.
            dataSize = receiveMessage(targetedMessageID, data);

        }
        return dataSize;

    }

    /**
     * Set the reference source device for speed controller mode.
     *
     * Choose encoder as the source of speed feedback when in speed control mode.
     * This is curently the only possible value, so we'll just call it for you in the constructor.
     *
     * @param reference Specify a SpeedReference.
     */
    private void setSpeedReference(SpeedReference reference) {
        byte[] dataBuffer = new byte[8];

        switch (m_controlMode.value) {
            case ControlMode.kSpeed_val:
                dataBuffer[0] = reference.value;
                setTransaction(LM_API_SPD_REF, dataBuffer, (byte) 1);
                break;
            default:
                // TODO: Error, Invalid
                return;
        }
    }

    /**
     * Set the reference source device for position controller mode.
     *
     * Choose between using and encoder and using a potentiometer
     * as the source of position feedback when in position control mode.
     *
     * @param reference Specify a PositionReference.
     */
    public void setPositionReference(PositionReference reference) {
        byte[] dataBuffer = new byte[8];

        switch (m_controlMode.value) {
            case ControlMode.kPosition_val:
                dataBuffer[0] = reference.value;
                setTransaction(LM_API_POS_REF, dataBuffer, (byte) 1);
                break;
            default:
                // TODO: Error, Invalid
                return;
        }
    }

    /**
     * Get the reference source device for position controller mode.
     *
     * @return A PositionReference indicating the currently selected reference device for position controller mode.
     */
    public PositionReference getPositionReference() {
        byte[] dataBuffer = new byte[8];
        byte dataSize = 0;

        switch (m_controlMode.value) {
            case ControlMode.kPosition_val:
                dataSize = getTransaction(LM_API_POS_REF, dataBuffer);
                if (dataSize == 1) {
                    switch (dataBuffer[0]) {
                        case PositionReference.kPotentiometer_val:
                            return PositionReference.kPotentiometer;
                        case PositionReference.kEncoder_val:
                            return PositionReference.kEncoder;
                    }
                }
                break;
            default:
                // TODO: Error, Invalid
                break;
        }
        return PositionReference.kEncoder;
    }

    /**
     * Set the P, I, and D constants for the closed loop modes.
     *
     * @param p The proportional gain of the Jaguar's PID controller.
     * @param i The integral gain of the Jaguar's PID controller.
     * @param d The differential gain of the Jaguar's PID controller.
     */
    public void setPID(double p, double i, double d) {
        byte[] dataBuffer = new byte[8];
        byte dataSize;

        switch (m_controlMode.value) {
            case ControlMode.kPercentVoltage_val:
                // TODO: Error, Not Valid
                break;
            case ControlMode.kSpeed_val:
                dataSize = packFXP16_16(dataBuffer, p);
                setTransaction(LM_API_SPD_PC, dataBuffer, dataSize);
                dataSize = packFXP16_16(dataBuffer, i);
                setTransaction(LM_API_SPD_IC, dataBuffer, dataSize);
                dataSize = packFXP16_16(dataBuffer, d);
                setTransaction(LM_API_SPD_DC, dataBuffer, dataSize);
                break;
            case ControlMode.kPosition_val:
                dataSize = packFXP16_16(dataBuffer, p);
                setTransaction(LM_API_POS_PC, dataBuffer, dataSize);
                dataSize = packFXP16_16(dataBuffer, i);
                setTransaction(LM_API_POS_IC, dataBuffer, dataSize);
                dataSize = packFXP16_16(dataBuffer, d);
                setTransaction(LM_API_POS_DC, dataBuffer, dataSize);
                break;
            case ControlMode.kCurrent_val:
                dataSize = packFXP16_16(dataBuffer, p);
                setTransaction(LM_API_ICTRL_PC, dataBuffer, dataSize);
                dataSize = packFXP16_16(dataBuffer, i);
                setTransaction(LM_API_ICTRL_IC, dataBuffer, dataSize);
                dataSize = packFXP16_16(dataBuffer, d);
                setTransaction(LM_API_ICTRL_DC, dataBuffer, dataSize);
                break;
        }
    }

    /**
     * Get the Proportional gain of the controller.
     *
     * @return The proportional gain.
     */
    public double getP() {
        byte[] dataBuffer = new byte[8];
        byte dataSize = 0;

        switch (m_controlMode.value) {
            case ControlMode.kPercentVoltage_val:
                // TODO: Error, Not Valid
                break;
            case ControlMode.kSpeed_val:
                dataSize = getTransaction(LM_API_SPD_PC, dataBuffer);
                if (dataSize == 4) {
                    return unpackFXP16_16(dataBuffer);
                }
                break;
            case ControlMode.kPosition_val:
                dataSize = getTransaction(LM_API_POS_PC, dataBuffer);
                if (dataSize == 4) {
                    return unpackFXP16_16(dataBuffer);
                }
                break;
            case ControlMode.kCurrent_val:
                dataSize = getTransaction(LM_API_ICTRL_PC, dataBuffer);
                if (dataSize == 4) {
                    return unpackFXP16_16(dataBuffer);
                }
                break;
        }
        return 0.0;
    }

    /**
     * Get the Intregral gain of the controller.
     *
     * @return The integral gain.
     */
    public double getI() {
        byte[] dataBuffer = new byte[8];
        byte dataSize = 0;

        switch (m_controlMode.value) {
            case ControlMode.kPercentVoltage_val:
                // TODO: Error, Not Valid
                break;
            case ControlMode.kSpeed_val:
                dataSize = getTransaction(LM_API_SPD_IC, dataBuffer);
                if (dataSize == 4) {
                    return unpackFXP16_16(dataBuffer);
                }
                break;
            case ControlMode.kPosition_val:
                dataSize = getTransaction(LM_API_POS_IC, dataBuffer);
                if (dataSize == 4) {
                    return unpackFXP16_16(dataBuffer);
                }
                break;
            case ControlMode.kCurrent_val:
                dataSize = getTransaction(LM_API_ICTRL_IC, dataBuffer);
                if (dataSize == 4) {
                    return unpackFXP16_16(dataBuffer);
                }
                break;
        }
        return 0.0;
    }

    /**
     * Get the Differential gain of the controller.
     *
     * @return The differential gain.
     */
    public double getD() {
        byte[] dataBuffer = new byte[8];
        byte dataSize = 0;

        switch (m_controlMode.value) {
            case ControlMode.kPercentVoltage_val:
                // TODO: Error, Not Valid
                break;
            case ControlMode.kSpeed_val:
                dataSize = getTransaction(LM_API_SPD_DC, dataBuffer);
                if (dataSize == 4) {
                    return unpackFXP16_16(dataBuffer);
                }
                break;
            case ControlMode.kPosition_val:
                dataSize = getTransaction(LM_API_POS_DC, dataBuffer);
                if (dataSize == 4) {
                    return unpackFXP16_16(dataBuffer);
                }
                break;
            case ControlMode.kCurrent_val:
                dataSize = getTransaction(LM_API_ICTRL_DC, dataBuffer);
                if (dataSize == 4) {
                    return unpackFXP16_16(dataBuffer);
                }
                break;
        }
        return 0.0;
    }

    /**
     * Enable the closed loop controller.
     *
     * Start actually controlling the output based on the feedback.
     * If starting a position controller with an encoder reference,
     * use the encoderInitialPosition parameter to initialize the
     * encoder state.
     * @param encoderInitialPosition Encoder position to set if position with encoder reference.  Ignored otherwise.
     */
    public void enableControl(double encoderInitialPosition) {
        byte[] dataBuffer = new byte[8];
        byte dataSize = 0;

        switch (m_controlMode.value) {
            case ControlMode.kPercentVoltage_val:
                // TODO: Error, Not Valid
                break;
            case ControlMode.kSpeed_val:
                setTransaction(LM_API_SPD_T_EN, dataBuffer, dataSize);
                break;
            case ControlMode.kPosition_val:
                dataSize = packFXP16_16(dataBuffer, encoderInitialPosition);
                setTransaction(LM_API_POS_T_EN, dataBuffer, dataSize);
                break;
            case ControlMode.kCurrent_val:
                setTransaction(LM_API_ICTRL_T_EN, dataBuffer, dataSize);
                break;
        }
    }

    /**
     * Disable the closed loop controller.
     *
     * Stop driving the output based on the feedback.
     */
    public void disableControl() {
        byte[] dataBuffer = new byte[8];
        byte dataSize = 0;

        switch (m_controlMode.value) {
            case ControlMode.kPercentVoltage_val:
                // TODO: Error, Not Valid
                break;
            case ControlMode.kSpeed_val:
                setTransaction(LM_API_SPD_DIS, dataBuffer, dataSize);
                break;
            case ControlMode.kPosition_val:
                setTransaction(LM_API_POS_DIS, dataBuffer, dataSize);
                break;
            case ControlMode.kCurrent_val:
                setTransaction(LM_API_ICTRL_DIS, dataBuffer, dataSize);
                break;
        }
    }

    /**
     * Get the voltage at the battery input terminals of the Jaguar.
     *
     * @return The bus voltage in Volts.
     */
    public double getBusVoltage() {
        byte[] dataBuffer = new byte[8];
        byte dataSize = 0;

        dataSize = getTransaction(LM_API_STATUS_VOLTBUS, dataBuffer);
        if (dataSize == 2) {
            return unpackFXP8_8(dataBuffer);
        }
        return 0.0;
    }

    /**
     * Get the voltage being output from the motor terminals of the Jaguar.
     *
     * @return The output voltage in Volts.
     */
    public double getOutputVoltage() {
        byte[] dataBuffer = new byte[8];
        byte dataSize = 0;
        double busVoltage;

        // Read the bus voltage first so we can return units of volts
        busVoltage = getBusVoltage();
        // Then read the volt out which is in percentage of bus voltage units.
        dataSize = getTransaction(LM_API_STATUS_VOLTOUT, dataBuffer);
        if (dataSize == 2) {
            return (m_maxOutputVoltage / kApproxBusVoltage) * busVoltage * unpackPercentage(dataBuffer);
        }
        return 0.0;
    }

    /**
     * Get the current through the motor terminals of the Jaguar.
     *
     * @return The output current in Amps.
     */
    public double getOutputCurrent() {
        byte[] dataBuffer = new byte[8];
        byte dataSize;

        dataSize = getTransaction(LM_API_STATUS_CURRENT, dataBuffer);
        if (dataSize == 2) {
            return unpackFXP8_8(dataBuffer);
        }
        return 0.0;

    }

    /**
     * Get the internal temperature of the Jaguar.
     *
     * @return The temperature of the Jaguar in degrees Celsius.
     */
    public double getTemperature() {
        byte[] dataBuffer = new byte[8];
        byte dataSize;

        dataSize = getTransaction(LM_API_STATUS_TEMP, dataBuffer);
        if (dataSize == 2) {
            return unpackFXP8_8(dataBuffer);
        }
        return 0.0;
    }

    /**
     * Get the position of the encoder or potentiometer.
     *
     * @return The position of the motor based on the configured feedback.
     */
    public double getPosition() {
        byte[] dataBuffer = new byte[8];
        byte dataSize;

        dataSize = getTransaction(LM_API_STATUS_POS, dataBuffer);
        if (dataSize == 4) {
            return unpackFXP16_16(dataBuffer);
        }
        return 0.0;
    }

    /**
     * Get the speed of the encoder.
     *
     * @return The speed of the motor in RPM based on the configured feedback.
     */
    public double getSpeed() {
        byte[] dataBuffer = new byte[8];
        byte dataSize;

        dataSize = getTransaction(LM_API_STATUS_SPD, dataBuffer);
        if (dataSize == 4) {
            return unpackFXP16_16(dataBuffer);
        }
        return 0.0;
    }

    /**
     * Get the status of the forward limit switch.
     *
     * @return The motor is allowed to turn in the forward direction when true.
     */
    public boolean getForwardLimitOK() {
        byte[] dataBuffer = new byte[8];
        byte dataSize;

        dataSize = getTransaction(LM_API_STATUS_LIMIT, dataBuffer);
        if (dataSize == 1) {
            return (dataBuffer[0] & Limits.kForwardLimit_val) != 0;
        }
        return false;
    }

    /**
     * Get the status of the reverse limit switch.
     *
     * @return The motor is allowed to turn in the reverse direction when true.
     */
    public boolean getReverseLimitOK() {
        byte[] dataBuffer = new byte[8];
        byte dataSize;

        dataSize = getTransaction(LM_API_STATUS_LIMIT, dataBuffer);
        if (dataSize == 1) {
            return (dataBuffer[0] & Limits.kReverseLimit_val) != 0;
        }
        return false;
    }

    /**
     * Get the status of any faults the Jaguar has detected.
     *
     * @return A bit-mask of faults defined by the "Faults" enum class.
     */
    public short getFaults() {
        byte[] dataBuffer = new byte[8];
        byte dataSize;

        dataSize = getTransaction(LM_API_STATUS_FAULT, dataBuffer);
        if (dataSize == 2) {
            return (short)unpackINT16(dataBuffer);
        }
        return 0;
    }

    /**
     * Check if the Jaguar's power has been cycled since this was last called.
     *
     * This should return true the first time called after a Jaguar power up,
     * and false after that.
     *
     * @return The Jaguar was power cycled since the last call to this function.
     */
    public boolean getPowerCycled() {
        byte[] dataBuffer = new byte[8];
        byte dataSize;

        dataSize = getTransaction(LM_API_STATUS_POWER, dataBuffer);
        if (dataSize == 1) {
            boolean powerCycled = dataBuffer[0] != 0;

            // Clear the power cycled bit now that we've accessed it
            dataBuffer[0] = 1;
            setTransaction(LM_API_STATUS_POWER, dataBuffer, (byte) 1);

            return powerCycled;
        }
        return false;
    }

    /**
     * Set the maximum voltage change rate.
     *
     * When in percent voltage output mode, the rate at which the voltage changes can
     * be limited to reduce current spikes.  Set this to 0.0 to disable rate limiting.
     *
     * @param rampRate The maximum rate of voltage change in Percent Voltage mode in V/s.
     */
    public void setVoltageRampRate(double rampRate) {
        byte[] dataBuffer = new byte[8];
        byte dataSize;

        switch (m_controlMode.value) {
            case ControlMode.kPercentVoltage_val:
                dataSize = packPercentage(dataBuffer, rampRate / (m_maxOutputVoltage * kControllerRate));
                setTransaction(LM_API_VOLT_SET_RAMP, dataBuffer, dataSize);
                break;
            default:
                return;
        }
    }

    /**
     * Get the version of the firmware running on the Jaguar.
     *
     * @return The firmware version.  0 if the device did not respond.
     */
    public long getFirmwareVersion() {
        byte[] dataBuffer = new byte[8];
        byte dataSize;

        // Set the MSB to tell the 2CAN that this is a remote message.
        dataSize = getTransaction(0x80000000 | CAN_MSGID_API_FIRMVER, dataBuffer);
        if (dataSize == 4) {
            return unpackINT32(dataBuffer);
        }
        return 0;
    }

    /**
     * Get the version of the Jaguar hardware.
     *
     * @return The hardware version. 1: Jaguar,  2: Black Jaguar
     */
    public byte getHardwareVersion() {
        byte[] dataBuffer = new byte[8];
        byte dataSize;

        dataSize = getTransaction(LM_API_HWVER, dataBuffer);
        if (dataSize == 1 + 1) {
            if (dataBuffer[0] == m_deviceNumber) {
                return dataBuffer[1];
            }
        }
        // Assume Gray Jag if there is no response
        return LM_HWVER_JAG_1_0;
    }

    /**
     * Configure what the controller does to the H-Bridge when neutral (not driving the output).
     *
     * This allows you to override the jumper configuration for brake or coast.
     *
     * @param mode Select to use the jumper setting or to override it to coast or brake.
     */
    public void configNeutralMode(NeutralMode mode) {
        byte[] dataBuffer = new byte[8];

        dataBuffer[0] = mode.value;
        setTransaction(LM_API_CFG_BRAKE_COAST, dataBuffer, (byte) 1);
    }

    /**
     * Configure how many codes per revolution are generated by your encoder.
     *
     * @param codesPerRev The number of counts per revolution in 1X mode.
     */
    public void configEncoderCodesPerRev(short codesPerRev) {
        byte[] dataBuffer = new byte[8];
        byte dataSize;

        dataSize = packINT16(dataBuffer, codesPerRev);
        setTransaction(LM_API_CFG_ENC_LINES, dataBuffer, dataSize);
    }

    /**
     * Configure the number of turns on the potentiometer.
     *
     * There is no special support for continuous turn potentiometers.
     * Only integer numbers of turns are supported.
     *
     * @param turns The number of turns of the potentiometer
     */
    public void configPotentiometerTurns(short turns) {
        byte[] dataBuffer = new byte[8];
        byte dataSize;

        dataSize = packINT16(dataBuffer, turns);
        setTransaction(LM_API_CFG_POT_TURNS, dataBuffer, dataSize);
    }

    /**
     * Configure Soft Position Limits when in Position Controller mode.
     *
     * When controlling position, you can add additional limits on top of the limit switch inputs
     * that are based on the position feedback.  If the position limit is reached or the
     * switch is opened, that direction will be disabled.
     *
     * @param forwardLimitPosition The position that if exceeded will disable the forward direction.
     * @param reverseLimitPosition The position that if exceeded will disable the reverse direction.
     */
    public void configSoftPositionLimits(double forwardLimitPosition, double reverseLimitPosition) {
        byte[] dataBuffer = new byte[8];
        byte dataSize;

        dataSize = packFXP16_16(dataBuffer, forwardLimitPosition);
        dataBuffer[dataSize++] = (forwardLimitPosition > reverseLimitPosition) ? (byte) 1 : (byte) 0;
        setTransaction(LM_API_CFG_LIMIT_FWD, dataBuffer, dataSize);

        dataSize = packFXP16_16(dataBuffer, reverseLimitPosition);
        dataBuffer[dataSize++] = forwardLimitPosition <= reverseLimitPosition ? (byte) 1 : (byte) 0;
        setTransaction(LM_API_CFG_LIMIT_REV, dataBuffer, dataSize);

        dataBuffer[0] = LimitMode.kSoftPositionLimit_val;
        setTransaction(LM_API_CFG_LIMIT_MODE, dataBuffer, (byte) 1);
    }

    /**
     * Disable Soft Position Limits if previously enabled.
     *
     * Soft Position Limits are disabled by default.
     */
    public void disableSoftPositionLimits() {
        byte[] dataBuffer = new byte[8];

        dataBuffer[0] = LimitMode.kSwitchInputsOnly_val;
        setTransaction(LM_API_CFG_LIMIT_MODE, dataBuffer, (byte) 1);
    }

    /**
     * Configure the maximum voltage that the Jaguar will ever output.
     *
     * This can be used to limit the maximum output voltage in all modes so that
     * motors which cannot withstand full bus voltage can be used safely.
     *
     * @param voltage The maximum voltage output by the Jaguar.
     */
    public void configMaxOutputVoltage(double voltage) {
        byte[] dataBuffer = new byte[8];
        byte dataSize;

        m_maxOutputVoltage = voltage;
        dataSize = packFXP8_8(dataBuffer, voltage);
        setTransaction(LM_API_CFG_MAX_VOUT, dataBuffer, dataSize);
    }

    /**
     * Configure how long the Jaguar waits in the case of a fault before resuming operation.
     *
     * Faults include over temerature, over current, and bus under voltage.
     * The default is 3.0 seconds, but can be reduced to as low as 0.5 seconds.
     *
     * @param faultTime The time to wait before resuming operation, in seconds.
     */
    public void configFaultTime(double faultTime) {
        byte[] dataBuffer = new byte[8];
        byte dataSize;

        // Message takes ms
        dataSize = packINT16(dataBuffer, (short) (faultTime * 1000.0));
        setTransaction(LM_API_CFG_FAULT_TIME, dataBuffer, dataSize);
    }
}
