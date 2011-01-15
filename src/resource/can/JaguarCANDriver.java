package resource.can;

import com.sun.cldc.jna.*;
import com.sun.cldc.jna.ptr.IntByReference;
import edu.wpi.first.wpilibj.util.*;

public class JaguarCANDriver {

    public static final int kMaxMessageDataSize = 8;
    private static final Object syncObject = new Object();
    // void FRC_NetworkCommunication_JaguarCANDriver_sendMessage(UINT32 messageID, const UINT8 *data, UINT8 dataSize, INT32 *status);
    private static final Function sendMessageFn = NativeLibrary.getDefaultInstance().getFunction("FRC_NetworkCommunication_JaguarCANDriver_sendMessage");
    private static final IntByReference sendStatus = new IntByReference(0);
    private static final Pointer sendDataBufferPointer = new Pointer(kMaxMessageDataSize);

    /**
     * Send a message on the CAN bus
     * @param messageID CAN MessageID to send on the CAN
     * @param data Data payload to send with the message
     */
    public static synchronized void sendMessage(int messageID, byte[] data, int dataSize) {
        sendDataBufferPointer.setBytes(0, data, 0, dataSize);
        sendStatus.setValue(0);
        sendMessageFn.call4(messageID, sendDataBufferPointer, dataSize, sendStatus.getPointer().address().toUWord().toPrimitive());
        if (sendStatus.getValue() != 0) {
            throw new UncleanStatusException(sendStatus.getValue(), "Fatal status code detected:  " + Integer.toString(sendStatus.getValue()));
        }
    }
    // void FRC_NetworkCommunication_JaguarCANDriver_receiveMessage(UINT32 *messageID, UINT8 *data, UINT8 *dataSize, UINT32 timeoutMs, INT32 *status);
    private static final Function receiveMessageFn = NativeLibrary.getDefaultInstance().getFunction("FRC_NetworkCommunication_JaguarCANDriver_receiveMessage");
    private static final Pointer recvDataBufferPointer = new Pointer(kMaxMessageDataSize);
    private static final IntByReference recvStatus = new IntByReference(0);
    private static final Pointer messageIdPtr = new Pointer(4);
    private static final Pointer dataSizePtr = new Pointer(1);

    /**
     * Wait for a message to be received from the CAN bus.
     * @param messageID MessageID filter to specify what message ID to be expected.
     * @param data Buffer for received data
     * @param timeout Number of seconds to wait for the expected message
     * @return Actual size of the valid bytes in data
     */
    public byte receiveMessage(int messageID, byte[] data, double timeout) {
        byte dataSize = 0;
        synchronized (syncObject) {
            recvStatus.setValue(0);
            messageIdPtr.setInt(0, messageID);
            dataSizePtr.setByte(0, (byte) 0);
            receiveMessageFn.call5(messageIdPtr, recvDataBufferPointer, dataSizePtr, (int) (timeout * 1000.0), recvStatus.getPointer().address().toUWord().toPrimitive());
            if (recvStatus.getValue() != 0) {
                throw new UncleanStatusException(recvStatus.getValue(), "Fatal status code detected:  " + Integer.toString(recvStatus.getValue()));
            }
            dataSize = dataSizePtr.getByte(0);
            receivedMessageId = messageIdPtr.getInt(0);
            recvDataBufferPointer.getBytes(0, data, 0, data.length < dataSize ? data.length : dataSize);
        }
        return dataSize;
    }

    /**
     * Call receiveMessage with a default timeout parameter of 100ms
     * @param messageID MessageID filter to specify what message ID to be expected.
     * @param data Buffer for received data
     * @return Actual size of the valid bytes in data
     */
    public byte receiveMessage(int messageID, byte[] data) {
        return receiveMessage(messageID, data, 0.1);
    }
    public int receivedMessageId;
}
