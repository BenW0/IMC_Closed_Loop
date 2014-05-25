#Intelligent Motor Controller Communication Specifications

Matthew Sorensen and Ben Weiss

Winter 2014 // ME 498/599

This document describes the structure and flow of communications between the host Arduino and the slave motor driver 
microcontrollers. The protocol used (tentatively I²C) is assumed to allow communication between the master and a single
slave at a time, and the address information is assumed integrated into the protocol layer.

The master initiates all messages, and all messages terminate with a response acknowledging successful processing 
and (if needed) including the information requested by the message. 

In addition to the following messages, one synchronization line is attached to every processor in the chain. It behaves as
a "move complete" pin, which is pulled high by resistor. To start a move, the Master allows the line to be pulled high. 
After a specified delay, every slave on the bus (and the master, if the master is also executing the move) pulls the line low, 
indicating the move is in progress. When a slave (or master) finishes a move, it tri-states the pin indicating its completion, 
then measures the time until the move complete pin goes high, indicating the start of the next move. This time serves as an 
indicator of how well-synchronized the move was, and can be reported to the master in the status query.

Slave addresses are assigned according to the axis the slave controls, which is set by dip switches on each motor 
controller board. Slave addresses count upwards from a base address, specified in Marlin/Configuration.h and on each slave.

##Messages

### General Message Structure

Messages always consist of two sections - a transmission and a response. The transmission consists of a byte for message type,
a structure of message content, and finishes with a parity byte. The response consists of a response byte, followed by the 
pre-determined number of bytes in the response structure, and concluding with another parity byte. Parity is computed using
a longitudinal xor computation to detect communication errors. Even if the slave has an error, the current implementation requires
it to reply with the expected number of bytes, even if they are just meaningless.

###*Initialize*

This message triggers startup events on the slave, including flushing of the queued move pipeline. Issued mid-print, this message can be used to force a flush of the pipeline. This message clears any error conditions that require a restart to resolve, and forces a release of the Move Complete line.

####Message Content

  * Message ID = IMC_MSG_INITIALIZE – 1 byte
  * Host firmware revision number – 2 bytes.
  * Reserved for future parameters – 6 bytes 

####Response Content

  * Response status – {IMC_RSP_OK, IMC_RSP_UNKNOWN, IMC_RSP_ERROR} – 1 byte
  * Slave hardware revision number – 2 bytes
  * Slave firmware revision number – 2 bytes

###*Status Query*

This message requests information from the slave’s operating state. Querying clears any set error conditions that do not require a restart with an initialization message. Additional error details can be obtained by querying the IMC_PARAM_ERROR_INFO1 and IMC_PARAM_ERROR_INFO2 parameters.

####Message Content

  * Message ID = IMC_MSG_STATUS – 1 byte

####Response Content

  * Message Response – {IMC_RSP_OK, IMC_RSP_UNKNOWN, IMC_RSP_ERROR} – 1 byte
  * Slave status – {IMC_STATUS_OK or error code} – 1 byte
  * Queued moves – uint8

###*Home Axis*

This message causes the slave to perform its homing routine. The homing routine will start immediately and causes the Move Complete pin to be pulled low until homing is complete.

####Message Content

  * Message ID – IMC_MSG_HOME – 1 byte

####Response Content

  * Message Response – {IMC_RSP_OK, IMC_RSP_UNKNOWN, IMC_RSP_ERROR} – 1 byte
  * Initial actuator position in steps - uint32

###*Queue Move*

Adds a queue to the slave’s move queue

####Message Content

  * Message ID – IMC_MSG_QUEUEMOVE – 1 byte
  * Axis move length in steps – int32
  * Total move length in steps – uint32
  * Initial rate in steps/minute – uint32
  * Nominal feedrate in steps/minute – uint32
  * Final rate in steps/minute – uint32
  * Acceleration in (steps/minute*minute) – uint32
  * Step number to stop accelerating at – uint32
  * Step number to start decelerating at – uint32

####Response Content

  * Message Response – {IMC_RSP_OK, IMC_RSP_UNKNOWN, IMC_RSP_ERROR, IMC_RSP_QUEUEFULL} – 1 byte


###*Get Parameter*

Retrieves a controller parameter from the slave.

####Message Content

  * Message ID – IMC_MSG_GETPARAM – 1 byte
  * Param ID – 1 byte

####Response Content

  * Message Response – {IMC_RSP_OK, IMC_RSP_UNKNOWN, IMC_RSP_ERROR} – 1 byte
  * Parameter value – 4 bytes

###*Set Parameter*

Sets a controller parameter on the slave. Assumes consistent byte order between master and slave.

####Message Content

  * Message ID – IMC_MSG_SETPARAM – 1 byte
  * Param ID – 1 byte
  * Value – 4 byte

### Response Content

  * Message Response = {IMC_RSP_OK, IMC_RSP_UNKNOWN, IMC_RSP_ERROR} – 1 byte

###*Quickstop*

Stops all motion on the slave and clears all buffered moves. This is for build cancel, idle timeout, and other non-critical stops that
shouldn't result in further moves.

####Message Content

  * Message ID – IMC_MSG_SETPARAM – 1 byte

### Response Content

  * Message Response = {IMC_RSP_OK, IMC_RSP_UNKNOWN, IMC_RSP_ERROR} – 1 byte

##Slave Parameters

More parameters will be added in future slave firmware revisions to support more interesting control architectures.
*  IMC_PARAM_ERROR_INFO1  – additional error information (error-dependent). Read-only.
*  IMC_PARAM_ERROR_INFO2 – additional error information (error-dependent). Read-only.
*  IMC_PARAM_FLIP_AXISS – flip the axis? (1 = normal, -1 = reverse) Used for enforcing limit switch logic and move direction.
*  IMC_PARAM_HOME_DIR
*  IMC_PARAM_MIN_SOFTWARE_ENDSTOPS – stop motion on limit switch hit
*  IMC_PARAM_MAX_SOFTWARE_ENDSTOPS – stop motion on limit switch hit
*  IMC_PARAM_MIN_LIMIT_EN – enable minimum limit switch
*  IMC_PARAM_MIN_LIMIT_INV – invert minimum limit switch logic
*  IMC_PARAM_MIN_LIMIT_PULLUP – enable pullup on minimum limit switch 
*  IMC_PARAM_MAX_LIMIT_EN – enable minimum limit switch
*  IMC_PARAM_MAX_LIMIT_INV – invert minimum limit switch logic
*  IMC_PARAM_MAX_LIMIT_PULLUP – enable pullup on minimum limit switch
*  IMC_PARAM_MIN_POS - minimum allowable position of the motor
*  IMC_PARAM_MAX_POS - maximum allowable position of the motor
*  IMC_PARAM_HOME_POS - position to assign when home switch trips
*  IMC_PARAM_HOMING_FEEDRATE - feedrate to use when homing (steps/sec)
*  IMC_PARAM_MOTOR_ON – motor driver enabled. Read/write.
*  IMC_PARAM_MOTOR_IDLE_TIMEOUT – time to wait before automatically disabling motors.
*  IMC_PARAM_SLOWDOWN – engage “slowdown” mode so buffers can fill. Not set locally (only via host)
*  IMC_PARAM_LOCATION - Gets/sets the current encoder location. Can be used to manually set location.
*  IMC_PARAM_SYNC_ERROR - (read only) The average time elapsed between move completion (releasing Move Complete pin) and Move Complete pin going high.
*  IMC_PARAM_LAST_HOME - (read only) <<<not quite sure what this does>>>
*  IMC_PARAM_MICROSTEPPING - (write only) The microstepping mode used by the driver.
*  IMC_PARAM_MIN_LIMIT_STATE - (read only) The state of the minimum limit switch.
*  IMC_PARAM_MAX_LIMIT_STATE - (read only) The state of the maximum limit switch.


## Slave Error Codes

### Control Failure

Control algorithm cannot achieve desired convergence, but actuator motion is achievable. Closed-loop controller only. Master response is to alert the user. Cleared on the next Initialize message.

### Electrical Failure

Something is wrong electrically (over-current, over-temp, whatever). IMC_PARAM_ERROR_INFO1 contains more detailed information

### Mechanical Failure

Motion isn’t happening (we’re sending current to the motor, but nothing’s happening).

### Move Timed Out

The requested move took longer than the watchdog timer allows for a single move. This is generally thrown if a homing routine was started but the limit switch never triggered.

# Communication Errors

When i2c reports a communication error in the transmission, or messages fail parity checks on either end, the defined behavior is as follows:
 * Slaves should only execute commands received without errors and with correct parity.
 * After executing, the slave responds with an "OK" response byte, informing the master of success.
 * Master will re-transmit packets if transmit errors occur, and will retransmit packets if slave response indicates transmission error (i.e. IMC_RSP_ERROR or IMC_RSP_UNKNOWN).
 * The master will re-request responses (but NOT re-transmit the original packets) if communication/parity issues occur in receiving the response.
