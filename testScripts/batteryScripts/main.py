import serial
import time

INFO_COMMAND = b"??"
SHUTDOWN_COMMAND = b"QQ"


def shutdown_duckiebattery(serial_port="/dev/ttyACM0", baud_rate=9600):
    """
    Sends a shutdown command to the Duckiebattery via USB serial connection.
    """
    # Command to be sent
    command = SHUTDOWN_COMMAND
    try:
        # Open serial connection to the Duckiebattery
        ser = serial.Serial(port=serial_port, baudrate=baud_rate, timeout=1)
        print(
            "Connected to Duckiebattery on {} at {} baud.".format(
                serial_port, baud_rate
            )
        )

        # Send the shutdown command
        ser.write(command)
        print("Shutdown command sent to Duckiebattery.")

        # Optional: Read response from Duckiebattery
        response = ser.readline().decode("utf-8").strip()
        if response:
            print("Duckiebattery response: {}".format(response))

        # Close the serial connection
        ser.close()

    except serial.SerialException as e:
        print("Serial exception occurred: {}".format(e))
    except Exception as e:
        print("An error occurred while sending the shutdown command: {}".format(e))


if _name_ == "_main_":
    # Adjust the serial port and baud rate as needed
    shutdown_duckiebattery(serial_port="/dev/ttyACM0", baud_rate=9600)
