import serial
import time
import json
INFO_COMMAND = b"??"
SHUTDOWN_COMMAND = b"QQ"


def shutdown_duckiebattery(serial_port="/dev/ttyACM0", baud_rate=9600):
    """
    Sends a shutdown command to the Duckiebattery via USB serial connection.
    """
    # Command to be sent
    command = b"?"
    try:
        # Open serial connection to the Duckiebattery
        ser = serial.Serial(port=serial_port, baudrate=baud_rate, timeout=10)
        print("Connected to Duckiebattery on {} at {} baud.".format(serial_port, baud_rate))

        # Send the shutdown command
        ser.write(command)
        print("Shutdown command sent to Duckiebattery.")

        # Optional: Read response from Duckiebattery
        response = ser.readline().decode("utf-8").strip()
        if response:
            print("Duckiebattery response: {}".format(response))

        # Try parsing JSON
        
        parsed_data = json.loads(response)
        first_value = float(next(iter(parsed_data())))

        # # Extract SOC(%)
        # soc_key = next((key for key in parsed_data if "SOC" in key), None)
        # if soc_key:
        #     soc_percentage = float(parsed_data[soc_key])
        #     print ("Duckiebattery percentage: {}".format(soc_percentage))
        # else:
        #     print("SOC key not found in data.")





        # # Parse the JSON string into a dictionary
        # parsed_data = json.loads(response)

        # # Extract and convert SOC(%) to float
        # soc_percentage = float(parsed_data["SOC(%)"])
        # print ("Duckiebattery percentage: {}".format(soc_percentage))

        # Close the serial connection
        ser.close()

    except serial.SerialException as e:
        print("Serial exception occurred: {}".format(e))
    except Exception as e:
        print("An error occurred while sending the shutdown command: {}".format(e))


if __name__ == "__main__":
    # Adjust the serial port and baud rate as needed
    shutdown_duckiebattery(serial_port="/dev/ttyACM0", baud_rate=9600)

    
