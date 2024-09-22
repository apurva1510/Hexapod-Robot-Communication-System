import bluetooth
from new_control_4 import Control
 

SERVICE_UUID = "00001101-0000-1000-8000-00805F9B34FB" # Pre-defined UUID 
SERVICE_NAME = "BluetoothSerial" #Service/Socket name

def main():
    server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM) #Socket init.
    server_sock.bind(("", bluetooth.PORT_ANY))
    server_sock.listen(1)

    bluetooth.advertise_service(server_sock, SERVICE_NAME,
                                 service_id=SERVICE_UUID,
                                 service_classes=[bluetooth.SERIAL_PORT_CLASS],
                                 profiles=[bluetooth.SERIAL_PORT_PROFILE])

    print("Waiting for connection...")
    client_sock, client_info = server_sock.accept()
    print("Accepted connection from", client_info)

    try:
        while True:
            data = client_sock.recv(1024)
            if len(data) == 0:
                break

            #Integration Starts here:

            else: #Remove the else if it is causing an error or empty terminal

                decodedData = data.decode()
                splitData = decodedData.split(',')
                whichJoystick = splitData[0].strip()
                robotMovementInstance = new_control_4.Control()  # Create an instance of the Control class
                
                if whichJoystick == "L":

                    xRotate = splitData[1]
                    print("Leftjoystick X Coordinate:",xRotate)
                 #   continue

                elif whichJoystick == "R": 
                    xMove = float(splitData[1])
                    print("Right joystick X Coordinate:", xMove )
                    if "R" in splitData[2]:
                        yMove = float(splitData[2].replace("R", ""))
                        print("Right joystick Y Coordinate:", yMove )
                    else:
                        yMove = float(splitData[2])
                        print("Right joystick Y Coordinate:", yMove )
                        
                robotMovementInstance.condition(xMove, yMove, xRotate)  # Call the condition method with the parameters
                        
                if 'L' in str(decodedData) or 'R' in str(decodedData):
                    continue
                else:
                    print("Commands Received:", decodedData)

    except Exception as e:
        print("Error:", e)
    finally:
        client_sock.close() #Terminates the socket on exiting the app
        server_sock.close() 

if __name__ == "__main__":
    main()