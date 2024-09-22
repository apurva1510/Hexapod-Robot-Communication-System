package com.example.capstonerobottest1;

// Import Android libraries and classes
import android.os.Handler;
import android.Manifest;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.ListView;
import android.widget.Toast;
import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import android.widget.EditText;
import android.widget.AdapterView;
import android.os.Message;

import android.widget.ArrayAdapter;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Set;
import java.util.UUID;


public class MainActivity extends AppCompatActivity implements JoystickView.JoystickListener {

    private static final int PERMISSION_REQUEST_CODE = 1001;// Permission Request Variable assignment

    // Bluetooth programming variables
    private BluetoothAdapter bluetoothAdapter; 
    private BluetoothSocket bluetoothSocket;
    private OutputStream outputStream;
    private InputStream inputStream;
    private Button robotGOButton, robotSTOPButton, robotHOMEButton, robotGAIT1Button, robotGAIT2Button, powerButton, tmuButton;//UI Elements
    private TextView statusTextView;
    private boolean isPowerOn = false;
    private boolean continuousSending = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) { // onCreate method called when activity is first created
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            // Check for Bluetooth permissions
            if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) { //Location can be used for getting access to the Raspberry Pi's Location. 
                // Permission not granted, request it
                ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.ACCESS_FINE_LOCATION}, PERMISSION_REQUEST_CODE);
            } else {
                // Permission granted, initialize Bluetooth
                initializeBluetooth();
            }
        } else {
            // Runtime permission not needed for versions below Marshmallow, initialize Bluetooth directly
            initializeBluetooth();
        }

        robotGAIT1Button = findViewById(R.id.gait1); // Initialize UI elements
        robotGAIT2Button = findViewById(R.id.gait2);
        robotSTOPButton = findViewById(R.id.stop);
        robotHOMEButton = findViewById(R.id.home);
        statusTextView = findViewById(R.id.statusTextView);
        powerButton = findViewById(R.id.power_button);
        tmuButton = findViewById(R.id.tmu);

        // Define click listeners for buttons
        powerButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                continuousSending = false;
                try{
                    // Toggle the power state
                    isPowerOn = !isPowerOn;
                    if (isPowerOn) {
                        sendData("POWER ON");
                        Toast.makeText(MainActivity.this, "Robot is powered ON", Toast.LENGTH_SHORT).show();
                    } else {
                        sendData("POWER OFF");
                        Toast.makeText(MainActivity.this, "Robot is powered OFF", Toast.LENGTH_SHORT).show();
                    }
                }catch (Exception e) {
                        e.printStackTrace();
                        Toast.makeText(MainActivity.this, "Error: " + e.getMessage(), Toast.LENGTH_SHORT).show();
                }
            }
        });

        tmuButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                try{
                    Toast.makeText(MainActivity.this, "Welcome to Toronto Metropolitian University", Toast.LENGTH_LONG).show();
                }catch (Exception e) {
                    e.printStackTrace();
                    Toast.makeText(MainActivity.this, "Error: " + e.getMessage(), Toast.LENGTH_LONG).show();
                }
            }
        });

        // robotGAIT1Button.setOnClickListener(new View.OnClickListener() {
        //     @Override
        //     public void onClick(View v) {
        //         try {
        //             Log.d("Button Click", "GAIT1 Robot button clicked");
        //             statusTextView.setText("Robot in GAIT1 mode");
        //             sendData("GAIT1");
        //             Toast.makeText(MainActivity.this, "Robot is in GAIT 1 Mode", Toast.LENGTH_SHORT).show();
        //         } catch (Exception e) {
        //             e.printStackTrace();
        //             Toast.makeText(MainActivity.this, "Error: " + e.getMessage(), Toast.LENGTH_SHORT).show();
        //         }
        //     }
        // }); //changed gait1
        robotGAIT1Button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                try {
                    Log.d("Button Click", "GAIT1 Robot button clicked");
                    statusTextView.setText("Robot in GAIT1 mode");
                    Toast.makeText(MainActivity.this, "Robot is in GAIT 1 Mode", Toast.LENGTH_SHORT).show();

                    // Set the flag to true to start continuous sending
                    continuousSending = true;
                    
                    // Start the loop to continuously send the command
                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            while (continuousSending) {
                                sendData("GAIT1");
                                try {
                                    // Sleep for a short duration to avoid overwhelming the Bluetooth connection
                                    Thread.sleep(1000); // Adjust the sleep duration as needed
                                } catch (InterruptedException e) {
                                    e.printStackTrace();
                                }
                            }
                        }
                    }).start();
                    
                } catch (Exception e) {
                    e.printStackTrace();
                    Toast.makeText(MainActivity.this, "Error: " + e.getMessage(), Toast.LENGTH_SHORT).show();
                }
            }
        });
        robotGAIT2Button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                continuousSending = false;
                try {
                    Log.d("Button Click", "GAIT2 Robot button clicked");
                    statusTextView.setText("Robot in GAIT2 mode");
                    sendData("GAIT2");
                    Toast.makeText(MainActivity.this, "Robot is in GAIT 2 Mode", Toast.LENGTH_SHORT).show();
                } catch (Exception e) {
                    e.printStackTrace();
                    Toast.makeText(MainActivity.this, "Error: " + e.getMessage(), Toast.LENGTH_SHORT).show();
                }
            }
        });

        robotHOMEButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                continuousSending = false;
                try{
                    Log.d("Button Click", "HOME button clicked");
                    statusTextView.setText("Robot is Homing");
                    sendData("HOME");
                    Toast.makeText(MainActivity.this, "Robot is Homing, Semi-Autonomous mode activated", Toast.LENGTH_SHORT).show();

                } catch (Exception e) {
                    e.printStackTrace();
                    Toast.makeText(MainActivity.this, "Error: " + e.getMessage(), Toast.LENGTH_SHORT).show();
                }
            }
        });

        robotSTOPButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                continuousSending = false;
                try{
                    Log.d("Button Click", "STOP button clicked");
                    statusTextView.setText("Robot stopped");
                    sendData("STOP");
                    Toast.makeText(MainActivity.this, "Interrupt Requested, Robot is stopping", Toast.LENGTH_SHORT).show();
                } catch (Exception e) {
                    e.printStackTrace();
                    Toast.makeText(MainActivity.this, "Error: " + e.getMessage(), Toast.LENGTH_SHORT).show();
                }
            }
        });
    }


    @Override
    public void onJoystickMoved(float xPercent, float yPercent, int id) {

        continuousSending = false;
        // Check the ID to determine which joystick is triggered
        if (id == JoystickView.JOYSTICK_LEFT_ID) {
            if (xPercent >= -1.0 && xPercent <= -0.99){
                yPercent = 0;
                Log.d("Joystick", "Joystick ID: " + id + ", X percent: " + xPercent + ", Y percent: " + yPercent);
                statusTextView.setText("Manual Mode - Robot is Rotating Left");  
                sendData("L" + "," + xPercent + "," + yPercent + "," + "LEFT");
            }
            else if (xPercent <= +1.0 && xPercent >= +0.99){
                yPercent = 0;
                Log.d("Joystick", "Joystick ID: " + id + ", X percent: " + xPercent + ", Y percent: " + yPercent);
                statusTextView.setText("Manual Mode - Robot is Rotating Right");  
                sendData("L" + "," + xPercent + "," + yPercent + "," + "RIGHT");
            }
        } else if (id == JoystickView.JOYSTICK_RIGHT_ID) {
            statusTextView.setText("Manual Mode - Robot in Motion");
            Log.d("Joystick", "Joystick ID: " + id + ", X percent: " + xPercent + ", Y percent: " + yPercent);
            sendData("R" + "," + xPercent + "," + yPercent + "," + "MOVE");
        }
    }


    private void initializeBluetooth() {
        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        // Check if Bluetooth is supported on the device
        if (bluetoothAdapter == null) {
            Toast.makeText(this, "Bluetooth is not supported on this device", Toast.LENGTH_SHORT).show(); // Handle the case where Bluetooth is not supported
            return;
        }

        // Enable Bluetooth
        if (!bluetoothAdapter.isEnabled()) {
            Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableBtIntent, 1);
        }

        // Get paired Bluetooth devices
        Set<BluetoothDevice> pairedDevices = bluetoothAdapter.getBondedDevices();

        //RFCOMM Service to connect to Raspberry Pi
        if (pairedDevices.size() > 0) {
            String deviceAddress = "D8:3A:DD:5C:C6:F5"; // Replace with Raspberry Pi's MAC address
            BluetoothDevice device = bluetoothAdapter.getRemoteDevice(deviceAddress);
            UUID uuid = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB"); // BluetoothSocket using a UUID

            try {
                // Create a socket and connect
                bluetoothSocket = device.createRfcommSocketToServiceRecord(uuid);
                bluetoothSocket.connect();
                outputStream = bluetoothSocket.getOutputStream();
                inputStream = bluetoothSocket.getInputStream();
                startReceivingData();
                Toast.makeText(MainActivity.this, "Bluetooth connected", Toast.LENGTH_SHORT).show();
            } catch (IOException e) {
                e.printStackTrace();
                Toast.makeText(MainActivity.this, "Failed to connect to Bluetooth device", Toast.LENGTH_SHORT).show();
            }
        }

        // Add a timeout mechanism for Bluetooth connection attempts
        new Handler().postDelayed(new Runnable() {
            @Override
            public void run() {
                if (!bluetoothSocket.isConnected()) {
                    // Connection attempt failed, notify user or handle accordingly
                    Toast.makeText(MainActivity.this, "Failed to connect to Bluetooth device. Please try again.", Toast.LENGTH_SHORT).show();
                }
            }
        }, 10000); // 10 seconds timeout
    }

    // Method to send data over Bluetooth
    private void sendData(String data) {
        
        if (outputStream != null) {
            try {
                outputStream.write(data.getBytes());
                Log.d("Bluetooth", "Data sent: " + data);
            } catch (IOException e) {
                e.printStackTrace();
                Log.e("Bluetooth", "Error sending data: " + e.getMessage());
                runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    Toast.makeText(MainActivity.this, "Error sending data: " + e.getMessage(), Toast.LENGTH_SHORT).show();
                    }
                });
            }
        } else {
            Log.e("Bluetooth", "OutputStream is null. Data: '" + data + "', not sent.");
            runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(MainActivity.this, "Error: OutputStream is null. Data not sent.", Toast.LENGTH_SHORT).show();
                }
            });
        }
    }

    // Function to read/recv from Raspberry Pi
    private void startReceivingData() {
        new Thread(new Runnable() {
            @Override
            public void run() {
                byte[] buffer = new byte[1024]; // Array to contain message and number of bytes
                int bytes;

                while (true) {
                    try {
                        bytes = inputStream.read(buffer);
                        String incomingMessage = new String(buffer, 0, bytes);
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                // WIP Can add feature to read the recving data from the Pi
                            }
                        });
                    } catch (IOException e) {
                        e.printStackTrace();
                        break;
                    }
                }
            }
        }).start();
    }

    // @Override
    // protected void onDestroy() { //Close Bluetooth Socket
    //     super.onDestroy();
    //     try {
    //         if (bluetoothSocket != null) {
    //             bluetoothSocket.close();
    //         }
    //     } catch (IOException e) {
    //         e.printStackTrace();
    //     }
    // }//changed gait1

    @Override
    protected void onDestroy() {
        super.onDestroy();
        // Stop the continuous sending loop
        continuousSending = false;
        try {
            if (bluetoothSocket != null) {
                bluetoothSocket.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) { // Method to handle permission request result
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);

        if (requestCode == PERMISSION_REQUEST_CODE) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                // Permission granted, initialize Bluetooth
                initializeBluetooth();
            } else {
                // Permission denied, notify user
                Toast.makeText(this, "Permission denied, Bluetooth functionality disabled", Toast.LENGTH_SHORT).show();
            }
        }
    }
}
