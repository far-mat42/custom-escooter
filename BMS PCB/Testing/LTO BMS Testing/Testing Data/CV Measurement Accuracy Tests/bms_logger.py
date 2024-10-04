import serial
import csv
import time

# Set up the serial connection
ser = serial.Serial(
    port='COM4',  # Adjust port as necessary
    baudrate=9600,        # Adjust baudrate as necessary
    timeout=0             # Non-blocking mode
)

# Clear any existing data in the buffer
ser.reset_input_buffer()

# Open (or create) the CSV file and set up the writer
with open("uart_data.csv", mode="a", newline="") as csv_file:
    csv_writer = csv.writer(csv_file)
    
    # Optionally, write headers to the CSV file if it's a new file
    csv_writer.writerow(["Timestamp", "Data"])

    try:
        print("Listening for the most recent data...")

        while True:
            if ser.in_waiting > 0:  # Check if new data is available
                data = ser.read(ser.in_waiting).decode('utf-8').strip()  # Read and decode the latest data
                if data:
                    # Get the current timestamp
                    timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                    
                    # Print the received data for debugging
                    print(f"Received at {timestamp}: {data}")
                    
                    # Write the timestamp and data to the CSV file
                    csv_writer.writerow([timestamp, data])
                    
                    # Ensure data is written to the file immediately
                    csv_file.flush()

    except KeyboardInterrupt:
        print("Stopping the script.")

    finally:
        ser.close()  # Close the serial connection