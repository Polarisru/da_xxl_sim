import zmq
import threading
import time

# Function to handle sending commands to the server
def command_sender():
    # Create a REQ socket for sending commands
    context = zmq.Context()
    req_socket = context.socket(zmq.REQ)
    req_socket.connect("tcp://localhost:5555")  # Connect to the REP socket of the server

    while True:
        # Get command from the user
        command = input("Enter command (or 'exit' to quit): ")
        if command.lower() == "exit":
            print("Exiting command sender...")
            break

        # Send the command to the server
        req_socket.send_string(command)

        # Wait for a response from the server
        response = req_socket.recv_string()
        print(f"Server response: {response}")

    # Clean up
    req_socket.close()
    context.term()

# Function to handle receiving subscribed messages
def message_receiver():
    # Create a SUB socket for receiving messages
    context = zmq.Context()
    sub_socket = context.socket(zmq.SUB)
    sub_socket.connect("tcp://localhost:5556")  # Connect to the PUB socket of the server
    sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages

    print("Listening for periodic data from the server...")

    while True:
        # Receive a message from the server
        message = sub_socket.recv_string()
        print(f"Received periodic data: {message}")

    # Clean up (this won't be reached in this example)
    sub_socket.close()
    context.term()

if __name__ == "__main__":
    # Start the message receiver in a separate thread
    receiver_thread = threading.Thread(target=message_receiver)
    receiver_thread.daemon = True  # Daemonize thread to exit when the main program exits
    receiver_thread.start()

    # Start the command sender in the main thread
    command_sender()