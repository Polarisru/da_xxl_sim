import win32file
import win32pipe

PIPE_NAME = r"\\.\pipe\ServoPipe"

def main():
    print("Connecting to server...")
    try:
        pipe = win32file.CreateFile(
            PIPE_NAME,
            win32file.GENERIC_READ | win32file.GENERIC_WRITE,
            0, None, win32file.OPEN_EXISTING, 0, None
        )
        print("Connected to server.")

        # Send a message to the server
        message = "Hello from the Python client!"
        win32file.WriteFile(pipe, message.encode('utf-8'))
        print(f"Sent to server: {message}")

        # Try to read multiple responses
        while True:
            try:
                result, response = win32file.ReadFile(pipe, 1024)
                print(f"Received from server: {response.decode('utf-8')}")
            except Exception as e:
                print(f"Error: {e}")
                break

    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'pipe' in locals():
            win32file.CloseHandle(pipe)
            print("Pipe closed.")

if __name__ == "__main__":
    main()
