#include <windows.h>
#include <stdio.h>
#include <string.h>

#define PIPE_NAME "\\\\.\\pipe\\ServoPipe"
#define BUFFER_SIZE 1024

int main() {
    HANDLE hPipe;
    OVERLAPPED overlapped = {0};
    char buffer[BUFFER_SIZE];
    DWORD bytesRead, bytesWritten;
    BOOL hasPendingIO = FALSE;

    // Create an event for the OVERLAPPED structure
    overlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (overlapped.hEvent == NULL) {
        printf("Error creating event: %ld\n", GetLastError());
        return 1;
    }

    while (1) {
        // Create a new named pipe instance
        hPipe = CreateNamedPipe(
            PIPE_NAME,
            PIPE_ACCESS_DUPLEX | FILE_FLAG_OVERLAPPED, // Asynchronous read/write
            PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT, // Message mode
            1, BUFFER_SIZE, BUFFER_SIZE, 0, NULL
        );

        if (hPipe == INVALID_HANDLE_VALUE) {
            printf("Error creating named pipe: %ld\n", GetLastError());
            CloseHandle(overlapped.hEvent);
            return 1;
        }

        printf("Waiting for a client to connect...\n");

        // Wait for a client to connect
        BOOL connected = ConnectNamedPipe(hPipe, &overlapped);
        if (!connected && GetLastError() == ERROR_IO_PENDING) {
            hasPendingIO = TRUE;
        } else if (GetLastError() == ERROR_PIPE_CONNECTED) {
            printf("Client connected immediately.\n");
        } else if (!connected) {
            printf("Error connecting named pipe: %ld\n", GetLastError());
            CloseHandle(hPipe);
            continue; // Retry by creating a new pipe instance
        }

        // Wait for connection or completion of pending I/O
        if (hasPendingIO) {
            DWORD waitResult = WaitForSingleObject(overlapped.hEvent, INFINITE);
            if (waitResult != WAIT_OBJECT_0) {
                printf("Error waiting for client connection: %ld\n", GetLastError());
                CloseHandle(hPipe);
                continue;
            }
            hasPendingIO = FALSE;
        }
        printf("Client connected.\n");

        // Communication loop
        while (1) {
            memset(buffer, 0, BUFFER_SIZE);

            // Start reading from the client
            BOOL readSuccess = ReadFile(hPipe, buffer, BUFFER_SIZE - 1, &bytesRead, &overlapped);
            if (!readSuccess && GetLastError() == ERROR_IO_PENDING) {
                // Wait for the read operation to complete
                DWORD waitResult = WaitForSingleObject(overlapped.hEvent, INFINITE);
                if (waitResult != WAIT_OBJECT_0) {
                    printf("Error waiting for read: %ld\n", GetLastError());
                    break;
                }
                readSuccess = GetOverlappedResult(hPipe, &overlapped, &bytesRead, FALSE);
            }

            if (!readSuccess) {
                if (GetLastError() == ERROR_BROKEN_PIPE) {
                    printf("Client disconnected.\n");
                    break;
                }
                printf("Error reading from client: %ld\n", GetLastError());
                break;
            }

            buffer[bytesRead] = '\0'; // Null-terminate the received string
            printf("Received from client: %s\n", buffer);

            // Prepare a response
            const char *response = "Message received!";
            DWORD responseLength = (DWORD)strlen(response);

            // Start writing to the client
            BOOL writeSuccess = WriteFile(hPipe, response, responseLength, &bytesWritten, &overlapped);
            if (!writeSuccess && GetLastError() == ERROR_IO_PENDING) {
                // Wait for the write operation to complete
                DWORD waitResult = WaitForSingleObject(overlapped.hEvent, INFINITE);
                if (waitResult != WAIT_OBJECT_0) {
                    printf("Error waiting for write: %ld\n", GetLastError());
                    break;
                }
                writeSuccess = GetOverlappedResult(hPipe, &overlapped, &bytesWritten, FALSE);
            }

            if (!writeSuccess) {
                printf("Error writing to client: %ld\n", GetLastError());
                break;
            }

            printf("Sent to client: %s\n", response);
        }

        // Cleanup for this client
        DisconnectNamedPipe(hPipe);
        CloseHandle(hPipe);
        printf("Ready for the next client.\n");
    }

    // Cleanup
    CloseHandle(overlapped.hEvent);
    printf("Server shutting down.\n");
    return 0;
}
