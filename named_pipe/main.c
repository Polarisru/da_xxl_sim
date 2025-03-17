#include <windows.h>
#include <stdio.h>
#include <stdbool.h>

#define PIPE_NAME "\\\\.\\pipe\\ServoPipe"
#define BUFFER_SIZE 1024

typedef struct {
    HANDLE pipe;
    OVERLAPPED overlapped;
    char buffer[BUFFER_SIZE];
    DWORD bytesTransferred;
    bool isConnected;
} PipeInstance;

// Function to create a new pipe instance
PipeInstance createPipeInstance() {
    PipeInstance instance;
    instance.pipe = CreateNamedPipe(
        PIPE_NAME,
        PIPE_ACCESS_DUPLEX | FILE_FLAG_OVERLAPPED,
        PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT,
        PIPE_UNLIMITED_INSTANCES, BUFFER_SIZE, BUFFER_SIZE, 0, NULL
    );

    if (instance.pipe == INVALID_HANDLE_VALUE) {
        printf("Failed to create named pipe. Error: %lu\n", GetLastError());
        exit(1);
    }

    ZeroMemory(&instance.overlapped, sizeof(OVERLAPPED));
    instance.overlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (instance.overlapped.hEvent == NULL) {
        printf("Failed to create event. Error: %lu\n", GetLastError());
        exit(1);
    }

    instance.bytesTransferred = 0;
    instance.isConnected = false;

    return instance;
}

// Function to reset a pipe instance for reuse
void resetPipeInstance(PipeInstance *instance) {
    if (!DisconnectNamedPipe(instance->pipe)) {
        printf("Error disconnecting pipe: %lu\n", GetLastError());
    } else {
        printf("Client disconnected.\n");
    }

    instance->isConnected = false;
    CloseHandle(instance->overlapped.hEvent);
    CloseHandle(instance->pipe);  // Close the existing pipe handle

    // Recreate the pipe instance
    instance->pipe = CreateNamedPipe(
        PIPE_NAME,
        PIPE_ACCESS_DUPLEX | FILE_FLAG_OVERLAPPED,
        PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT,
        PIPE_UNLIMITED_INSTANCES, BUFFER_SIZE, BUFFER_SIZE, 0, NULL
    );

    if (instance->pipe == INVALID_HANDLE_VALUE) {
        printf("Failed to recreate named pipe. Error: %lu\n", GetLastError());
        exit(1);
    }

    ZeroMemory(&instance->overlapped, sizeof(OVERLAPPED));
    instance->overlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (instance->overlapped.hEvent == NULL) {
        printf("Failed to recreate event. Error: %lu\n", GetLastError());
        exit(1);
    }
}

void writeToPipeInstance(PipeInstance *instance, const char *msg) {
  if (instance->isConnected) {
    DWORD bytesWritten;
    BOOL success;
    success = WriteFile(instance->pipe, msg, (DWORD)strlen(msg), &bytesWritten, NULL);

    if (!success && GetLastError() == ERROR_BROKEN_PIPE) {
        //printf("Error writing to client (disconnected): %lu\n", GetLastError());
        resetPipeInstance(instance);
    } else {
        printf("Message sent: %s\n", msg);
    }
  }
}

// Function to handle asynchronous operations on the pipe
void handlePipeInstance(PipeInstance *instance) {
    if (!instance->isConnected) {
        // Start an asynchronous connection
        BOOL connected = ConnectNamedPipe(instance->pipe, &instance->overlapped);
        if (!connected && GetLastError() == ERROR_IO_PENDING) {
            // Connection is pending; return control to the main loop
            return;
        } else if (connected || GetLastError() == ERROR_PIPE_CONNECTED) {
            // Client is connected
            instance->isConnected = true;
            printf("Client connected.\n");
        } else {
            printf("Failed to connect client. Error: %lu\n", GetLastError());
            resetPipeInstance(instance);
        }
    } else {
        // Handle client communication
        BOOL success = ReadFile(
            instance->pipe,
            instance->buffer,
            BUFFER_SIZE - 1,
            &instance->bytesTransferred,
            &instance->overlapped
        );

        if (!success && GetLastError() == ERROR_IO_PENDING) {
            // Read operation is pending
            return;
        } else if (success || GetLastError() == ERROR_IO_PENDING) {
            // Successfully read data
            WaitForSingleObject(instance->overlapped.hEvent, INFINITE);
            GetOverlappedResult(instance->pipe, &instance->overlapped, &instance->bytesTransferred, FALSE);
            instance->buffer[instance->bytesTransferred] = '\0';
            printf("Received message: %s\n", instance->buffer);

            // Send a response to the client
            const char *response = "Message received!";
//            DWORD bytesWritten;
//            success = WriteFile(instance->pipe, response, (DWORD)strlen(response), &bytesWritten, NULL);
//
//            if (!success && GetLastError() == ERROR_BROKEN_PIPE) {
//                printf("Error writing to client (disconnected): %lu\n", GetLastError());
//                resetPipeInstance(instance);
//            } else {
//                printf("Response sent.\n");
//            }
            writeToPipeInstance(instance, response);

        } else if (GetLastError() == ERROR_BROKEN_PIPE) {
            //printf("Client disconnected during read.\n");
            resetPipeInstance(instance);
        } else {
            printf("Error reading from client. Error: %lu\n", GetLastError());
            resetPipeInstance(instance);
        }
    }
}

int main() {
    PipeInstance pipe = createPipeInstance();

    printf("Server is running. Waiting for clients...\n");

    while (true) {
        // Handle pipe instance in a non-blocking way
        handlePipeInstance(&pipe);
        Sleep(50); // Avoid 100% CPU usage in the loop
        writeToPipeInstance(&pipe, "Tick");
    }

    CloseHandle(pipe.pipe);
    CloseHandle(pipe.overlapped.hEvent);
    return 0;
}
