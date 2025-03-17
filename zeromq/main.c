#include <zmq.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

int main() {
    // Create a ZeroMQ context
    void *context = zmq_ctx_new();
    if (!context) {
        fprintf(stderr, "Failed to create context\n");
        return 1;
    }

    // Create a PUB socket (for publishing data)
    void *publisher = zmq_socket(context, ZMQ_PUB);
    if (!publisher) {
        fprintf(stderr, "Failed to create PUB socket\n");
        zmq_ctx_destroy(context);
        return 1;
    }

    // Bind the PUB socket to a TCP port
    int rc = zmq_bind(publisher, "tcp://*:5556");
    if (rc != 0) {
        fprintf(stderr, "Failed to bind PUB socket\n");
        zmq_close(publisher);
        zmq_ctx_destroy(context);
        return 1;
    }

    // Create a REP socket (for receiving commands)
    void *responder = zmq_socket(context, ZMQ_REP);
    if (!responder) {
        fprintf(stderr, "Failed to create REP socket\n");
        zmq_close(publisher);
        zmq_ctx_destroy(context);
        return 1;
    }

    // Bind the REP socket to a TCP port
    rc = zmq_bind(responder, "tcp://*:5555");
    if (rc != 0) {
        fprintf(stderr, "Failed to bind REP socket\n");
        zmq_close(responder);
        zmq_close(publisher);
        zmq_ctx_destroy(context);
        return 1;
    }

    printf("Server is running and ready to receive commands and publish data...\n");

    int count = 0;
    while (1) {
        // Check for incoming commands (non-blocking)
        zmq_pollitem_t items[] = {
            { responder, 0, ZMQ_POLLIN, 0 }  // Listen for commands on the REP socket
        };
        int rc = zmq_poll(items, 1, 0);  // Poll with 0ms timeout (non-blocking)

        if (rc > 0 && items[0].revents & ZMQ_POLLIN) {
            // Receive a command from the client
            char command[256];
            int bytes_received = zmq_recv(responder, command, sizeof(command), 0);
            if (bytes_received < 0) {
                fprintf(stderr, "Failed to receive command\n");
                continue;
            }

            // Null-terminate the command
            command[bytes_received] = '\0';

            // Process the command
            printf("Received command: %s\n", command);

            // React to the command
            if (strcmp(command, "stop") == 0) {
                printf("Stopping server...\n");
                break;
            } else if (strcmp(command, "reset") == 0) {
                count = 0;
                printf("Counter reset to 0\n");
            } else {
                printf("Unknown command: %s\n", command);
            }

            // Send a response to the client
            const char *response = "Command processed";
            int bytes_sent = zmq_send(responder, response, strlen(response), 0);
            if (bytes_sent < 0) {
                fprintf(stderr, "Failed to send response\n");
            }
        }

        // Publish data periodically
        char message[256];
        snprintf(message, sizeof(message), "Data %d: message text", count);
        count = (count + 1) % 10;

        int bytes_sent = zmq_send(publisher, message, strlen(message), 0);
        if (bytes_sent < 0) {
            fprintf(stderr, "Failed to send message\n");
            break;
        }

        printf("Published: %s\n", message);

        // Wait for 1 second before sending the next message
        sleep(1);
    }

    // Clean up
    zmq_close(responder);
    zmq_close(publisher);
    zmq_ctx_destroy(context);
    return 0;
}
