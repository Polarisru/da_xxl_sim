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
        fprintf(stderr, "Failed to create socket\n");
        zmq_ctx_destroy(context);
        return 1;
    }

    // Bind the socket to a TCP port
    int rc = zmq_bind(publisher, "tcp://*:5556");
    if (rc != 0) {
        fprintf(stderr, "Failed to bind socket\n");
        zmq_close(publisher);
        zmq_ctx_destroy(context);
        return 1;
    }

    printf("Server is running and publishing data...\n");

    int count = 0;
    while (1) {
        // Create a message to publish
        char message[256];
        snprintf(message, sizeof(message), "Data %d", count++);

        // Publish the message
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
    zmq_close(publisher);
    zmq_ctx_destroy(context);
    return 0;
}
