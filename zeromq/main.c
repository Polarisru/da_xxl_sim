#include "FreeRTOS.h"
#include "task.h"
#include "zmq.h"
#include <stdio.h>
#include <string.h>

static int count = 0;

// Task to publish data periodically
void vPublisherTask(void *pvParameters) {
    // Create a ZeroMQ context and PUB socket
    void *context = zmq_ctx_new();
    void *publisher = zmq_socket(context, ZMQ_PUB);
    if (zmq_bind(publisher, "tcp://*:5556") != 0) {
        printf("Failed to bind PUB socket\n");
        //vTaskDelete(NULL);
    }

    printf("Publisher task started. Publishing data on tcp://*:5556...\n");

    while (1) {
        // Create a message to publish
        char message[256];
        snprintf(message, sizeof(message), "Data %d: message text", count);
        count = (count + 1) % 10;

        // Publish the message
        if (zmq_send(publisher, message, strlen(message), 0) < 0) {
            printf("Failed to publish message\n");
        } else {
            printf("Published: %s\n", message);
        }

        // Wait for 1 second before sending the next message
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Clean up (this will not be reached in this example)
    zmq_close(publisher);
    zmq_ctx_destroy(context);
    //vTaskDelete(NULL);
}

// Task to handle incoming commands
void vCommandTask(void *pvParameters) {
    // Create a ZeroMQ context and REP socket
    void *context = zmq_ctx_new();
    void *responder = zmq_socket(context, ZMQ_REP);
    if (zmq_bind(responder, "tcp://*:5555") != 0) {
        printf("Failed to bind REP socket\n");
        //vTaskDelete(NULL);
    }

    printf("Command task started. Listening for commands on tcp://*:5555...\n");

    while (1) {
        // Receive a command from the client
        char command[256];
        int bytes_received = zmq_recv(responder, command, sizeof(command), 0);
        if (bytes_received > 0) {
            command[bytes_received] = '\0';  // Null-terminate the command
            printf("Received command: %s\n", command);

            // Process the command
            const char *response = "Command processed";
            if (strcmp(command, "stop") == 0) {
                printf("Stopping server...\n");
                break;
            } else if (strcmp(command, "reset") == 0) {
                printf("Counter reset requested\n");
                count = 0;
            } else {
                printf("Unknown command: %s\n", command);
            }

            // Send a response to the client
            if (zmq_send(responder, response, strlen(response), 0) < 0) {
                printf("Failed to send response\n");
            }
        }

        // Small delay to yield CPU
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Clean up
    zmq_close(responder);
    zmq_ctx_destroy(context);
    //vTaskDelete(NULL);
}

// Main function
int main() {
    // Create the publisher task
    if (xTaskCreate(vPublisherTask, "Publisher", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL) != pdPASS) {
        printf("Failed to create publisher task\n");
        return 1;
    }

    // Create the command task
    if (xTaskCreate(vCommandTask, "Command", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL) != pdPASS) {
        printf("Failed to create command task\n");
        return 1;
    }

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // The program should never reach here
    for (;;);
    return 0;
}
