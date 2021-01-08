#ifndef NODES_H_
#define NODES_H_

#include <Arduino.h>
//#include "canFrameStream.h"
#include "can_com.h"
#include "arduino_msg.h"

extern can_frame_stream cf_stream;

extern Comunication comObj;

class nodeL
{
public:
    byte ID; //byte

    volatile bool new_node = false; // When identified a new node in the ISR routine
    volatile bool initializing_network = true;
    volatile uint8_t n_nodes = 1; // Number of network nodes
    uint8_t *node_list = (uint8_t *)malloc(sizeof(uint8_t));

    bool EONI_flag = false;

    void sortNodes()
    {
        uint8_t tmp = 0;
        for (int i = 0; i < n_nodes; i++)
        {
            for (int j = i + 1; j < n_nodes; j++)
            {
                if (node_list[j] < node_list[i])
                {
                    tmp = node_list[i];
                    node_list[i] = node_list[j];
                    node_list[j] = tmp;
                }
            }
        }
    }

    int getNodeIDX(uint8_t ID_)
    {
        int j = 0;
        // Establish the order to save the previous control intents in vector uff_PWM
        for (j = 0; j < n_nodes; j++)
        {
            if (node_list[j] == ID_)
                return j;
        }
        return -1;
    }

    bool checkID(uint8_t id)
    { // Checks if the given id is a member of the node network
        for (int i = 0; i < n_nodes; i++)
        {
            if (node_list[i] == id)
            { // id matches the list member
                return true;
            }
        }
        return false;
    }

    unsigned long t = 0; // aux DEBUG
    void network_init()
    {
        can_frame frame;
        Serial.print(ID);
        Serial.println(" - Initializing network...");
        unsigned long time_ref = millis(); // Initial time stamp
                                           // Flag to indentify End Of Network Identification cycle
        while ((millis() - time_ref < 20000))
        {
            if (EONI_flag == true)
            {
                new_node = false;
                cli();
                cf_stream.get(frame);
                sei();
                //Serial.println(frame.data[0]);
                n_nodes++; // incrementes number of nodes
                node_list = (uint8_t *)realloc(node_list, n_nodes);
                node_list[n_nodes - 1] = frame.can_id; // Adds new node ID to list of nodes
                // Serial.print("Identified node ID: ");
                // Serial.println(frame.can_id);
                break;
            }
            if ((millis() - t) >= 1000)
            {
                Serial.println((t++) / 1000);
                t = millis();
            }
        }

        Serial.println("20s passed OR EONI Flag received!");
        delay(ID * 10 + 15);
        comObj.write_byte(ID, NID);        // Broadcasts own ID
        time_ref = millis();               // New time stamp
        while (millis() - time_ref < 5000) // Listens for other nodes
        {
            if (new_node)
            {
                new_node = false;

                cli();
                cf_stream.get(frame);
                sei();
                //Serial.println(frame.data[0]);
                n_nodes++; // incrementes number of nodes
                node_list = (uint8_t *)realloc(node_list, n_nodes);
                node_list[n_nodes - 1] = frame.can_id; // Adds new node ID to list of nodes
                // Serial.print("Identified node ID: ");
                // Serial.println(frame.can_id);
            }
        }

        initializing_network = false;
        //Serial.print(ID);
        //Serial.println("\tEnd of network initialization!");
        // while (1)
        // {
        //   write_byte(ID, NID);
        //   delay(100);
        // }
    }
};

#endif