#include "custom-json.h"
#include "product_config.h"
#include <stdio.h>
#include <inttypes.h>


static const char *Convert_Float(float value);

// This function reduce Float to string convertion without FPU to 1/3 of normal time.
// This function have been inspired by Benoit Blanchon Blog : https://blog.benoitblanchon.fr/lightweight-float-to-string/
const char *Convert_Float(float value)
{
    float remainder;
    static char output[39];
    output[0] = 0;
    if (value > -0.0001)
    {
        remainder = (value - (int32_t)value) * 1000.0;
        sprintf(output, "%" PRId32 ".%03" PRIu32 "", (int32_t)value, (uint32_t)remainder);
    }
    else
    {
        remainder = (-(value - (int32_t)value)) * 1000.0;
        if ((int32_t)value == 0)
        {
            sprintf(output, "-%" PRId32 ".%03" PRIu32 "", (int32_t)value, (uint32_t)remainder);
        }
        else
        {
            sprintf(output, "%" PRId32 ".%03" PRIu32 "", (int32_t)value, (uint32_t)remainder);
        }
    }

    // rounding values
    // remainder -= *decimalPart;
    // if (remainder >= 0.5)
    // {
    //     (*decimalPart)++;
    // }
    return output;
}


// This function are called by the gate conversion functions in case of an unknown type.
// This functions allow you to easily add and manage your custom type and commands.

// This function is called by the gate to convert a service type into a string.
// This is typically used in the end of detection to create a Json representing the device routing table.
// The name of the type will be used by pyluos to create the right object.
const char *Convert_CustomStringFromType(luos_type_t type)
{
    if (type == POINT_2D)
    {
        return "point_2D";
    }
    if (type == POWER_TYPE)
    {
        return "power";
    }
    return NULL;
}

// This function is called by the gate to convert a piece of Json into a message.
// This is typically used when a Json is received by the gate with an unknown property.
// You can use it to compose your own message out of the Json data and send it to the right service.
void Convert_CustomJsonToMsg(service_t *service, uint16_t target_id, char *property, const json_t *jobj, char *json_str)
{
    msg_t msg;
    msg.header.target_mode = IDACK;
    msg.header.target      = target_id;
    const uint16_t property_type = json_getType(jobj);
    // Target linear position 2D
    if (property && !strcmp(property, "linear_pos_2D"))
    {
        // Check the size of the array. If we have more than one data, this mean that this data is a binary size of a trajectory. If we have 2 data, this is an unique point.
        json_t const *item = json_getChild(jobj);
        if (json_getSibling(item) != NULL)
        {
            // We only have one point in this data
            pos_2d_t pos;
            pos.x = (uint16_t)json_getInteger(item);
            item  = json_getSibling(item);
            pos.y = (uint16_t)json_getInteger(item);
            // Create the message
            msg.header.cmd  = LINEAR_POSITION_2D;
            msg.header.size = sizeof(pos_2d_t);
            memcpy(msg.data, &pos, sizeof(pos_2d_t));
            // Send the message
            Luos_SendMsg(service, &msg);
        }
        else
        {
            int i = 0;
            // This is a binary
            int size = (int)json_getInteger(item);
            // Find the first \r of the current json_str
            for (i = 0; i < GATE_BUFF_SIZE; i++)
            {
                if (json_str[i] == '\n')
                {
                    i++;
                    break;
                }
            }
            if (i < GATE_BUFF_SIZE - 1)
            {
                // Create the message
                msg.header.cmd = LINEAR_POSITION_2D;
                Luos_SendData(service, &msg, &json_str[i], (unsigned int)size);
            }
        }
        return;
    }
    if (property && !strcmp(property, "buffer_mode"))
    {
        msg.data[0]     = (char)json_getInteger(jobj);
        msg.header.cmd  = BUFFER_MODE;
        msg.header.size = sizeof(char);
        Luos_SendMsg(service, &msg);
        return;
    }
    if (property && !strcmp(property, "sampling_freq"))
    {
        float freq         = (float)json_getReal(jobj);
        time_luos_t period = TimeOD_TimeFrom_s(1.0f / freq);
        TimeOD_TimeToMsg(&period, &msg);
        Luos_SendMsg(service, &msg);
        return;
    }
    // ratio
    if ((property && !strcmp(property, "power_ratio")) && (property_type == JSON_ARRAY))
    {
        int i = 0;
        // this is a trajectory
        int size = (int)json_getInteger(json_getChild(jobj));
        if (size == 0)
        {
            // This trajaectory is empty
            return;
        }
        // find the first \r of the current buf
        for (i = 0; i < GATE_BUFF_SIZE; i++)
        {
            if (json_str[i] == '\n')
            {
                i++;
                break;
            }
        }
        if (i < GATE_BUFF_SIZE - 1)
        {
            msg.header.cmd = RATIO;
            Luos_SendData(service, &msg, &json_str[i], (unsigned int)size);
        }
        return;
    }

    // new_pos
    if ((property && !strcmp(property, "new_pos")) && ((property_type == JSON_REAL) || (property_type == JSON_INTEGER)))
    {
        float new_pos = (float)json_getReal(jobj);
        msg.header.cmd = NEW_POS;
        memcpy(msg.data, &new_pos, sizeof(float));
        msg.header.size = sizeof(float);
        while (Luos_SendMsg(service, &msg) == FAILED)
        {
            Luos_Loop();
        }
        return;
    }


    // Receive stepPerTurn For Stepper Motors
    if ((property && !strcmp(property, "spt")) && ((property_type == JSON_REAL) || (property_type == JSON_INTEGER)))
    {
        float stepPerTurn = (float)json_getReal(jobj);
        msg.header.cmd = STEP_PER_TURN;
        memcpy(msg.data, &stepPerTurn, sizeof(float));
        msg.header.size = sizeof(float);
        while (Luos_SendMsg(service, &msg) == FAILED)
        {
            Luos_Loop();
        }
        return;
    }

    // Receive active buffer size return For Stepper Motors trajectory
    if ((property && !strcmp(property, "buf_return")) && (property_type == JSON_BOOLEAN))
    {
        msg.data[0]     = json_getBoolean(jobj);
        msg.header.cmd  = BUFFER_RETURN;
        msg.header.size = sizeof(char);
        Luos_SendMsg(service, &msg);
        return;
    }

    // Receive nb_pattern for laser and galvo services
    if ((property && !strcmp(property, "nb_pattern")) && ((property_type == JSON_REAL) || (property_type == JSON_INTEGER)))
    {
        float nb_pattern = (float)json_getReal(jobj);
        msg.header.cmd = SET_NB_PATTERN;
        memcpy(msg.data, &nb_pattern, sizeof(float));
        msg.header.size = sizeof(float);
        while (Luos_SendMsg(service, &msg) == FAILED)
        {
            Luos_Loop();
        }
        return;
    }
    
    // Receive position encoder mode activation 
    if ((property && !strcmp(property, "encod_mode")) && (property_type == JSON_BOOLEAN))
    {
        msg.data[0]     = json_getBoolean(jobj);
        msg.header.cmd  = ENCOD_MODE;
        msg.header.size = sizeof(char);
        Luos_SendMsg(service, &msg);
        return;
    }

}

// This function is called by the gate to convert a message into a piece of Json.
// This is typically used when a message is received by the gate with an unknown command.
// You can use it to compose your own piece of Json out of the message data.
void Convert_CustomMsgToJson(const msg_t *msg, char *data)
{
    float fdata;
    if (msg->header.cmd == LINEAR_POSITION_2D)
    {
        // This is our custom message, so we can convert it to JSON
        // In this case we will don't need it but I did the code for the sake of the example.
        if (msg->header.size == sizeof(pos_2d_t))
        {
            // Size ok, now fill the struct from msg data
            pos_2d_t pos;
            memcpy(&pos, msg->data, msg->header.size);
            // create the Json content
            sprintf(data, "\"linear_pos_2D\":[%2d,%2d],", pos.x, pos.y);
        }
    }
    if (msg->header.cmd == NEW_POS)
    {
        memcpy(&fdata, msg->data, sizeof(float));
        sprintf(data, "\"new_pos\":%s,", Convert_Float(fdata));
    }
    if (msg->header.cmd == STEP_PER_TURN)
    {
        memcpy(&fdata, msg->data, sizeof(float));
        sprintf(data, "\"spt\":%s,", Convert_Float(fdata));
    }
    if (msg->header.cmd == BUFFER_SIZE)
    {
        memcpy(&fdata, msg->data, sizeof(float));
        sprintf(data, "\"buffer_size\":%s,", Convert_Float(fdata));
    }
    if (msg->header.cmd == POSITION_ENCODER)
    {
        memcpy(&fdata, msg->data, sizeof(float));
        sprintf(data, "\"encod_pos\":%s,", Convert_Float(fdata));
    }
}
