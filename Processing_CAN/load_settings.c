#include <stdio.h>
//#include "process.h"
#include <stdint.h>

struct can_msg {
    unsigned char id[2]; //2 bytes
    unsigned char pkt; //1 byte
    uint8_t start; //start byte in the packet, 1 byte
    uint8_t len; //number of bytes of data, 1 byte
    unsigned char bits[3]; //3 bytes (1 bit for whether each bytes enabled--max payload = 22 bytes --> 3 needed) 
    uint8_t lts; //number of bytes to send, 1 byte
    uint8_t idx;
};

struct msg_type {
    unsigned char id; //1 byte
    uint8_t data_len; //in bytes, 1 byte
    int speed; //speed to send in ms
    uint8_t num_messages;
    int len;
};

struct msg_type med;
struct msg_type fast;
struct msg_type stat;

//File should be in the form of id, pkt, len of original data (Bytes), bytes to send, length to send 
// bytes per line  = 9
//counters is a variable that tracks, for each type of packet, how many bytes have been assigned
int process_line(char* msg, struct can_msg* msg_table, int counter) {
    struct can_msg M = { 0 };
    M.id[0] = msg[0];
    M.id[1] = msg[1];
    M.pkt = msg[2];
    M.start = msg[3];
    M.len = msg[4];
    M.bits[0] = msg[5];
    M.bits[1] = msg[6];
    M.bits[2] = msg[7];
    M.lts = msg[8];
    M.idx = msg[9];
    msg_table[counter] = M;
    return counter++;
}

void get_msg_type_details(char* buffer) {
    uint8_t s;

    //Get the details for the medium speed message
    med.id = buffer[1];
    med.data_len = buffer[2];
    s = buffer[3];
    med.speed = 10 * (int)s;
    med.num_messages = buffer[4];
    med.len = med.data_len + 3;

    printf("Med\n");
    printf("id: %c\n", med.id);
    printf("data_len: %u\n", med.data_len);
    printf("speed: %i\n", med.speed);
    printf("num_messages: %u\n", med.num_messages);

    //Get the details for the fast speed message
    fast.id = buffer[5];
    fast.data_len = buffer[6];
    s = buffer[7];
    fast.speed = 10 * (int)s;
    fast.num_messages = buffer[8];
    fast.len = fast.data_len + 3;

    printf("Fast\n");
    printf("id: %c\n", fast.id);
    printf("data_len: %u\n", fast.data_len);
    printf("speed: %i\n", fast.speed);
    printf("num_messages: %u\n", fast.num_messages);

    //Get the details for the stat speed message
    stat.id = buffer[9];
    stat.data_len = buffer[10];
    s = buffer[11];
    stat.speed = 10 * (int)s;
    stat.num_messages = buffer[12];
    stat.len = stat.data_len + 3;

    printf("Stat\n");
    printf("id: %c\n", stat.id);
    printf("data_len: %u\n", stat.data_len);
    printf("speed: %i\n", stat.speed);
    printf("num_messages: %u\n", stat.num_messages);
}

int main() {
    //unsigned char msg[] = {0x01,0x91, 0x66, 0x06, 0x08, 0xFF,0x00, 0x00, 0x08 };

    FILE *set_file;
    char buffer[255];
    char *filename = "settings.bin";
    struct can_msg table[2];

    set_file = fopen(filename, "r");
    if (set_file == NULL) {
        perror("Error opening file");
        return 1;
    }

    //Get number of messages
    fread(buffer, 1, 13, set_file);
    uint8_t num_msgs = buffer[0];
    printf("num messages: %u\n", num_msgs);

    //Get message types
    get_msg_type_details(buffer);

    //get individual message details
    for (int count = 0; count < num_msgs; count++) {
        fread(buffer, 1, 10, set_file);
        /*for (int i = 0; i < 9; i++) {
            printf("%x", buffer[i]);
        }*/
        process_line(buffer, table, count);


        /*printf("\n");
        printf("id: %02x%02x\n", table[count].id[0], table[count].id[1]);
        printf("pkt: %c\n", table[count].pkt);
        printf("start: %u\n", table[count].start);
        printf("len: %u\n", table[count].len);
        printf("bits: %02x%02x%02x\n", table[count].bits[0], table[count].bits[1], table[count].bits[2]);
        printf("lts: %u\n", table[count].lts);
        printf("idx: %u\n", table[count].idx);*/
    }

    fclose(set_file);

    printf("\nMessage Information Loaded\n");



    return 0;
}