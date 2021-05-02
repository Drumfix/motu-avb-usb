/* This file contains a rough description of the host-to-device commands in form of pseudo structs.
   Sequence number used by host is initialized using the Host_Init message.
   The host uses 2 different sequence numbers for KERN and HTTP messages, but both start from the init value
   The device uses sequence numbers 0x40 - 0x7f
   The crc32 is calculated over the bytes starting with the byte at the U of UTOM until the end of the message
   (verified using the crc32 function from zlib)
*/
 
typedef struct {
    uint8_t sequence_number;      // 0x20  <- initial sequence number for KERN/HTTP commands
    uint8_t cmd;                  // 0x82 = init
    uint16_t size_1;              // 0x04
} Host_Init;

typedef struct {
    uint8_t sequence_number;      // sequence number  (0x20 - 0x3f)
    uint8_t cmd;                  // 0x81 = ack
    uint16_t size_1;              // 0x04
} Host_Ack;

typedef struct {
    uint8_t sequence_number;      // sequence number  (0x20 - 0x3f)
    uint8_t cmd;                  // 0x80
    uint16_t size_1;              // total size of message including header

    char header[4];               // NERK
    uint32_t unknown_1;           // = crc32
    uint32_t unknown_2;           // = sequence number KERN
    uint32_t unknown_3;           // = 1
    uint16_t zero_1;              // = frame number (start = 0 for messages that span more than 4K)
    uint16_t size_2;              // size of message including MOTU
    char motu[4];                 // UTOM
    uint32_t unknown_4;           // = 8
    uint32_t unknown_5;           // = 1
    uint32_t unknown_6;           // = 0

    uint32_t size_3;              // size of next submessage rest of message without size_3 word
    uint32_t word_size;           // = 4
    char     command[3];          // = GET
    uint32_t word_size_2;         // = strlen
    char     command_param[10];
    uint32_t num_1;               // = 1

    uint32_t word_size_3;
    char     nomatch[13];
    uint32_t word_size_4;
    char     nomatch_param[1];
    uint32_t num_2;               // 0

} Host_Kern_Get_Command;

typedef struct {
    uint8_t sequence_number;      // sequence number  (0x20 - 0x3f)
    uint8_t cmd;                  // 0x80
    uint16_t size_1;              // total size of message including header

    char header[4];               // PTTH
    uint32_t unknown_1;           // = crc32
    uint32_t unknown_2;           // = sequence number HTTP
    uint32_t unknown_3;           // = 1
    uint16_t zero_1;              // = frame number (start = 0 for messages that span more 4K)
    uint16_t size_2;              // size of message including MOTU
    char motu[4];                 // UTOM
    uint32_t unknown_4;           // = 8
    uint32_t unknown_5;           // = 1
    uint32_t unknown_6;           // = 0

    uint32_t size_3;              // size of next submessage = rest of message without size_3 word
    uint32_t word_size;           // = strlen("POST")
    char     command[4];          // = POST
    uint32_t word_size_2;         // = strlen("datastore ...")
    char     command_param[...];  // = "datastore..."
    uint32_t num_1;               // = 1

    uint32_t word_size_3;         // = strlen("Unsecure-Auth-MOTU")
    char     auth[18];            // = Unsecure-Auth-MOTU
    uint32_t word_size_4;         // = strlen("unicorn666")
    char     auth_param[10];      // = unicorn666
    uint32_t num_2;               // 0

    uint32_t size_4;              // size of next submessage rest of message without size_4 word
    uint32_t word_size_5;         // strlen("JSON")
    char     json[4];             // JSON
    uint32_t word_size_6;         // strlen("{"value": "..."}
    char     json_param[16];      // {"value": "..."}

} Host_Http_Post_Command;


typedef struct {
    uint8_t sequence_number;      // sequence number  (0x20 - 0x3f)
    uint8_t cmd;                  // 0x80
    uint16_t size_1;              // total size of message including header

    char header[4];               // NREK
    uint32_t unknown_1;           // = crc32
    uint32_t unknown_2;           // = sequence number KERN
    uint32_t unknown_3;           // = 1
    uint16_t zero_1;              // = frame number (start = 0 for messages that span more than 4K)
    uint16_t size_2;              // size of message including MOTU
    char motu[4];                 // UTOM
    uint32_t unknown_4;           // = 8
    uint32_t unknown_5;           // = 1
    uint32_t unknown_6;           // = 0

    uint32_t size_3;              // size of next submessage rest of message without size_3 word
    uint32_t word_size;
    char     command[4];
    uint32_t word_size_2;
    char     parameter[...];
    uint32_t num;                 // = 1
    uint32_t word_size_3;
    char     subcommand[...];
    uint32_t word_size_4;
    char     subcommand_param[...];
    uint32_t word_size_5;         // 0

    uint32_t size_4;              // size of rest of message without size_3 word
    uint32_t word_size_6;
    char     command_2[...];
    uint32_t word_size_7;
    char     parameter_2[...];
     
} Host_Kern_Command;


