#ifndef MOTU_AVB_STRINGS_H
#define MOTU_AVB_STRINGS_H

// used for commands and responses from the device

char http[] = "PTTH";
char kern[] = "NREK";
char motu[] = "UTOM";
char post[] = "POST";
char get[] = "GET";
char unsecure_auth_motu[] = "Unsecure-Auth-MOTU";
char unicorn666[] = "unicorn666";
char json[] = "json";
char value[] = "{\"value\" : \"%s\"}";
char if_non_match[] = "If-None-Match";

// used by device responses

char authorization[] = "Authorization";
char access_control_allow_origin[] = "Access-Control-Allow-Origin";
char access_control_allow_headers[] = "Access-Control-Allow-Headers";
char access_control_expose_headers[] = "Access-Control-Expose-Headers";
char content_type[] = "Content-Type";
char etag[] = "ETag";
char application_json[] = "application/json";

#endif
