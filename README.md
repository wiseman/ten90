# ten90

Packaging the guts of [dump1090](https://github.com/antirez/dump1090)
into a library.  Actually I started with [MacolmRobb's
fork](github.com/MalcolmRobb/dump1090) which has many new features.

The library currently does decoding of Mode A, Mode C and Mode S
binary message data (so you have to have already decoded the radio
signal).

See below for the very preliminary API.


## Building

```bash
$ ./configure
$ make
```

## Run unit tests

```bash
$ make check
```

## Run the dump1090 executable

```bash
$ ./dump1090
```


## API

### Version information

```c
const char* ten90_version()
```

Returns a string containing the library version.

### Contexts

```c
int ten90_context_init(ten90_context *context);
```
Initializes a `ten90_context` object, which most of the other API calls require.

```c
void ten90_context_destroy(ten90_context *context);
```

Cleans up a `ten90_context`, freeing memory, etc.

Example:

```
  ten90_context context;
  ten90_context_init(&context);
  // ...
  ten90_context_destroy(&context);
```

## Decoding messages

```c
int ten90_decode_hex_message(ten90_mode_s_message *mm,
                             char *hex,
                             ten90_context *context);
```

Decodes a message that is in hex string format.  Accepts several different formats:

### AVR "*"
```
*8DC07EE85837F6548E5BF91F71DF;
```

### AVR "@"

_Need example._

int ten90_decode_bin_message(ten90_mode_s_message *mm, char *p, ten90_context *context);
void ten90_decode_mode_a_message(ten90_mode_s_message *mm, int ModeA);
void ten90_decode_mode_s_message(ten90_mode_s_message *mm, unsigned char *msg, ten90_context*);
