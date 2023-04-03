
#include "string.h"

void error(const char * message){
	Serial.printlnf("FATAL ERROR: %s", message);
	delay(100);
	System.enterSafeMode();
}	

#define LED_PIN D0
#define SENSOR_TRIGGER_PIN D2
#define SENSOR_ECHO_PIN D1

#define PULSE_TIMEOUT 0xFFFFFFFF

unsigned int readPulse(unsigned int pin, unsigned int value, unsigned int timeout){
	unsigned int start = micros();
	
	if( pinReadFast(pin) != value ){
		if( readPulse(pin, !value, timeout) == PULSE_TIMEOUT )
			return PULSE_TIMEOUT;
	}
	
	while( pinReadFast(pin) == value ){
		if( micros() - start >= timeout )
			return PULSE_TIMEOUT;
	}
	
	return micros() - start;
}

unsigned int readSensor(){
	digitalWrite(SENSOR_TRIGGER_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(SENSOR_TRIGGER_PIN, LOW);
	
	unsigned int delayTime = readPulse(SENSOR_ECHO_PIN, HIGH, 40000);
	unsigned int distance = delayTime; // cm
	
	return distance;
}

////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////

TCPClient client;

#define OUTGOING_BUFFER_SIZE 1024

uint8_t outgoingBuffer[OUTGOING_BUFFER_SIZE];
unsigned int outgoingBufferLength = 0;


void pushNibbles(uint8_t x, uint8_t y){
	outgoingBuffer[outgoingBufferLength++] = ((x & 0xF) << 4) | (y & 0xF);
}

void pushByte(uint8_t x){
	outgoingBuffer[outgoingBufferLength++] = x;
}

void pushShort(uint16_t x){
	outgoingBuffer[outgoingBufferLength++] = (x >> 8) & 0xFF;
	outgoingBuffer[outgoingBufferLength++] = x & 0xFF;
}

void pushBytes(uint8_t * array, unsigned int length){
	memcpy(outgoingBuffer + outgoingBufferLength, array, length);
	outgoingBufferLength += length;
}

void pushString(const char * str){
	unsigned int length = strlen(str);
	pushBytes((uint8_t*)str, length);
}

void pushWideString(const char * str){
	unsigned int length = strlen(str);
	pushShort(length);
	pushBytes((uint8_t*)str, length);
}


void flushOutPacketBuffer(){
    
	Serial.printlnf("Sending %u bytes", outgoingBufferLength);
	Serial.print("HEX: ");
	for(int i = 0; i < outgoingBufferLength; i++){
		Serial.printf("%02x ", outgoingBuffer[i]);
	}
	Serial.println();
	Serial.print("ASCII: ");
	for(int i = 0; i < outgoingBufferLength; i++){
	    if(outgoingBuffer[i] >= 32 && outgoingBuffer[i] <= 127)
		    Serial.printf("%c", outgoingBuffer[i]);
		else
		    Serial.print(".");
	}
	Serial.println();
	Serial.println();
	
	client.write(outgoingBuffer, outgoingBufferLength);
	outgoingBufferLength = 0;
}

#define INCOMING_BUFFER_SIZE 1024

uint8_t incomingBuffer[INCOMING_BUFFER_SIZE];
unsigned int incomingBufferHead = 0;
unsigned int incomingBufferTail = 0;

unsigned int incomingBufferLength(){
    return incomingBufferTail - incomingBufferHead;
}

void receiveAvailableBytes(){
    unsigned int start = incomingBufferTail;
    
    if(client.available()){
        while(client.available()){
            incomingBuffer[incomingBufferTail] = client.read();
        
            if(incomingBufferTail == INCOMING_BUFFER_SIZE - 1){
                incomingBufferTail = 0;
            } else {
                incomingBufferTail++;
            }
        }
    }
    
    if(incomingBufferTail - start == 0) return;
    
    Serial.printlnf("Received %u bytes", incomingBufferTail - start);
	Serial.print("HEX: ");
	for(int i = start; i < incomingBufferTail; i++){
		Serial.printf("%02x ", incomingBuffer[i]);
	}
	Serial.println();
	Serial.print("ASCII: ");
	for(int i = start; i < incomingBufferTail; i++){
	    if(incomingBuffer[i] >= 32 && incomingBuffer[i] <= 127)
		    Serial.printf("%c", incomingBuffer[i]);
		else
		    Serial.print(".");
	}
	Serial.println();
	Serial.println();
}

uint8_t popByte(){
    
    if(incomingBufferLength() == 0) error("Read too many bytes.");
    
    uint8_t b = incomingBuffer[incomingBufferHead];
    if(incomingBufferHead == INCOMING_BUFFER_SIZE - 1){
        incomingBufferHead = 0;
    } else {
        incomingBufferHead++;
    }
    return b;
}

uint16_t popShort(){
    uint16_t s = 0;
    s |= popByte() << 8;
    s |= popByte();
    return s;
}

String popWideString(){
    uint16_t stringLength = popShort();
    String s;
    s.reserve(stringLength);
    for(int i = 0; i < stringLength && i < 128; i++){
        s += (char)popByte();
    }
    return s;
}

String popBytes(unsigned int stringLength){
    String s;
    s.reserve(stringLength);
    for(int i = 0; i < stringLength && i < 128; i++){
        s += (char)popByte();
    }
    return s;
}

////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////

String brokerHostname = "broker.emqx.io";
unsigned int brokerPort = 1883;
IPAddress brokerIP;

#define CONNECT_COMMAND 1
#define CONNACK_COMMAND 2
#define PUBLISH_COMMAND 3
#define PUBACK_COMMAND 4
#define PUBREC_COMMAND 5
#define PUBREL_COMMAND 6
#define PUBCOMP_COMMAND 7
#define SUBSCRIBE_COMMAND 8
#define SUBACK_COMMAND 9
#define PINGREQ_COMMAND 12
#define PINGRESP_COMMAND 13
#define DISCONNECT_COMMAND 14

unsigned int globalPacketID = 0;
unsigned int keepAliveInterval;
unsigned int lastKeepAlive;

struct PacketHeader {
	unsigned int commandType : 4;
	unsigned int controlFlags : 4;
	uint32_t length;
	
	unsigned int _bufferLocation = 0;
	
	void encode(){
	    _bufferLocation = outgoingBufferLength;
	    pushNibbles(commandType, controlFlags);
	    pushByte(length);
	}
	
	void fixEncoding(){
	    outgoingBuffer[_bufferLocation + 1] = outgoingBufferLength - _bufferLocation - 2;
	}
	
	static PacketHeader decode(){
	    PacketHeader header;
	    
	    uint8_t x = popByte();
	    header.commandType = (x >> 4) & 0xF;
	    header.controlFlags = x & 0xF;
	    header.length = popByte();
	    
	    return header;
	}
	
	static PacketHeader read(){
	    unsigned int original = incomingBufferHead;
	    PacketHeader header = PacketHeader::decode();
	    incomingBufferHead = original;
	    return header;
	}
};

#define CONFLAG_USERNAME (1<<7)
#define CONFLAG_PASSWORD (1<<6)
#define CONFLAG_WILL_RETAIN (1<<5)
#define CONFLAG_WILL_QOS_SHIFT 3
#define CONFLAG_WILL_FLAG (1<<2)
#define CONFLAG_CLEAN_SESSION (1<<1)

struct ConnectPacket {
	PacketHeader header;
	uint8_t connectionFlags;
	uint16_t keepAlive;
	String clientID;
	
	static ConnectPacket create(String clientID, uint16_t keepAlive = 60){
	    ConnectPacket p;
	    
	    p.connectionFlags = CONFLAG_CLEAN_SESSION;
	    p.keepAlive = keepAlive;
	    p.clientID = clientID;
	    
	    p.header = {CONNECT_COMMAND, 0x00, NULL};
	    
	    return p;
	}

	void encode(){
	    
		header.encode();
		pushWideString("MQTT");
		pushByte(0x04);
		pushByte(connectionFlags);
		pushShort(keepAlive);
		pushWideString(clientID);
		header.fixEncoding();
		
	}
};

struct ConnackPacket {
	PacketHeader header;
	uint8_t flags;
	uint8_t returnCode;
	
	static ConnackPacket decode(){
	    ConnackPacket packet;
	    
	    packet.header = PacketHeader::decode();
	    packet.flags = popByte();
	    packet.returnCode = popByte();
	    
	    return packet;
	}
};

struct SubscribePacket {
	PacketHeader header;
	uint16_t packetID;
	String topicName;
	uint8_t qos;
	
	static SubscribePacket create(String topicName, uint8_t qos = 0){
	    SubscribePacket packet;
	    
	    packet.header = {SUBSCRIBE_COMMAND, 0b0010, NULL};
	    
	    packet.packetID = globalPacketID++;
	    packet.topicName = topicName;
	    packet.qos = qos;
	    
	    return packet;
	}
	
	void encode(){
	    
	    header.encode();
	    
	    pushShort(packetID);
	    pushWideString(topicName);
	    pushByte(qos);
	    
	    header.fixEncoding();
	}
};

struct SubackPacket {
	PacketHeader header;
	uint16_t packetID;
	uint8_t returnCode;
	
	static SubackPacket decode(){
	    SubackPacket packet;
	    
	    packet.header = PacketHeader::decode();
	    packet.packetID = popShort();
	    packet.returnCode = popByte();
	    
	    return packet;
	}
};

struct PublishPacket {
	PacketHeader header;
	String topicName;
	uint16_t packetID;
	String payload;
	
	static PublishPacket create(String topicName, String payload, uint8_t qos = 1){
	    PublishPacket packet;
	    
	    packet.header = {PUBLISH_COMMAND, qos << 1, NULL};
	    packet.topicName = topicName;
	    packet.packetID = globalPacketID++;
	    packet.payload = payload;
	    
	    return packet;
	}
	
	static PublishPacket decode(){
        PublishPacket packet;
        
        packet.header = PacketHeader::decode();
        packet.topicName = popWideString();
        
        bool hasPacketID = ((packet.header.controlFlags >> 1) & 0b11) > 0;
        
        if(hasPacketID) packet.packetID = popShort();
        
        packet.payload = popBytes(
            packet.header.length 
            - (hasPacketID ? sizeof(uint16_t) : 0)
            - packet.topicName.length() 
            - sizeof(packet.packetID)
        );
        
        return packet;
	}
	
	void encode(){
	    header.encode();
	    pushWideString(topicName);
	    pushShort(packetID);
	    pushString(payload);
	    header.fixEncoding();
	}
};

struct PubackPacket {
    PacketHeader header;
    uint16_t packetID;
    
    static PubackPacket decode(){
        PubackPacket packet;
        
        packet.header = PacketHeader::decode();
        packet.packetID = popShort();
        
        return packet;
    }
};

struct PingreqPacket {
    PacketHeader header;
    
    static PingreqPacket create(){
        PingreqPacket packet;
        
        packet.header = {PINGREQ_COMMAND, 0, NULL};
        
        return packet;
    }
    
    void encode(){
        header.encode();
        header.fixEncoding();
    }
};

struct PingrespPacket {
    PacketHeader header;
    
    static PingrespPacket decode(){
        PingrespPacket packet;
        
        packet.header = PacketHeader::decode();
        
        return packet;
    }
};

struct DisconnectPacket {
    PacketHeader header;
    
    static DisconnectPacket create(){
        DisconnectPacket packet;
        
        packet.header = {DISCONNECT_COMMAND, 0, NULL};
        
        return packet;
    }
    
    void encode(){
        header.encode();
        header.fixEncoding();
    }
};

void publishHandler(PublishPacket packet);

unsigned int handlePacket(){
    PacketHeader header = PacketHeader::read();
    
    switch(header.commandType){
        case CONNACK_COMMAND:
            Serial.printlnf("CONNACK %u, %u", header.controlFlags, header.length);
            ConnackPacket::decode();
        break;
        
        case PUBACK_COMMAND:
        case PUBREC_COMMAND:
        case PUBREL_COMMAND:
        case PUBCOMP_COMMAND:
            Serial.printlnf("<- PUB* %u, %u, %u", header.commandType, header.controlFlags, header.length);
            PubackPacket::decode();
        break;
        
        case SUBACK_COMMAND:
            Serial.printlnf("<- SUBACK %u, %u", header.controlFlags, header.length);
            SubackPacket::decode();
        break;
        
        case PUBLISH_COMMAND: {
            PublishPacket packet = PublishPacket::decode();
            
            Serial.printlnf("<- PUBLISH: Message received in topic '%s': %s", 
                packet.topicName.c_str(), 
                packet.payload.c_str()
            );
            
            publishHandler(packet);
            
        } break;
        
        case PINGRESP_COMMAND:
            PingrespPacket::decode();
        break;
        
        default:
            Serial.printlnf("<- ??? %u, %u, %u", header.commandType, header.controlFlags, header.length);
            error("Program does not support handling this command type.");
        break;
    }
    
    return header.commandType;
}

bool awaitPacket(unsigned int targetCommandType){
	unsigned int start = millis();
	while(millis() - start < 10000){
		receiveAvailableBytes();
		while(incomingBufferLength() > 0){
		    if(handlePacket() == targetCommandType){
		        return true;
		    }
		}
	}
	
	Serial.printlnf("Timed out before receiving packet. (command type = %u)", targetCommandType);
	return false;
}

void connect(String clientID, unsigned int keepAlive = 60){
    keepAliveInterval = keepAlive;
    
    ConnectPacket connectPacket = ConnectPacket::create(clientID, keepAlive);
	connectPacket.encode();
	flushOutPacketBuffer();
	
	awaitPacket(CONNACK_COMMAND);
	
	lastKeepAlive = millis();
}

void pingreq(){
    PingreqPacket packet = PingreqPacket::create();
    packet.encode();
    flushOutPacketBuffer();
}

void disconnect(){
    DisconnectPacket disconnectPacket = DisconnectPacket::create();
    disconnectPacket.encode();
    flushOutPacketBuffer();
}

uint16_t publish(String topicName, String payload){
    PublishPacket publishPacket = PublishPacket::create(topicName, payload);
	publishPacket.encode();
	flushOutPacketBuffer();
	
	awaitPacket(PUBACK_COMMAND);
	
	return publishPacket.packetID;
}

void subscribe(String topicName){
    SubscribePacket subscribePacket = SubscribePacket::create(topicName);
	subscribePacket.encode();
	flushOutPacketBuffer();

    awaitPacket(SUBACK_COMMAND);
}

////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////

void blinkLED(unsigned int onDuration, unsigned int offDuration, unsigned int times = 1){
    for(int i = 0; i < times; i++){
        digitalWrite(LED_PIN, HIGH);
    	delay(onDuration);
    	digitalWrite(LED_PIN, LOW);
    	delay(offDuration);
    }
}

String waveTopicName = "SIT210/wave";
String patTopicName = "SIT210/pat";
unsigned int sinceLastWave = 0;
String ownName = "jessargon";

void publishHandler(PublishPacket packet){
    if(packet.topicName == waveTopicName && packet.payload != ownName)
        blinkLED(300, 300, 3);
        
    if(packet.topicName == patTopicName && packet.payload != ownName)
        blinkLED(700, 200, 2);
}

void setup() {
    
	pinMode(LED_PIN, OUTPUT);
	pinMode(SENSOR_TRIGGER_PIN, OUTPUT);
	pinMode(SENSOR_ECHO_PIN, INPUT);
	
	Serial.begin(9600);
	for(int i = 0; i < 32; i++) Serial.println("");
	Serial.println("Setup complete.");
	
	brokerIP = WiFi.resolve(brokerHostname);
	client.connect(brokerIP, brokerPort);
	
	connect("clientjessb", 60);
	subscribe(waveTopicName);
	subscribe(patTopicName);
	
	sinceLastWave = millis();
	
}

void loop() {
    
    if( millis() - lastKeepAlive >= (keepAliveInterval*1000) - (keepAliveInterval*250) ){
        lastKeepAlive = millis();
        pingreq();
    }
    
    receiveAvailableBytes();
    if(incomingBufferLength() > 0)
        handlePacket();
	
	unsigned int distance = readSensor();
	if(distance < 4000 && millis() - sinceLastWave > 1000){
	    sinceLastWave = millis();
	    publish(
	        waveTopicName, 
	        ownName.c_str()
	   );
	}
	
	delay(60);
	
}