#ifndef _MessageProcessor_h_
#define _MessageProcessor_h_

#include <Arduino.h>
#include <functional>
#include "PacketSerial.h"
#include "CRC32.h"

unsigned __exidx_start;
unsigned __exidx_end;

// Request or Response flags
enum ReqResp {
  REQUEST =   0,
  RESPONSE =  1
};

// Command, ACK and NACK flags. 
enum AckNack {
  COMMAND =   0, // The message is a command
  ACK =       1, // The message is an ACK for an earlier command
  NACK =      2, // The message is a NACK for an earlier command
  MISSING =   3, // The sequ num in this header is missing -- got a later value. 
};

// Flow control flags
enum FlowControl {
  OK =      0, // Sender may send normally
  SLOW =    1, // The queue is getting full
  FULL =    2, // The Queue is full; stop sending
  EMPTY =   3, // The queu is empty 
};

enum CommandCode  {
  MOVE =    1,  // a normal movement segment
  RUN =     2,
  STOP =    3,
  DEBUG =   4,  // Payload is a debug message; the next packet is text
  MESSAGE = 5,  // Payload is a message; the next packet is text
  ERROR =   6,  // Some error
  ECHO  =   7   // Echo the incomming header
};

struct Header {
  uint16_t seq;                         // Packet sequence number
  enum CommandCode code : 6;                // Command number
  enum AckNack ack_nack: 2;
  enum ReqResp req_resp: 1;
  enum FlowControl flow_control: 2;
  uint8_t n: 5;                         // Number of axes or messages following header
  uint32_t crc = 0;                     // Payload CRC // 4
}; // 8 bytes

struct AxisHeader {
  uint32_t segment_time = 0; // total segment time, in microseconds // 4
  // Axis number mapping, packed as nibbles. 
  // The value of the nth nibble is the axis number of the nth AxisSegment record
  // The default value is an identiy map. 
  uint32_t axis_numbers = 0x87654321; 
};

struct AxisSegment {
  int32_t steps; // Number of steps to move the axis
  uint32_t v0; // Initial segment velocity
  uint32_t v1; // Final velocity
};

enum  MoveCommandState {
  WAITING_COMMAND_HEADER, WAITING_AXIS_HEADER, WAITING_AXIS_SEGMENT, DONE
};

enum RecieveState {
  WAITING_HEADER, BUILDING_MESSAGE
};

#define MESSAGE_BUF_SIZE 254
#define PRINTF(...) snprintf((this->message), (MESSAGE_BUF_SIZE),__VA_ARGS__); sendMessage();

class SerialDataProcessor{
public: 

  SerialDataProcessor(Stream &stream) : stream(stream),  ps(stream) {
    
    std::function<void(const uint8_t* buffer, size_t size)> f = [this](const uint8_t* buffer, size_t size) {
      this->processPacket(buffer, size);
    };

    ps.setPacketHandler(f);

  }

  void update() {
    ps.update();
  }

  void sendMessage(){
    Serial.println(message);

    Header* h = resetHeader(0,CommandCode::MESSAGE);
    h->n = 1;
    //ps.send((const uint8_t*)h, sizeof(Header));
    //ps.send((const uint8_t*)message, strlen(message));

  }

  void sendHeader(){
    out_header.crc = 0;
    out_header.crc = CRC32::calculate((const uint8_t*)&out_header, sizeof(out_header));
    ps.send((const uint8_t*)&out_header, sizeof(out_header));
  }

  void sendAck(uint16_t seq, enum CommandCode code){
    Header* h = resetHeader(seq,code);
    h->ack_nack = AckNack::ACK;
    h->seq = seq;
    sendHeader();
  }

  void sendNack(uint16_t seq, enum CommandCode code){
    Header* h = resetHeader(seq,code);
    h->ack_nack = AckNack::NACK;
    h->seq = seq;
    sendHeader();
  }

  Header* resetHeader(uint16_t seq, enum CommandCode code){
    out_header.seq = seq;
    out_header.code = code;
    out_header.ack_nack = AckNack::COMMAND;
    out_header.req_resp = ReqResp::RESPONSE;
    out_header.flow_control = FlowControl::OK;
    out_header.n = 0;
    return &out_header;
  }

  // Top Level packet processing
  void processPacket(const uint8_t* buffer, size_t size){

    if (recieve_state == RecieveState::WAITING_HEADER){

      if (size != sizeof(Header)){
        PRINTF("Expected header, but message not correct size");
        return;
      }

      memcpy(&last_header, (Header*)buffer, sizeof(Header));

      int lh_crc = last_header.crc;
      last_header.crc = 0;
      int crc = CRC32::calculate((const uint8_t*)&last_header, sizeof(last_header));
      last_header.crc = crc;

      if (lh_crc != crc){
        PRINTF("Bad CRC for header %d!=%d", lh_crc, crc);
        return;
      }

      remaining_parts = 0;
      move_command.state = MoveCommandState::WAITING_COMMAND_HEADER;

      sendAck(last_header.seq, last_header.code);

    }

    if (last_header.code == CommandCode::MOVE){
      processMoveCommand(buffer, size);
    } else if(last_header.code == CommandCode::ECHO){
      processEchoCommand(buffer, size);
    } else {
      PRINTF("Unknown header code");
    }

    if (move_command.state == MoveCommandState::DONE){
      // Do something with the finished move command
      recieve_state = RecieveState::WAITING_HEADER;
      processFinishedMoveCommand();
    }
  }

  void processEchoCommand(const uint8_t* buffer, size_t size){

    // Echo the header
    if(recieve_state == RecieveState::WAITING_HEADER){
      
      if (last_header.n > 0){
        // There are additional echo parts, so prepare to recieve them. 
        remaining_parts = last_header.n;
        recieve_state = RecieveState::BUILDING_MESSAGE;
      } else {
        remaining_parts = 0; 
        recieve_state = RecieveState::WAITING_HEADER; // Just to be explicit
      }

      PRINTF("Sending Echo header remaining=%d size=%d", remaining_parts, size)
      
      ps.send((const uint8_t*)&last_header, sizeof(Header));
    // Echo all of the remaining parts
    } else {
      if (--remaining_parts == 0){
        recieve_state = RecieveState::WAITING_HEADER;
      }
      PRINTF("Sending Echo payload remaining=%d size=%d", remaining_parts, size);
      ps.send(buffer,size);
    }
  }

  void processMoveCommand(const uint8_t* buffer, size_t size){

    if(move_command.state == WAITING_COMMAND_HEADER){
      move_command.axis_index = 0;

      if (last_header.n > 0){
        move_command.state = MoveCommandState::WAITING_AXIS_HEADER;
      } else {
        // No axes, so already done. 
        move_command.state = MoveCommandState::WAITING_COMMAND_HEADER;
      }

      recieve_state = RecieveState::BUILDING_MESSAGE;

    } else if(move_command.state == WAITING_AXIS_HEADER){

      if (size != sizeof(AxisHeader)){
        PRINTF("Expected axis header, but message not correct size");
        return;
      }

      memcpy(&move_command.axis_header, (AxisHeader*)buffer, sizeof(AxisHeader));
      move_command.state = MoveCommandState::WAITING_AXIS_SEGMENT;
      
    } else if (move_command.state == WAITING_AXIS_SEGMENT){

      if (size != sizeof(AxisSegment)){
        PRINTF("Expected axis segment, but message not correct size");
        return;
      }

      memcpy(&move_command.axes[axis_index], (AxisSegment*)buffer, sizeof(AxisSegment));
      axis_index++;

      if (axis_index > last_header.n){
        move_command.state = MoveCommandState::DONE; // Done!
      }
    }
  }

  void processFinishedMoveCommand(){

    move_command.state = MoveCommandState::WAITING_COMMAND_HEADER;

  }

  void processOtherCommand(){

  }


  void processMove(){

  }

private:


  Stream& stream;
  PacketSerial ps;

  int axis_index = -1; // Axis number when loading axis records. 

  RecieveState recieve_state = RecieveState::WAITING_HEADER;

  Header out_header; // For outgoing messages

  Header last_header;
  int remaining_parts = 0; // Number of additional non-header components

  char message[MESSAGE_BUF_SIZE]; // For sprintf debug messages

  struct MoveCommand {
    MoveCommandState state = MoveCommandState::WAITING_COMMAND_HEADER;
    AxisHeader axis_header;
    uint8_t axis_index = 0;
    AxisSegment axes[8];
  } move_command;

};


#endif // _MessageProcessor_h_