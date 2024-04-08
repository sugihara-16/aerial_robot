/*
******************************************************************************
* File Name          : kondo_servo.h
* Description        : kondo servo interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __KONDO_SERVO_H
#define __KONDO_SERVO_H

#include "config.h"
#include <ros.h>
#include <spinal/ServoControlCmd.h>
#include <spinal/ServoState.h>
#include <map>

#define MAX_SERVO_NUM 32
#define KONDO_SERVO_UPDATE_INTERVAL 10
#define KONDO_SERVO_TIMEOUT 1
#define KONDO_SERVO_POSITION_MIN 3500
#define KONDO_SERVO_POSITION_MAX 11500
#define KONDO_SERVO_ANGLE_MIN -2.36
#define KONDO_SERVO_ANGLE_MAX 2.36
#define SERVO_STATE_PUB_INTERVAL 25 //40Hz
#define KONDO_BUFFER_SIZE 512
#define KONDO_POSITION_RX_SIZE 3

#ifdef STM32H7
  uint8_t dma_rx_buf_[KONDO_BUFFER_SIZE] __attribute__((section(".GpsRxBufferSection")));
#else
  uint8_t dma_rx_buf_[KONDO_BUFFER_SIZE];
#endif
  uint32_t rd_ptr_ = 0;

class KondoServo
{
private:
  UART_HandleTypeDef* huart_;
  spinal::ServoStates servo_state_msg_;
  ros::NodeHandle* nh_;
  ros::Subscriber<spinal::ServoControlCmd, KondoServo> kondo_servo_control_sub_;
  ros::Publisher servo_state_pub_;
  uint16_t target_position_[MAX_SERVO_NUM];
  uint16_t current_position_[MAX_SERVO_NUM];
  bool activated_[MAX_SERVO_NUM];
  uint16_t pos_rx_buf_[KONDO_POSITION_RX_SIZE];
  uint32_t servo_state_pub_last_time_;
  uint32_t dma_write_ptr_ ;
  uint32_t pos_rx_ptr_ ;
public:
  ~KondoServo(){}
  KondoServo():
    kondo_servo_control_sub_("kondo_servo_cmd", &KondoServo::servoControlCallback, this),
    servo_state_pub_("kondo_servo_states", &servo_state_msg_)
  {
  }

  void init(UART_HandleTypeDef* huart, ros::NodeHandle* nh)
  {
    huart_ = huart;
    nh_ = nh;

    nh_->subscribe(kondo_servo_control_sub_);
    nh_->advertise(servo_state_pub_);

    __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
    __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
    HAL_HalfDuplex_EnableReceiver(huart_);
    HAL_UART_Receive_DMA(huart, dma_rx_buf_, RX_BUFFER_SIZE);

    memset(dma_rx_buf_, 0, RX_BUFFER_SIZE);
    memset(pos_rx_buf_, 0, KONDO_POSITION_RX_SIZE);

    servo_state_msg_.servos_length = 5;
    servo_state_msg_.servos = new spinal::ServoState[5];
    servo_state_pub_last_time_ = 0;

    pos_rx_ptr_ = 0;
  }

  void update()
  {

    for(int i = 0; i < MAX_SERVO_NUM; i++)
      {
        if(activated_[i])
          {
            writePosCmd(i, target_position_[i]);
          } else
          {
            writePosCmd(i, 0);  //freed
          }
      }
    
    if(HAL_GetTick() - servo_state_pub_last_time_ > SERVO_STATE_PUB_INTERVAL)
      {
        servo_state_pub_last_time_ = HAL_GetTick();
        sendServoState();
      }

  }

  void writePosCmd(int id, uint16_t target_position)
  {
    int tx_size = 3;
    uint8_t tx_buff[tx_size];
    uint8_t ret;

    /* transmit */
    tx_buff[0] = 0x80 + id;
    tx_buff[1] = (uint8_t)((target_position & 0x3f80) >> 7); // higher 7 bits of 14 bits
    tx_buff[2] = (uint8_t)(target_position & 0x007f);        // lower  7 bits of 14 bits
    HAL_HalfDuplex_EnableTransmitter(huart_);
    ret = HAL_UART_Transmit(huart_, tx_buff, tx_size, 1);

    /* receive */
    if(ret == HAL_OK)
      {
        HAL_HalfDuplex_EnableReceiver(huart_);
      }

    /* getting data from Ring Buffer */
    while(true){
      int data = read();
      if(data < 0) break;
      pos_rx_buf_[pos_rx_ptr_] = (uint8_t)data;

      if(pos_rx_ptr_ == 2) registerPos();

      pos_rx_ptr_ ++;
      pos_rx_ptr_ %= KONDO_POSITION_RX_SIZE;
      
    }
  }

  int read()
  {
    /* handle RX Overrun Error */
    if ( __HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE) )
      {
        __HAL_UART_CLEAR_FLAG(huart_,
                              UART_CLEAR_NEF | UART_CLEAR_OREF | UART_FLAG_RXNE | UART_FLAG_ORE);
        HAL_UART_Receive_DMA(huart_, dma_rx_buf_, RX_BUFFER_SIZE); // restart
      }
    dma_write_ptr_ =  (KONDO_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (KONDO_BUFFER_SIZE);

    int c = -1;
    uint32_t tick_start = HAL_GetTick();
    while(true){
      if(rd_ptr_ != dma_write_ptr_)
        {
          c = (int)dma_rx_buf_[rd_ptr_++];
          rd_ptr_ %= KONDO_BUFFER_SIZE;
          return c;
        }

      if ((HAL_GetTick() - tick_start) > KONDO_SERVO_TIMEOUT)
        {
          return c;
        }
    }
  }

  void registerPos()
  {
    int id = (int)(pos_rx_buf_[0] & 0x1f);
    uint16_t current_position = (uint16_t)((0x7f & pos_rx_buf_[1]) << 7) + (uint16_t)(0x7f & pos_rx_buf_[2]);
    current_position_[id] = current_position;
    memset(pos_rx_buf_, 0, KONDO_POSITION_RX_SIZE);
  }

  

  bool available()
  {
    dma_write_ptr_ =  (GPS_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (GPS_BUFFER_SIZE);
    return (rd_ptr_ != dma_write_ptr_);
  }


  void servoControlCallback(const spinal::ServoControlCmd& cmd_msg)
  {
    for(int i = 0; i < cmd_msg.index_length; i++)
      {
        if(0 <= cmd_msg.index[i] && cmd_msg.index[i] < MAX_SERVO_NUM)
          {
            if(KONDO_SERVO_POSITION_MIN <= cmd_msg.angles[i] && cmd_msg.angles[i] <= KONDO_SERVO_POSITION_MAX)
              {
                activated_[cmd_msg.index[i]] = true;
                target_position_[cmd_msg.index[i]] = cmd_msg.angles[i];
              }
            else if(cmd_msg.angles[i] == 0)
              {
                activated_[cmd_msg.index[i]] = false;
              }
          }
      }
  }

  void sendServoState()
  {
    servo_state_msg_.stamp = nh_->now();
    bool send_flag = false;
    for (unsigned int i = 0; i < 5; i++)
      {   
        send_flag = true;
        spinal::ServoState servo;
        servo.index = i;
        servo.angle = current_position_[i];
        servo_state_msg_.servos[i] = servo;

        if(send_flag) servo_state_pub_.publish(&servo_state_msg_);
      }
  }

  void setTargetPos(const std::map<uint16_t, float>& servo_map)
  {
    for(auto servo : servo_map) {
      uint16_t id = servo.first;
      float angle = servo.second;
      uint16_t target_pos = rad2KondoPosConv(angle);

      // temporary command to free servo is angle = 100.0
      if(angle == 100.0)
        {
          activated_[id] = false;
          target_position_[id] = 7500;
          // char buf[100];
          // sprintf(buf, "servo id: %d is freed!", id);
          // nh_->l
        }
      else if(KONDO_SERVO_POSITION_MIN <= target_pos && target_pos <= KONDO_SERVO_POSITION_MAX)
        {
          activated_[id] = true;
          target_position_[id] = target_pos;
        }
    }
  }

  
  uint16_t rad2KondoPosConv(float angle)
  {
    uint16_t kondo_pos = (uint16_t)((KONDO_SERVO_POSITION_MAX-KONDO_SERVO_POSITION_MIN)*(-angle - KONDO_SERVO_ANGLE_MIN)/(KONDO_SERVO_ANGLE_MAX - KONDO_SERVO_ANGLE_MIN) + KONDO_SERVO_POSITION_MIN); //min-max normarization
    return kondo_pos;
  }

  float kondoPos2RadConv(int pos)
  {
    float angle = -(float)((KONDO_SERVO_ANGLE_MAX-KONDO_SERVO_ANGLE_MIN)*(pos - KONDO_SERVO_POSITION_MIN)/(KONDO_SERVO_POSITION_MAX - KONDO_SERVO_POSITION_MIN) + KONDO_SERVO_ANGLE_MIN); //min-max normarization

    return angle;
  }

  void inactivate(int id)
  {
    activated_[id] = false;
  }

  void activate(int id)
  {
    activated_[id] = true;
  }

};

#endif
